"""
Real-time wheelchair control logic.

This module does NOT start automatically when imported.

Normal use:

    run(
        stop_requested=...,
        log=...
    )

where stop_requested() returns True when chair control should stop.

Inputs:
- steering joystick: GP27 / ADC1
- throttle joystick: GP28 / ADC2
- mode selector: GP4 and GP7

Speed selection:
- The backward half of the throttle joystick is RESERVED
  exclusively for speed selection.
- Pull backward past the switching threshold.
- Hold for SPEED_CHANGE_HOLD_MS.
- Return to neutral.
- Speed then advances:
      LOW -> MEDIUM -> HIGH -> LOW

Backward joystick movement NEVER commands the motors.

Outputs:
- steering servo: GP10
- motor-controller PWM:
    BR: GP17
    BL: GP18
    FR: GP14
    FL: GP15
- status LED: GP2

Modes:
- TERRAIN
- SNOW
- ROAD

Speeds:
- LOW
- MEDIUM
- HIGH

Speed LED:
- LOW:    solid ON
- MEDIUM: slow blink
- HIGH:   fast blink

main.py may use the status LED during startup/network connection.
Once run() starts, this module takes over the LED for speed
indication.

When run directly:
    The real chair control loop is started as a self-test.
    Ctrl-C stops all outputs.
"""

from machine import ADC, Pin, PWM
from time import sleep_ms, ticks_ms, ticks_diff


# ============================================================
# GENERAL
# ============================================================

LOOP_PERIOD_MS = 20

# Print diagnostics every 10 control loops.
# At 20 ms per loop this is approximately 5 Hz.
DIAGNOSTIC_INTERVAL_LOOPS = 10


# ============================================================
# JOYSTICK
# ============================================================

x_adc = ADC(27)
y_adc = ADC(28)

# Steering calibration
X_LEFT = 52
X_CENTER = 35000
X_RIGHT = 65393
X_DEADZONE = 1000

# Throttle calibration
Y_BACKWARD = 6632
Y_CENTER = 44581
Y_FORWARD = 65384
Y_DEADZONE = 1000

Y_FORWARD_FULL = Y_FORWARD


def get_x_percentage(x):

    if abs(x - X_CENTER) <= X_DEADZONE:
        return 0.0

    if x < X_CENTER:

        x = max(
            x,
            X_LEFT
        )

        fraction = (
            (X_CENTER - X_DEADZONE) - x
        ) / (
            (X_CENTER - X_DEADZONE) - X_LEFT
        )

        fraction = max(
            0.0,
            min(1.0, fraction)
        )

        return -100.0 * fraction

    else:

        x = min(
            x,
            X_RIGHT
        )

        fraction = (
            x - (X_CENTER + X_DEADZONE)
        ) / (
            X_RIGHT - (X_CENTER + X_DEADZONE)
        )

        fraction = max(
            0.0,
            min(1.0, fraction)
        )

        return 100.0 * fraction


def get_y_percentage(y):

    if abs(y - Y_CENTER) <= Y_DEADZONE:
        return 0.0

    if y < X_CENTER:

        y = max(
            y,
            Y_BACKWARD
        )

        fraction = (
            (Y_CENTER - Y_DEADZONE) - y
        ) / (
            (Y_CENTER - Y_DEADZONE) - Y_BACKWARD
        )

        fraction = max(
            0.0,
            min(1.0, fraction)
        )

        return -100.0 * fraction

    else:

        y = min(
            y,
            Y_FORWARD
        )

        fraction = (
            y - (Y_CENTER + Y_DEADZONE)
        ) / (
            Y_FORWARD - (Y_CENTER + Y_DEADZONE)
        )

        fraction = max(
            0.0,
            min(1.0, fraction)
        )

        return 100.0 * fraction


# ============================================================
# MODE SELECTOR
# ============================================================

mode_gp4 = Pin(
    4,
    Pin.IN,
    Pin.PULL_UP
)

mode_gp7 = Pin(
    7,
    Pin.IN,
    Pin.PULL_UP
)

current_mode = "TERRAIN"


def read_mode():

    global current_mode

    p4 = mode_gp4.value()
    p7 = mode_gp7.value()

    if p7 == 1 and p4 == 1:

        current_mode = "TERRAIN"

    elif p7 == 0 and p4 == 1:

        current_mode = "SNOW"

    elif p7 == 1 and p4 == 0:

        current_mode = "ROAD"

    # p4 == 0 and p7 == 0 is invalid.
    # Retain the previous valid mode.

    return current_mode


# ============================================================
# SPEED SELECTION USING BACKWARD JOYSTICK
# ============================================================

# Speed always starts at LOW when run() begins.
current_speed = "LOW"

SPEED_ORDER = (
    "LOW",
    "MEDIUM",
    "HIGH",
)

# A deliberate backward pull is required.
#
# 0.70 means the joystick must be at least 70% of the way
# from neutral to the calibrated full-backward position.
#
# The entire backward side remains unavailable to the motors,
# regardless of this threshold.
SPEED_CHANGE_BACKWARD_FRACTION = 0.70

# Backward position must be held for this long before it becomes
# an armed speed-change request.
SPEED_CHANGE_HOLD_MS = 500

# After the hold succeeds, the joystick must return into the
# ordinary throttle neutral region before the speed changes.
#
# This gives us a deliberate:
#
#     pull -> hold -> release
#
# gesture rather than changing speed merely by crossing a point.
SPEED_CHANGE_NEUTRAL_LIMIT = (
    Y_CENTER - Y_DEADZONE
)

SPEED_CHANGE_BACKWARD_THRESHOLD = int(
    Y_CENTER
    - SPEED_CHANGE_BACKWARD_FRACTION
    * (
        Y_CENTER
        - Y_BACKWARD
    )
)

speed_change_hold_start_ms = None
speed_change_armed = False


def reset_speed_change_state():

    global speed_change_hold_start_ms
    global speed_change_armed

    speed_change_hold_start_ms = None
    speed_change_armed = False


def advance_speed():

    global current_speed

    index = SPEED_ORDER.index(
        current_speed
    )

    index = (
        index + 1
    ) % len(
        SPEED_ORDER
    )

    current_speed = SPEED_ORDER[
        index
    ]

    return current_speed


def update_speed_selection(y):

    """
    Handle deliberate speed changes using backward joystick travel.

    Sequence:

        1. Pull backward beyond SPEED_CHANGE_BACKWARD_THRESHOLD.
        2. Hold for SPEED_CHANGE_HOLD_MS.
        3. Return to neutral.
        4. Speed advances one step.

    Pulling backward briefly does nothing.

    Holding backward indefinitely changes nothing until the
    joystick is released.

    After a speed change, another complete pull-hold-release
    gesture is required.
    """

    global speed_change_hold_start_ms
    global speed_change_armed

    now = ticks_ms()

    # --------------------------------------------------------
    # ALREADY ARMED
    # --------------------------------------------------------

    if speed_change_armed:

        # Wait for joystick to return to the neutral region.
        if y >= SPEED_CHANGE_NEUTRAL_LIMIT:

            new_speed = advance_speed()

            speed_change_armed = False
            speed_change_hold_start_ms = None

            return new_speed

        return None

    # --------------------------------------------------------
    # BACKWARD PULL
    # --------------------------------------------------------

    if y <= SPEED_CHANGE_BACKWARD_THRESHOLD:

        if speed_change_hold_start_ms is None:

            speed_change_hold_start_ms = now

        elif (
            ticks_diff(
                now,
                speed_change_hold_start_ms
            )
            >= SPEED_CHANGE_HOLD_MS
        ):

            # Gesture is now armed.
            #
            # Do NOT change speed yet.
            # Require release to neutral first.
            speed_change_armed = True

        return None

    # --------------------------------------------------------
    # PULL ABORTED BEFORE HOLD COMPLETED
    # --------------------------------------------------------

    speed_change_hold_start_ms = None

    return None


# ============================================================
# STATUS / SPEED LED
# ============================================================

# main.py may use this LED while starting Wi-Fi / maintenance.
# Once run() starts, chair_logic owns it for speed indication.

status_led = Pin(
    2,
    Pin.OUT
)

# MEDIUM:
# 500 ms ON / 500 ms OFF
MEDIUM_LED_HALF_PERIOD_MS = 500

# HIGH:
# 125 ms ON / 125 ms OFF
HIGH_LED_HALF_PERIOD_MS = 125

led_last_toggle_ms = ticks_ms()
led_state = False


def initialise_speed_led():

    global led_last_toggle_ms
    global led_state

    led_last_toggle_ms = ticks_ms()

    if current_speed == "LOW":

        led_state = True
        status_led.value(1)

    else:

        led_state = False
        status_led.value(0)


def update_speed_led():

    global led_last_toggle_ms
    global led_state

    now = ticks_ms()

    # --------------------------------------------------------
    # LOW = SOLID ON
    # --------------------------------------------------------

    if current_speed == "LOW":

        if not led_state:

            led_state = True
            status_led.value(1)

        led_last_toggle_ms = now

        return

    # --------------------------------------------------------
    # SELECT BLINK RATE
    # --------------------------------------------------------

    if current_speed == "MEDIUM":

        half_period_ms = (
            MEDIUM_LED_HALF_PERIOD_MS
        )

    else:

        # HIGH
        half_period_ms = (
            HIGH_LED_HALF_PERIOD_MS
        )

    # --------------------------------------------------------
    # BLINK
    # --------------------------------------------------------

    if (
        ticks_diff(
            now,
            led_last_toggle_ms
        )
        >= half_period_ms
    ):

        led_state = not led_state

        status_led.value(
            1 if led_state else 0
        )

        led_last_toggle_ms = now


# ============================================================
# MOTOR SETTINGS
# ============================================================

MIN_MOTOR_VOLTAGE = 1.60

ROAD_MAX_VOLTAGE = {
    "LOW": 1.90,
    "MEDIUM": 2.20,
    "HIGH": 3.30,
}

SNOW_MAX_VOLTAGE = {
    "LOW": 1.90,
    "MEDIUM": 2.20,
    "HIGH": 3.30,
}

TERRAIN_FRONT_MAX_VOLTAGE = 1.80

TERRAIN_REAR_MULTIPLIER = {
    "LOW": 1.00,
    "MEDIUM": 1.2,
    "HIGH": 1.5,
}

THROTTLE_EXPONENT = 2.0
MIN_VOLTAGE_REGION = 0.05


# ============================================================
# MOTOR ACCELERATION
# ============================================================

MOTOR_MAX_SLEW_STEP_V = 0.010
MOTOR_START_SLEW_STEP_V = 0.0005
MOTOR_SLEW_ACCELERATION_V = 0.00005

motor_current_v = 0.0

motor_accel_step_v = (
    MOTOR_START_SLEW_STEP_V
)


# ============================================================
# MOTOR PWM
# ============================================================

PICO_PWM_VOLTAGE = 3.3
MOTOR_PWM_FREQUENCY = 20000

motor_BR = PWM(
    Pin(17)
)

motor_BL = PWM(
    Pin(18)
)

motor_FR = PWM(
    Pin(14)
)

motor_FL = PWM(
    Pin(15)
)

REAR_MOTOR_PWMS = (
    motor_BR,
    motor_BL,
)

FRONT_MOTOR_PWMS = (
    motor_FR,
    motor_FL,
)

MOTOR_PWMS = (
    motor_BR,
    motor_BL,
    motor_FR,
    motor_FL,
)

for pwm in MOTOR_PWMS:

    pwm.freq(
        MOTOR_PWM_FREQUENCY
    )

    pwm.duty_u16(0)


def voltage_to_duty(voltage):

    voltage = max(
        0.0,
        min(
            PICO_PWM_VOLTAGE,
            voltage
        )
    )

    return int(
        round(
            voltage
            / PICO_PWM_VOLTAGE
            * 65535
        )
    )


def set_motor_voltages(
    front_voltage,
    rear_voltage
):

    if front_voltage < MIN_MOTOR_VOLTAGE:
        front_voltage = 0.0

    if rear_voltage < MIN_MOTOR_VOLTAGE:
        rear_voltage = 0.0

    front_voltage = min(
        PICO_PWM_VOLTAGE,
        front_voltage
    )

    rear_voltage = min(
        PICO_PWM_VOLTAGE,
        rear_voltage
    )

    front_duty = voltage_to_duty(
        front_voltage
    )

    rear_duty = voltage_to_duty(
        rear_voltage
    )

    for pwm in FRONT_MOTOR_PWMS:

        pwm.duty_u16(
            front_duty
        )

    for pwm in REAR_MOTOR_PWMS:

        pwm.duty_u16(
            rear_duty
        )


def set_all_motor_voltage(voltage):

    set_motor_voltages(
        voltage,
        voltage
    )


# ============================================================
# THROTTLE MAPPING
# ============================================================

def get_throttle_fraction(y):

    # IMPORTANT:
    #
    # Anything at or below the forward edge of neutral produces
    # ZERO motor command.
    #
    # Therefore the entire backward joystick range is reserved
    # for speed selection and can never command the motors.

    throttle_start = (
        Y_CENTER
        + Y_DEADZONE
    )

    if y <= throttle_start:
        return 0.0

    if y >= Y_FORWARD_FULL:
        return 1.0

    fraction = (
        (y - throttle_start)
        / (
            Y_FORWARD_FULL
            - throttle_start
        )
    )

    return max(
        0.0,
        min(
            1.0,
            fraction
        )
    )


def get_front_max_voltage(
    mode,
    speed
):

    if mode == "TERRAIN":

        return TERRAIN_FRONT_MAX_VOLTAGE

    if mode == "SNOW":

        return SNOW_MAX_VOLTAGE[
            speed
        ]

    return ROAD_MAX_VOLTAGE[
        speed
    ]


def joystick_to_motor_voltage(
    y,
    mode,
    speed
):

    throttle = get_throttle_fraction(
        y
    )

    if throttle <= 0.0:
        return 0.0

    if throttle <= MIN_VOLTAGE_REGION:
        return MIN_MOTOR_VOLTAGE

    max_v = get_front_max_voltage(
        mode,
        speed
    )

    remaining_fraction = (
        (
            throttle
            - MIN_VOLTAGE_REGION
        )
        / (
            1.0
            - MIN_VOLTAGE_REGION
        )
    )

    remaining_fraction = max(
        0.0,
        min(
            1.0,
            remaining_fraction
        )
    )

    shaped = (
        remaining_fraction
        ** THROTTLE_EXPONENT
    )

    return (
        MIN_MOTOR_VOLTAGE
        + shaped
        * (
            max_v
            - MIN_MOTOR_VOLTAGE
        )
    )


# ============================================================
# FRONT / REAR MOTOR COMMANDS
# ============================================================

def get_rear_voltage(
    front_voltage,
    mode,
    speed
):

    if front_voltage <= 0.0:
        return 0.0

    if mode != "TERRAIN":
        return front_voltage

    multiplier = (
        TERRAIN_REAR_MULTIPLIER[
            speed
        ]
    )

    rear_voltage = (
        front_voltage
        * multiplier
    )

    return min(
        PICO_PWM_VOLTAGE,
        rear_voltage
    )


# ============================================================
# MOTOR SLEW / SLOW START
# ============================================================

def update_motor_voltage(
    target_v,
    mode,
    speed
):

    global motor_current_v
    global motor_accel_step_v

    if target_v <= 0.0:

        motor_current_v = 0.0

        motor_accel_step_v = (
            MOTOR_START_SLEW_STEP_V
        )

        set_motor_voltages(
            0.0,
            0.0
        )

        return (
            0.0,
            0.0
        )

    if motor_current_v <= 0.0:

        motor_current_v = (
            MIN_MOTOR_VOLTAGE
        )

        motor_accel_step_v = (
            MOTOR_START_SLEW_STEP_V
        )

    elif motor_current_v < target_v:

        difference = (
            target_v
            - motor_current_v
        )

        motor_current_v += min(
            motor_accel_step_v,
            difference
        )

        motor_accel_step_v += (
            MOTOR_SLEW_ACCELERATION_V
        )

        motor_accel_step_v = min(
            motor_accel_step_v,
            MOTOR_MAX_SLEW_STEP_V
        )

    elif motor_current_v > target_v:

        difference = (
            motor_current_v
            - target_v
        )

        motor_current_v -= min(
            MOTOR_MAX_SLEW_STEP_V,
            difference
        )

    motor_current_v = max(
        0.0,
        min(
            PICO_PWM_VOLTAGE,
            motor_current_v
        )
    )

    front_voltage = (
        motor_current_v
    )

    rear_voltage = get_rear_voltage(
        front_voltage,
        mode,
        speed
    )

    set_motor_voltages(
        front_voltage,
        rear_voltage
    )

    return (
        front_voltage,
        rear_voltage
    )


# ============================================================
# STEERING SERVO
# ============================================================

servo = PWM(
    Pin(10)
)

servo.freq(50)

SERVO_LEFT = 1100
SERVO_CENTER = 1500
SERVO_RIGHT = 1900

SERVO_RELEASE_DELAY_MS = 500

STEERING_STEP_US = {
    "TERRAIN": {
        "LOW": 8,
        "MEDIUM": 8,
        "HIGH": 8,
    },
    "SNOW": {
        "LOW": 8,
        "MEDIUM": 4,
        "HIGH": 4,
    },
    "ROAD": {
        "LOW": 8,
        "MEDIUM": 4,
        "HIGH": 4,
    },
}

servo_current_us = SERVO_CENTER
servo_last_active_ms = ticks_ms()
servo_enabled = False


def set_servo_us(pulse_us):

    global servo_enabled

    pulse_us = max(
        SERVO_LEFT,
        min(
            SERVO_RIGHT,
            pulse_us
        )
    )

    duty = int(
        pulse_us
        * 65535
        / 20000
    )

    servo.duty_u16(
        duty
    )

    servo_enabled = True


def disable_servo():

    global servo_enabled

    servo.duty_u16(0)

    servo_enabled = False


def joystick_to_servo(x):

    if abs(
        x - X_CENTER
    ) <= X_DEADZONE:

        return SERVO_CENTER

    if x < X_CENTER:

        x = max(
            x,
            X_LEFT
        )

        fraction = (
            X_CENTER
            - X_DEADZONE
            - x
        ) / (
            X_CENTER
            - X_DEADZONE
            - X_LEFT
        )

        fraction = max(
            0.0,
            min(
                1.0,
                fraction
            )
        )

        pulse = (
            SERVO_CENTER
            - fraction
            * (
                SERVO_CENTER
                - SERVO_LEFT
            )
        )

    else:

        x = min(
            x,
            X_RIGHT
        )

        fraction = (
            x
            - (
                X_CENTER
                + X_DEADZONE
            )
        ) / (
            X_RIGHT
            - (
                X_CENTER
                + X_DEADZONE
            )
        )

        fraction = max(
            0.0,
            min(
                1.0,
                fraction
            )
        )

        pulse = (
            SERVO_CENTER
            + fraction
            * (
                SERVO_RIGHT
                - SERVO_CENTER
            )
        )

    return int(
        pulse
    )


def update_servo(
    target_us,
    mode,
    speed,
    joystick_x
):

    global servo_current_us
    global servo_last_active_ms

    now = ticks_ms()

    joystick_active = (
        abs(
            joystick_x
            - X_CENTER
        )
        > X_DEADZONE
    )

    if joystick_active:

        servo_last_active_ms = now

    max_step = STEERING_STEP_US[
        mode
    ][
        speed
    ]

    difference = (
        target_us
        - servo_current_us
    )

    if difference > max_step:

        servo_current_us += max_step

    elif difference < -max_step:

        servo_current_us -= max_step

    else:

        servo_current_us = target_us

    if (
        not joystick_active
        and
        servo_current_us
        == SERVO_CENTER
        and
        ticks_diff(
            now,
            servo_last_active_ms
        )
        >= SERVO_RELEASE_DELAY_MS
    ):

        disable_servo()

        return int(
            servo_current_us
        )

    set_servo_us(
        servo_current_us
    )

    return int(
        servo_current_us
    )


# ============================================================
# OUTPUT STOP
# ============================================================

def stop_outputs():

    global motor_current_v
    global motor_accel_step_v

    motor_current_v = 0.0

    motor_accel_step_v = (
        MOTOR_START_SLEW_STEP_V
    )

    for pwm in MOTOR_PWMS:

        pwm.duty_u16(0)

    disable_servo()


# ============================================================
# MAIN CONTROL LOOP
# ============================================================

def run(
    stop_requested=None,
    log=print
):

    global servo_current_us
    global servo_last_active_ms
    global motor_current_v
    global motor_accel_step_v
    global current_speed
    global speed_change_hold_start_ms
    global speed_change_armed

    # --------------------------------------------------------
    # RESET CONTROL STATE
    # --------------------------------------------------------

    motor_current_v = 0.0

    motor_accel_step_v = (
        MOTOR_START_SLEW_STEP_V
    )

    set_motor_voltages(
        0.0,
        0.0
    )

    servo_current_us = (
        SERVO_CENTER
    )

    set_servo_us(
        SERVO_CENTER
    )

    servo_last_active_ms = (
        ticks_ms()
    )

    mode = read_mode()

    # Always begin normal chair operation at LOW speed.
    current_speed = "LOW"

    speed_change_hold_start_ms = None
    speed_change_armed = False

    initialise_speed_led()

    throttle_armed = False

    diagnostic_counter = 0

    log(
        "Chair control started. Speed: LOW"
    )

    # --------------------------------------------------------
    # CONTROL LOOP
    # --------------------------------------------------------

    while True:

        loop_start = ticks_ms()

        # ----------------------------------------------------
        # STOP REQUEST
        # ----------------------------------------------------

        if (
            stop_requested is not None
            and
            stop_requested()
        ):

            stop_outputs()

            log("")
            log(
                "Chair control stopped."
            )

            return

        # ----------------------------------------------------
        # READ INPUTS
        # ----------------------------------------------------

        x = x_adc.read_u16()
        y = y_adc.read_u16()

        mode = read_mode()

        # ----------------------------------------------------
        # SPEED SELECTION
        # ----------------------------------------------------

        changed_speed = (
            update_speed_selection(y)
        )

        if changed_speed is not None:

            log(
                "Speed changed to: {}".format(
                    changed_speed
                )
            )

        speed = current_speed

        # ----------------------------------------------------
        # SPEED LED
        # ----------------------------------------------------

        update_speed_led()

        # ----------------------------------------------------
        # STEERING
        # ----------------------------------------------------

        servo_target = (
            joystick_to_servo(x)
        )

        servo_us = update_servo(
            servo_target,
            mode,
            speed,
            x
        )

        # ----------------------------------------------------
        # THROTTLE ARMING
        # ----------------------------------------------------

        if not throttle_armed:

            # Require joystick to be in the neutral/backward
            # region before forward motor control becomes armed.
            if (
                y
                <= Y_CENTER
                + Y_DEADZONE
            ):

                throttle_armed = True

        # ----------------------------------------------------
        # THROTTLE
        # ----------------------------------------------------

        if throttle_armed:

            motor_target_v = (
                joystick_to_motor_voltage(
                    y,
                    mode,
                    speed
                )
            )

        else:

            motor_target_v = 0.0

        front_v, rear_v = (
            update_motor_voltage(
                motor_target_v,
                mode,
                speed
            )
        )

        # ----------------------------------------------------
        # DIAGNOSTICS
        # ----------------------------------------------------

        diagnostic_counter += 1

        if (
            diagnostic_counter
            >= DIAGNOSTIC_INTERVAL_LOOPS
        ):

            diagnostic_counter = 0

            x_percent = (
                get_x_percentage(x)
            )

            y_percent = (
                get_y_percentage(y)
            )

            if speed_change_armed:

                speed_state = (
                    "WAITING FOR RELEASE"
                )

            elif (
                speed_change_hold_start_ms
                is not None
            ):

                speed_state = (
                    "HOLDING"
                )

            else:

                speed_state = (
                    "READY"
                )

            log(
                "X: {:+.1f}% | Y: {:+.1f}% | "
                "Front: {:.3f} V | Rear: {:.3f} V | "
                "Servo: {} | Mode: {} | Speed: {} | "
                "Speed switch: {}".format(
                    x_percent,
                    y_percent,
                    front_v,
                    rear_v,
                    servo_us,
                    mode,
                    speed,
                    speed_state
                )
            )

        # ----------------------------------------------------
        # MAINTAIN 20 ms LOOP
        # ----------------------------------------------------

        elapsed = ticks_diff(
            ticks_ms(),
            loop_start
        )

        remaining = (
            LOOP_PERIOD_MS
            - elapsed
        )

        if remaining > 0:

            sleep_ms(
                remaining
            )


# ============================================================
# SELF-TEST
# ============================================================

def self_test():

    print()
    print("==============================")
    print("CHAIR LOGIC SELF-TEST")
    print("==============================")
    print()

    print(
        "WARNING: REAL CHAIR OUTPUTS ARE ENABLED."
    )

    print(
        "The joystick controls the motors and steering."
    )

    print(
        "GP4 and GP7 select TERRAIN / SNOW / ROAD."
    )

    print()
    print(
        "Backward throttle is reserved for speed selection."
    )

    print(
        "Pull backward past {:.0f}% and hold for {:.1f} s,"
        " then release to neutral.".format(
            SPEED_CHANGE_BACKWARD_FRACTION * 100,
            SPEED_CHANGE_HOLD_MS / 1000
        )
    )

    print(
        "Speed sequence: LOW -> MEDIUM -> HIGH -> LOW."
    )

    print()
    print(
        "LED: LOW=solid, MEDIUM=slow blink, HIGH=fast blink."
    )

    print()
    print(
        "Throttle must be neutral before motor drive arms."
    )

    print()
    print(
        "Press Ctrl-C to stop the self-test."
    )

    print()

    stop_outputs()

    try:

        run(
            stop_requested=None,
            log=print
        )

    except KeyboardInterrupt:

        print()
        print(
            "Self-test interrupted."
        )

    except Exception as error:

        print()
        print(
            "SELF-TEST ERROR: {}".format(
                error
            )
        )

        raise

    finally:

        stop_outputs()

        print(
            "Chair outputs stopped."
        )

        print()
        print("==============================")
        print("CHAIR SELF-TEST FINISHED")
        print("==============================")
        print()


# ============================================================
# RUN DIRECTLY
# ============================================================

if __name__ == "__main__":

    self_test()