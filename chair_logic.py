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
- speed selector: GP20 and GP21

Speed switch:
    LOW:     GP20=1, GP21=0
    MEDIUM:  GP20=1, GP21=1
    HIGH:    GP20=0, GP21=1
    INVALID: GP20=0, GP21=0

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

        x = max(x, X_LEFT)

        fraction = (
            (X_CENTER - X_DEADZONE) - x
        ) / (
            (X_CENTER - X_DEADZONE) - X_LEFT
        )

        fraction = max(0.0, min(1.0, fraction))

        return -100.0 * fraction

    else:

        x = min(x, X_RIGHT)

        fraction = (
            x - (X_CENTER + X_DEADZONE)
        ) / (
            X_RIGHT - (X_CENTER + X_DEADZONE)
        )

        fraction = max(0.0, min(1.0, fraction))

        return 100.0 * fraction


def get_y_percentage(y):

    if abs(y - Y_CENTER) <= Y_DEADZONE:
        return 0.0

    if y < Y_CENTER:

        y = max(y, Y_BACKWARD)

        fraction = (
            (Y_CENTER - Y_DEADZONE) - y
        ) / (
            (Y_CENTER - Y_DEADZONE) - Y_BACKWARD
        )

        fraction = max(0.0, min(1.0, fraction))

        return -100.0 * fraction

    else:

        y = min(y, Y_FORWARD)

        fraction = (
            y - (Y_CENTER + Y_DEADZONE)
        ) / (
            Y_FORWARD - (Y_CENTER + Y_DEADZONE)
        )

        fraction = max(0.0, min(1.0, fraction))

        return 100.0 * fraction


# ============================================================
# MODE SELECTOR
# ============================================================

mode_gp4 = Pin(4, Pin.IN, Pin.PULL_UP)
mode_gp7 = Pin(7, Pin.IN, Pin.PULL_UP)

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

    # Invalid combination retains previous mode.

    return current_mode


# ============================================================
# PHYSICAL SPEED SWITCH
# ============================================================

speed_gp20 = Pin(20, Pin.IN, Pin.PULL_UP)
speed_gp21 = Pin(21, Pin.IN, Pin.PULL_UP)

current_speed = "LOW"


def read_speed():

    """
    Read the physical three-position speed switch.

    LOW:
        GP20 = 1
        GP21 = 0

    MEDIUM:
        GP20 = 1
        GP21 = 1

    HIGH:
        GP20 = 0
        GP21 = 1

    INVALID:
        GP20 = 0
        GP21 = 0

    Invalid combination defaults to LOW.
    """

    global current_speed

    p20 = speed_gp20.value()
    p21 = speed_gp21.value()

    if p20 == 1 and p21 == 0:

        current_speed = "LOW"

    elif p20 == 1 and p21 == 1:

        current_speed = "MEDIUM"

    elif p20 == 0 and p21 == 1:

        current_speed = "HIGH"

    else:

        current_speed = "LOW"

    return current_speed


# ============================================================
# STATUS / SPEED LED
# ============================================================

status_led = Pin(2, Pin.OUT)

MEDIUM_LED_HALF_PERIOD_MS = 500
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

    if current_speed == "LOW":

        if not led_state:

            led_state = True
            status_led.value(1)

        led_last_toggle_ms = now

        return

    if current_speed == "MEDIUM":

        half_period_ms = MEDIUM_LED_HALF_PERIOD_MS

    else:

        half_period_ms = HIGH_LED_HALF_PERIOD_MS

    if ticks_diff(now, led_last_toggle_ms) >= half_period_ms:

        led_state = not led_state

        status_led.value(1 if led_state else 0)

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

motor_accel_step_v = MOTOR_START_SLEW_STEP_V


# ============================================================
# MOTOR PWM
# ============================================================

PICO_PWM_VOLTAGE = 3.3
MOTOR_PWM_FREQUENCY = 20000

motor_BR = PWM(Pin(17))
motor_BL = PWM(Pin(18))
motor_FR = PWM(Pin(14))
motor_FL = PWM(Pin(15))

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

    pwm.freq(MOTOR_PWM_FREQUENCY)
    pwm.duty_u16(0)


def voltage_to_duty(voltage):

    voltage = max(
        0.0,
        min(PICO_PWM_VOLTAGE, voltage)
    )

    return int(
        round(
            voltage / PICO_PWM_VOLTAGE * 65535
        )
    )


def set_motor_voltages(front_voltage, rear_voltage):

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

    front_duty = voltage_to_duty(front_voltage)
    rear_duty = voltage_to_duty(rear_voltage)

    for pwm in FRONT_MOTOR_PWMS:
        pwm.duty_u16(front_duty)

    for pwm in REAR_MOTOR_PWMS:
        pwm.duty_u16(rear_duty)


def set_all_motor_voltage(voltage):

    set_motor_voltages(voltage, voltage)


# ============================================================
# THROTTLE MAPPING
# ============================================================

def get_throttle_fraction(y):

    # Backward joystick movement never commands motors.

    throttle_start = Y_CENTER + Y_DEADZONE

    if y <= throttle_start:
        return 0.0

    if y >= Y_FORWARD_FULL:
        return 1.0

    fraction = (
        (y - throttle_start)
        / (Y_FORWARD_FULL - throttle_start)
    )

    return max(0.0, min(1.0, fraction))


def get_front_max_voltage(mode, speed):

    if mode == "TERRAIN":

        return TERRAIN_FRONT_MAX_VOLTAGE

    if mode == "SNOW":

        return SNOW_MAX_VOLTAGE[speed]

    return ROAD_MAX_VOLTAGE[speed]


def joystick_to_motor_voltage(y, mode, speed):

    throttle = get_throttle_fraction(y)

    if throttle <= 0.0:
        return 0.0

    if throttle <= MIN_VOLTAGE_REGION:
        return MIN_MOTOR_VOLTAGE

    max_v = get_front_max_voltage(mode, speed)

    remaining_fraction = (
        (throttle - MIN_VOLTAGE_REGION)
        / (1.0 - MIN_VOLTAGE_REGION)
    )

    remaining_fraction = max(
        0.0,
        min(1.0, remaining_fraction)
    )

    shaped = remaining_fraction ** THROTTLE_EXPONENT

    return (
        MIN_MOTOR_VOLTAGE
        + shaped * (max_v - MIN_MOTOR_VOLTAGE)
    )


# ============================================================
# FRONT / REAR MOTOR COMMANDS
# ============================================================

def get_rear_voltage(front_voltage, mode, speed):

    if front_voltage <= 0.0:
        return 0.0

    if mode != "TERRAIN":
        return front_voltage

    multiplier = TERRAIN_REAR_MULTIPLIER[speed]

    rear_voltage = front_voltage * multiplier

    return min(
        PICO_PWM_VOLTAGE,
        rear_voltage
    )


# ============================================================
# MOTOR SLEW / SLOW START
# ============================================================

def update_motor_voltage(target_v, mode, speed):

    global motor_current_v
    global motor_accel_step_v

    if target_v <= 0.0:

        motor_current_v = 0.0

        motor_accel_step_v = MOTOR_START_SLEW_STEP_V

        set_motor_voltages(0.0, 0.0)

        return (0.0, 0.0)

    if motor_current_v <= 0.0:

        motor_current_v = MIN_MOTOR_VOLTAGE

        motor_accel_step_v = MOTOR_START_SLEW_STEP_V

    elif motor_current_v < target_v:

        difference = target_v - motor_current_v

        motor_current_v += min(
            motor_accel_step_v,
            difference
        )

        motor_accel_step_v += MOTOR_SLEW_ACCELERATION_V

        motor_accel_step_v = min(
            motor_accel_step_v,
            MOTOR_MAX_SLEW_STEP_V
        )

    elif motor_current_v > target_v:

        difference = motor_current_v - target_v

        motor_current_v -= min(
            MOTOR_MAX_SLEW_STEP_V,
            difference
        )

    motor_current_v = max(
        0.0,
        min(PICO_PWM_VOLTAGE, motor_current_v)
    )

    front_voltage = motor_current_v

    rear_voltage = get_rear_voltage(
        front_voltage,
        mode,
        speed
    )

    set_motor_voltages(
        front_voltage,
        rear_voltage
    )

    return (front_voltage, rear_voltage)


# ============================================================
# STEERING SERVO
# ============================================================

servo = PWM(Pin(10))
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
        min(SERVO_RIGHT, pulse_us)
    )

    duty = int(
        pulse_us * 65535 / 20000
    )

    servo.duty_u16(duty)

    servo_enabled = True


def disable_servo():

    global servo_enabled

    servo.duty_u16(0)

    servo_enabled = False


def joystick_to_servo(x):

    if abs(x - X_CENTER) <= X_DEADZONE:

        return SERVO_CENTER

    if x < X_CENTER:

        x = max(x, X_LEFT)

        fraction = (
            X_CENTER - X_DEADZONE - x
        ) / (
            X_CENTER - X_DEADZONE - X_LEFT
        )

        fraction = max(
            0.0,
            min(1.0, fraction)
        )

        pulse = (
            SERVO_CENTER
            - fraction * (SERVO_CENTER - SERVO_LEFT)
        )

    else:

        x = min(x, X_RIGHT)

        fraction = (
            x - (X_CENTER + X_DEADZONE)
        ) / (
            X_RIGHT - (X_CENTER + X_DEADZONE)
        )

        fraction = max(
            0.0,
            min(1.0, fraction)
        )

        pulse = (
            SERVO_CENTER
            + fraction * (SERVO_RIGHT - SERVO_CENTER)
        )

    return int(pulse)


def update_servo(target_us, mode, speed, joystick_x):

    global servo_current_us
    global servo_last_active_ms

    now = ticks_ms()

    joystick_active = (
        abs(joystick_x - X_CENTER) > X_DEADZONE
    )

    if joystick_active:
        servo_last_active_ms = now

    max_step = STEERING_STEP_US[mode][speed]

    difference = target_us - servo_current_us

    if difference > max_step:

        servo_current_us += max_step

    elif difference < -max_step:

        servo_current_us -= max_step

    else:

        servo_current_us = target_us

    if (
        not joystick_active
        and servo_current_us == SERVO_CENTER
        and ticks_diff(
            now,
            servo_last_active_ms
        ) >= SERVO_RELEASE_DELAY_MS
    ):

        disable_servo()

        return int(servo_current_us)

    set_servo_us(servo_current_us)

    return int(servo_current_us)


# ============================================================
# OUTPUT STOP
# ============================================================

def stop_outputs():

    global motor_current_v
    global motor_accel_step_v

    motor_current_v = 0.0

    motor_accel_step_v = MOTOR_START_SLEW_STEP_V

    for pwm in MOTOR_PWMS:
        pwm.duty_u16(0)

    disable_servo()


# ============================================================
# MAIN CONTROL LOOP
# ============================================================

def run(stop_requested=None, log=print):

    global servo_current_us
    global servo_last_active_ms
    global motor_current_v
    global motor_accel_step_v
    global current_speed

    # --------------------------------------------------------
    # RESET CONTROL STATE
    # --------------------------------------------------------

    motor_current_v = 0.0

    motor_accel_step_v = MOTOR_START_SLEW_STEP_V

    set_motor_voltages(0.0, 0.0)

    servo_current_us = SERVO_CENTER

    set_servo_us(SERVO_CENTER)

    servo_last_active_ms = ticks_ms()

    mode = read_mode()

    # Read actual physical speed switch at startup.
    speed = read_speed()

    initialise_speed_led()

    throttle_armed = False

    diagnostic_counter = 0

    log(
        "Chair control started. Speed: {}".format(
            speed
        )
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
            and stop_requested()
        ):

            stop_outputs()

            log("")
            log("Chair control stopped.")

            return

        # ----------------------------------------------------
        # READ INPUTS
        # ----------------------------------------------------

        x = x_adc.read_u16()
        y = y_adc.read_u16()

        mode = read_mode()

        # ----------------------------------------------------
        # PHYSICAL SPEED SWITCH
        # ----------------------------------------------------

        previous_speed = current_speed

        speed = read_speed()

        if speed != previous_speed:

            log(
                "Speed changed to: {}".format(
                    speed
                )
            )

        # ----------------------------------------------------
        # SPEED LED
        # ----------------------------------------------------

        update_speed_led()

        # ----------------------------------------------------
        # STEERING
        # ----------------------------------------------------

        servo_target = joystick_to_servo(x)

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

            if y <= Y_CENTER + Y_DEADZONE:

                throttle_armed = True

        # ----------------------------------------------------
        # THROTTLE
        # ----------------------------------------------------

        if throttle_armed:

            motor_target_v = joystick_to_motor_voltage(
                y,
                mode,
                speed
            )

        else:

            motor_target_v = 0.0

        front_v, rear_v = update_motor_voltage(
            motor_target_v,
            mode,
            speed
        )

        # ----------------------------------------------------
        # DIAGNOSTICS
        # ----------------------------------------------------

        diagnostic_counter += 1

        if diagnostic_counter >= DIAGNOSTIC_INTERVAL_LOOPS:

            diagnostic_counter = 0

            x_percent = get_x_percentage(x)
            y_percent = get_y_percentage(y)

            log(
                "X: {:+.1f}% | Y: {:+.1f}% | "
                "Front: {:.3f} V | Rear: {:.3f} V | "
                "Servo: {} | Mode: {} | "
                "Speed switch: {}".format(
                    x_percent,
                    y_percent,
                    front_v,
                    rear_v,
                    servo_us,
                    mode,
                    speed
                )
            )

        # ----------------------------------------------------
        # MAINTAIN 20 ms LOOP
        # ----------------------------------------------------

        elapsed = ticks_diff(
            ticks_ms(),
            loop_start
        )

        remaining = LOOP_PERIOD_MS - elapsed

        if remaining > 0:
            sleep_ms(remaining)


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
        "GP20 and GP21 select LOW / MEDIUM / HIGH."
    )

    print()
    print(
        "Backward throttle movement does not drive motors."
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