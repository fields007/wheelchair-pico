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
- speed selector: GP4 and GP7
- speed input: GP20 and GP21

Outputs:
- steering servo: GP10
- motor-controller PWM:
  GP17, GP18, GP14, GP15

The status LED and network/maintenance button are handled
by main.py.

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

    if y < Y_CENTER:

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
# SPEED SELECTOR
# ============================================================

speed_gp4 = Pin(
    4,
    Pin.IN,
    Pin.PULL_UP
)

speed_gp7 = Pin(
    7,
    Pin.IN,
    Pin.PULL_UP
)

speed_mode = "SLOW"


def read_speed_mode():

    global speed_mode

    p4 = speed_gp4.value()
    p7 = speed_gp7.value()

    if p7 == 1 and p4 == 1:

        speed_mode = "SLOW"

    elif p7 == 0 and p4 == 1:

        speed_mode = "MEDIUM"

    elif p7 == 1 and p4 == 0:

        speed_mode = "ROAD"

    # p4 == 0 and p7 == 0 is invalid.
    # In that case retain the previous valid mode.

    return speed_mode


# ============================================================
# SPEED INPUT
# ============================================================

# Three-position speed input:
#
# GP20   GP21   Speed
#  0      1       3
#  1      1       2
#  1      0       1
#
#  0      0     invalid
#
# If an invalid combination is detected, the previous valid
# speed is retained.

speed_gp20 = Pin(
    20,
    Pin.IN,
    Pin.PULL_UP
)

speed_gp21 = Pin(
    21,
    Pin.IN,
    Pin.PULL_UP
)

displayed_speed = 2


def read_displayed_speed():

    global displayed_speed

    p20 = speed_gp20.value()
    p21 = speed_gp21.value()

    if p20 == 0 and p21 == 1:

        displayed_speed = 3

    elif p20 == 1 and p21 == 1:

        displayed_speed = 2

    elif p20 == 1 and p21 == 0:

        displayed_speed = 1

    # p20 == 0 and p21 == 0 is invalid.
    # Retain the previous valid value.

    return displayed_speed


# ============================================================
# MOTOR SETTINGS
# ============================================================

MIN_MOTOR_VOLTAGE = 1.60

SPEED_MAX_VOLTAGE = {
    "SLOW": 1.90,
    "MEDIUM": 2.20,
    "ROAD": 3.30,
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


def set_motor_voltage(voltage):

    if voltage < MIN_MOTOR_VOLTAGE:
        voltage = 0.0

    duty = voltage_to_duty(
        voltage
    )

    for pwm in MOTOR_PWMS:

        pwm.duty_u16(
            duty
        )


# ============================================================
# THROTTLE MAPPING
# ============================================================

def get_throttle_fraction(y):

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


def joystick_to_motor_voltage(
    y,
    current_speed_mode
):

    throttle = get_throttle_fraction(
        y
    )

    if throttle <= 0.0:
        return 0.0

    if throttle <= MIN_VOLTAGE_REGION:
        return MIN_MOTOR_VOLTAGE

    max_v = SPEED_MAX_VOLTAGE[
        current_speed_mode
    ]

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
# MOTOR SLEW / SLOW START
# ============================================================

def update_motor_voltage(target_v):

    global motor_current_v
    global motor_accel_step_v

    if target_v <= 0.0:

        motor_current_v = 0.0

        motor_accel_step_v = (
            MOTOR_START_SLEW_STEP_V
        )

        set_motor_voltage(
            0.0
        )

        return motor_current_v

    if motor_current_v <= 0.0:

        motor_current_v = (
            MIN_MOTOR_VOLTAGE
        )

        motor_accel_step_v = (
            MOTOR_START_SLEW_STEP_V
        )

        set_motor_voltage(
            motor_current_v
        )

        return motor_current_v

    if motor_current_v < target_v:

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

    set_motor_voltage(
        motor_current_v
    )

    return motor_current_v


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
    "SLOW": 8,
    "MEDIUM": 4,
    "ROAD": 2,
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
    current_speed_mode,
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
        current_speed_mode
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
    """
    Immediately set motor PWM to zero and disable the steering
    servo.

    Safe for main.py to call even if the control loop is not
    currently running.
    """

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
    """
    Run wheelchair control.

    stop_requested:
        Optional callback called once per control loop.

        If it returns True, all outputs are stopped and run()
        returns to the caller.

    log:
        Function accepting exactly one string argument.

        Normal print() can be used, or main.py can provide its
        own logging function.

    The throttle must return to neutral after run() starts
    before motor drive is armed.
    """

    global servo_current_us
    global servo_last_active_ms
    global motor_current_v
    global motor_accel_step_v

    # --------------------------------------------------------
    # RESET CONTROL STATE
    # --------------------------------------------------------

    motor_current_v = 0.0

    motor_accel_step_v = (
        MOTOR_START_SLEW_STEP_V
    )

    set_motor_voltage(
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

    current_speed_mode = (
        read_speed_mode()
    )

    throttle_armed = False

    diagnostic_counter = 0

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

        current_displayed_speed = (
            read_displayed_speed()
        )

        current_speed_mode = (
            read_speed_mode()
        )

        # ----------------------------------------------------
        # STEERING
        # ----------------------------------------------------

        servo_target = (
            joystick_to_servo(x)
        )

        servo_us = update_servo(
            servo_target,
            current_speed_mode,
            x
        )

        # ----------------------------------------------------
        # THROTTLE ARMING
        # ----------------------------------------------------

        if not throttle_armed:

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
                    current_speed_mode
                )
            )

        else:

            motor_target_v = 0.0

        motor_v = update_motor_voltage(
            motor_target_v
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

            log(
                "X: {:+.1f}% | Y: {:+.1f}% | "
                "Voltage: {:.3f} V | Servo: {} | "
                "Mode: {} | Speed: {}".format(
                    x_percent,
                    y_percent,
                    motor_v,
                    servo_us,
                    current_speed_mode,
                    current_displayed_speed
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
    """
    Run the real wheelchair control system as an interactive
    hardware self-test.

    This test DOES drive the real outputs.

    It tests:
    - steering joystick
    - throttle joystick
    - speed selector
    - all four motor PWM outputs
    - steering servo
    - GP20 / GP21 speed input
    - normal diagnostics

    Throttle neutral arming remains active.

    Press Ctrl-C to stop the test. All outputs are then stopped.
    """

    print()
    print("==============================")
    print("CHAIR LOGIC SELF-TEST")
    print("==============================")
    print()

    print(
        "WARNING: REAL CHAIR OUTPUTS ARE ENABLED."
    )

    print(
        "The joystick will control the motors and steering."
    )

    print(
    "GP20 and GP21 select displayed Speed 1 / 2 / 3."
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

    # Ensure we begin with outputs stopped.
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

        # Always attempt to stop outputs when leaving the test.
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