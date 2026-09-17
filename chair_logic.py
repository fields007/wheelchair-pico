from machine import ADC, Pin, PWM
from time import sleep_ms, ticks_ms, ticks_diff


# ============================================================
# GENERAL SETTINGS
# ============================================================

LOOP_PERIOD_MS = 20


# ============================================================
# STATUS LED
# ============================================================

led = Pin(25, Pin.OUT)
led.on()


# ============================================================
# JOYSTICK
#
# ADC1 / GP27:
#   low  = LEFT
#   high = RIGHT
#
# ADC2 / GP28:
#   low  = BACKWARD
#   high = FORWARD
# ============================================================

x_adc = ADC(27)
y_adc = ADC(28)


# ------------------------------------------------------------
# X calibration
#
# Measured:
#
# LEFT:    mean = 52
# CENTER:  mean = 35477
# RIGHT:   mean = 65393
# ------------------------------------------------------------

X_LEFT   = 52
X_CENTER = 35477
X_RIGHT  = 65393

X_DEADZONE = 750


# ------------------------------------------------------------
# Y calibration
#
# Measured:
#
# BACKWARD: mean = 6632
# CENTER:   mean = 44581
# FORWARD:  mean = 65384
# ------------------------------------------------------------

Y_BACKWARD = 6632
Y_CENTER   = 44581
Y_FORWARD  = 65384

Y_DEADZONE = 850


# ------------------------------------------------------------
# FULL-FORWARD SATURATION
# ------------------------------------------------------------

Y_FORWARD_FULL = Y_FORWARD


# ============================================================
# SPEED SELECTOR SWITCH
#
# GP7   GP4
#
#  1     1   -> SLOW
#  0     1   -> MEDIUM
#  1     0   -> HIGH
#  0     0   -> INVALID
#
# Both inputs use the Pico's internal pull-ups.
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


SPEED_MODES = (
    "SLOW",
    "MEDIUM",
    "HIGH",
)


# Start safely in SLOW until a valid switch state is read.

speed_mode = "SLOW"

last_printed_speed_mode = None


# ============================================================
# SPEED MODE UPDATE
# ============================================================

def read_speed_mode():

    global speed_mode
    global last_printed_speed_mode


    p4 = speed_gp4.value()
    p7 = speed_gp7.value()


    # --------------------------------------------------------
    # DECODE SWITCH
    # --------------------------------------------------------

    if p7 == 1 and p4 == 1:

        new_mode = "SLOW"


    elif p7 == 0 and p4 == 1:

        new_mode = "MEDIUM"


    elif p7 == 1 and p4 == 0:

        new_mode = "HIGH"


    else:

        new_mode = None


    # --------------------------------------------------------
    # VALID POSITION
    # --------------------------------------------------------

    if new_mode is not None:

        speed_mode = new_mode


        if speed_mode != last_printed_speed_mode:

            print(
                "SPEED SWITCH ->",
                speed_mode,
                "| GP7:",
                p7,
                "| GP4:",
                p4
            )


            last_printed_speed_mode = speed_mode


    # --------------------------------------------------------
    # INVALID POSITION
    #
    # Keep the last valid speed mode.
    # --------------------------------------------------------

    else:

        print(
            "WARNING: invalid speed switch state",
            "| GP7:",
            p7,
            "| GP4:",
            p4,
            "| keeping:",
            speed_mode
        )


    return (
        speed_mode,
        p7,
        p4
    )


# ============================================================
# MOTOR VOLTAGE SETTINGS
# ============================================================

LOW_MOTOR_VOLTAGE = 1.67


# ------------------------------------------------------------
# SECOND MOTOR PAIR
# ------------------------------------------------------------

ALL_MOTORS_ON_VOLTAGE  = 1.77
ALL_MOTORS_OFF_VOLTAGE = 1.75


# ------------------------------------------------------------
# MAXIMUM VOLTAGE FOR EACH SPEED MODE
# ============================================================

SPEED_VOLTAGES = {

    "SLOW": {
        "min": LOW_MOTOR_VOLTAGE,
        "max": 1.90,
    },

    "MEDIUM": {
        "min": LOW_MOTOR_VOLTAGE,
        "max": 2.20,
    },

    "HIGH": {
        "min": LOW_MOTOR_VOLTAGE,
        "max": 3.30,
    },
}


# ============================================================
# NONLINEAR THROTTLE
# ============================================================

THROTTLE_EXPONENT = 2.0


# ============================================================
# MOTOR ACCELERATION
# ============================================================

MOTOR_SLEW_STEP_V = 0.010

MOTOR_START_SLEW_STEP_V = 0.0005

MOTOR_SLEW_ACCELERATION_V = 0.00005


motor_accel_step_v = (
    MOTOR_START_SLEW_STEP_V
)

motor_current_v = 0.0


# ============================================================
# STEERING
# ============================================================

servo = PWM(
    Pin(10)
)

servo.freq(
    50
)


SERVO_LEFT   = 1100
SERVO_CENTER = 1500
SERVO_RIGHT  = 1900


# ------------------------------------------------------------
# SERVO RELEASE
# ------------------------------------------------------------

SERVO_RELEASE_DELAY_MS = 500


servo_last_active_ms = ticks_ms()

servo_enabled = True


# ------------------------------------------------------------
# SPEED-DEPENDENT STEERING RATE
# ------------------------------------------------------------

STEERING_STEP_US = {

    "SLOW": 8,

    "MEDIUM": 4,

    "HIGH": 2,
}


servo_current_us = SERVO_CENTER


# ============================================================
# SERVO OUTPUT
# ============================================================

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


# ============================================================
# DISABLE SERVO PWM
# ============================================================

def disable_servo():

    global servo_enabled


    servo.duty_u16(
        0
    )


    servo_enabled = False


# ============================================================
# JOYSTICK -> STEERING TARGET
# ============================================================

def joystick_to_servo(x):

    # --------------------------------------------------------
    # CENTER
    # --------------------------------------------------------

    if abs(
        x
        - X_CENTER
    ) <= X_DEADZONE:

        return SERVO_CENTER


    # --------------------------------------------------------
    # LEFT
    # --------------------------------------------------------

    if x < X_CENTER:

        x = max(
            x,
            X_LEFT
        )


        fraction = (
            (
                X_CENTER
                - X_DEADZONE
                - x
            )
            /
            (
                X_CENTER
                - X_DEADZONE
                - X_LEFT
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
            - fraction
            * (
                SERVO_CENTER
                - SERVO_LEFT
            )
        )


    # --------------------------------------------------------
    # RIGHT
    # --------------------------------------------------------

    else:

        x = min(
            x,
            X_RIGHT
        )


        fraction = (
            (
                x
                - (
                    X_CENTER
                    + X_DEADZONE
                )
            )
            /
            (
                X_RIGHT
                - (
                    X_CENTER
                    + X_DEADZONE
                )
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


# ============================================================
# STEERING RATE LIMIT + AUTOMATIC SERVO RELEASE
# ============================================================

def update_servo(
    target_us,
    speed_mode,
    joystick_x
):

    global servo_current_us
    global servo_last_active_ms
    global servo_enabled


    now = ticks_ms()


    # --------------------------------------------------------
    # CHECK WHETHER STEERING JOYSTICK IS ACTIVE
    # --------------------------------------------------------

    joystick_active = (
        abs(
            joystick_x
            - X_CENTER
        )
        > X_DEADZONE
    )


    # --------------------------------------------------------
    # JOYSTICK MOVED
    # --------------------------------------------------------

    if joystick_active:

        servo_last_active_ms = now

        servo_enabled = True


    # --------------------------------------------------------
    # MOVE SERVO COMMAND TOWARD TARGET
    # --------------------------------------------------------

    max_step = STEERING_STEP_US[
        speed_mode
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


    # --------------------------------------------------------
    # CHECK WHETHER WE ARE FULLY CENTERED
    # --------------------------------------------------------

    centered = (
        not joystick_active
        and servo_current_us == SERVO_CENTER
    )


    # --------------------------------------------------------
    # CENTERED -> RELEASE AFTER DELAY
    # --------------------------------------------------------

    if centered:

        if ticks_diff(
            now,
            servo_last_active_ms
        ) >= SERVO_RELEASE_DELAY_MS:

            disable_servo()


            return int(
                servo_current_us
            )


    # --------------------------------------------------------
    # OTHERWISE KEEP SERVO ACTIVE
    # --------------------------------------------------------

    set_servo_us(
        servo_current_us
    )


    return int(
        servo_current_us
    )


# ============================================================
# MOTOR PWM OUTPUTS
# ============================================================

motor_17 = PWM(
    Pin(17)
)

motor_18 = PWM(
    Pin(18)
)

motor_12 = PWM(
    Pin(12)
)

motor_15 = PWM(
    Pin(15)
)


MOTOR_PWMS = (
    motor_17,
    motor_18,
    motor_12,
    motor_15,
)


for pwm in MOTOR_PWMS:

    pwm.freq(
        20000
    )

    pwm.duty_u16(
        0
    )


# ============================================================
# PWM VOLTAGE OUTPUT
# ============================================================

PICO_PWM_VOLTAGE = 3.3


def set_pwm_voltage(
    pwm,
    voltage
):

    voltage = max(
        0.0,
        min(
            PICO_PWM_VOLTAGE,
            voltage
        )
    )


    if voltage <= 0.0:

        duty = 0


    elif voltage >= PICO_PWM_VOLTAGE:

        duty = 65535


    else:

        duty = int(
            round(
                voltage
                / PICO_PWM_VOLTAGE
                * 65535
            )
        )


    pwm.duty_u16(
        duty
    )


# ============================================================
# MOTOR PAIR SELECTION
# ============================================================

all_motors_enabled = False


def set_motor_voltage(voltage):

    global all_motors_enabled


    # --------------------------------------------------------
    # STOP
    # --------------------------------------------------------

    if voltage < LOW_MOTOR_VOLTAGE:

        all_motors_enabled = False


        for pwm in MOTOR_PWMS:

            set_pwm_voltage(
                pwm,
                0.0
            )


        return


    # --------------------------------------------------------
    # SECOND MOTOR PAIR HYSTERESIS
    # --------------------------------------------------------

    if all_motors_enabled:

        if voltage < ALL_MOTORS_OFF_VOLTAGE:

            all_motors_enabled = False


    else:

        if voltage > ALL_MOTORS_ON_VOLTAGE:

            all_motors_enabled = True


    # --------------------------------------------------------
    # GP17 + GP12
    # --------------------------------------------------------

    set_pwm_voltage(
        motor_17,
        voltage
    )

    set_pwm_voltage(
        motor_12,
        voltage
    )


    # --------------------------------------------------------
    # GP18 + GP15
    # --------------------------------------------------------

    if all_motors_enabled:

        set_pwm_voltage(
            motor_18,
            voltage
        )

        set_pwm_voltage(
            motor_15,
            voltage
        )


    else:

        set_pwm_voltage(
            motor_18,
            0.0
        )

        set_pwm_voltage(
            motor_15,
            0.0
        )


# ============================================================
# JOYSTICK -> THROTTLE FRACTION
# ============================================================

def get_throttle_fraction(y):

    start = (
        Y_CENTER
        + Y_DEADZONE
    )


    # --------------------------------------------------------
    # NEUTRAL / BACKWARDS
    # --------------------------------------------------------

    if y <= start:

        return 0.0


    # --------------------------------------------------------
    # FULL FORWARD
    # --------------------------------------------------------

    if y >= Y_FORWARD_FULL:

        return 1.0


    # --------------------------------------------------------
    # NORMALIZED RANGE
    # --------------------------------------------------------

    fraction = (
        y
        - start
    ) / (
        Y_FORWARD_FULL
        - start
    )


    return max(
        0.0,
        min(
            1.0,
            fraction
        )
    )


# ============================================================
# JOYSTICK -> TARGET MOTOR VOLTAGE
# ============================================================

def joystick_to_motor_voltage(
    y,
    speed_mode
):

    throttle_fraction = (
        get_throttle_fraction(
            y
        )
    )


    # --------------------------------------------------------
    # STOP
    # --------------------------------------------------------

    if throttle_fraction <= 0.0:

        return 0.0


    min_v = SPEED_VOLTAGES[
        speed_mode
    ]["min"]


    max_v = SPEED_VOLTAGES[
        speed_mode
    ]["max"]


    # --------------------------------------------------------
    # EXACT FULL THROTTLE
    # --------------------------------------------------------

    if throttle_fraction >= 1.0:

        return max_v


    # --------------------------------------------------------
    # NONLINEAR THROTTLE
    # --------------------------------------------------------

    throttle_shaped = (
        throttle_fraction
        ** THROTTLE_EXPONENT
    )


    voltage = (
        min_v
        + throttle_shaped
        * (
            max_v
            - min_v
        )
    )


    return voltage


# ============================================================
# SMOOTH MOTOR VOLTAGE UPDATE
# ============================================================

def update_motor_voltage(
    target_v,
    speed_mode
):

    global motor_current_v
    global motor_accel_step_v


    # --------------------------------------------------------
    # STOP
    # --------------------------------------------------------

    if target_v <= 0.0:

        motor_current_v = 0.0


        motor_accel_step_v = (
            MOTOR_START_SLEW_STEP_V
        )


        set_motor_voltage(
            0.0
        )


        return motor_current_v


    # --------------------------------------------------------
    # START FROM REST
    # --------------------------------------------------------

    if motor_current_v <= 0.0:

        motor_current_v = (
            LOW_MOTOR_VOLTAGE
        )


        motor_accel_step_v = (
            MOTOR_START_SLEW_STEP_V
        )


        set_motor_voltage(
            motor_current_v
        )


        return motor_current_v


    # --------------------------------------------------------
    # ACCELERATION
    # --------------------------------------------------------

    if motor_current_v < target_v:

        difference = (
            target_v
            - motor_current_v
        )


        change = min(
            motor_accel_step_v,
            difference
        )


        motor_current_v += change


        motor_accel_step_v += (
            MOTOR_SLEW_ACCELERATION_V
        )


        motor_accel_step_v = min(
            motor_accel_step_v,
            MOTOR_SLEW_STEP_V
        )


    # --------------------------------------------------------
    # DECELERATION
    # --------------------------------------------------------

    elif motor_current_v > target_v:

        difference = (
            motor_current_v
            - target_v
        )


        change = min(
            MOTOR_SLEW_STEP_V,
            difference
        )


        motor_current_v -= change


    # --------------------------------------------------------
    # HARD SAFETY CLAMP
    # --------------------------------------------------------

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
# STARTUP
# ============================================================

set_motor_voltage(
    0.0
)


set_servo_us(
    SERVO_CENTER
)


servo_last_active_ms = ticks_ms()


# Read initial physical switch position.

speed_mode, speed_p7, speed_p4 = read_speed_mode()


# ============================================================
# STARTUP THROTTLE SAFETY
# ============================================================

throttle_armed = False


print()
print("Throttle DISARMED.")
print("Return joystick to neutral to arm.")
print()


# ============================================================
# MAIN LOOP
# ============================================================

diagnostic_counter = 0


while True:

    loop_start = ticks_ms()


    # --------------------------------------------------------
    # READ JOYSTICK
    # --------------------------------------------------------

    x = x_adc.read_u16()
    y = y_adc.read_u16()


    # --------------------------------------------------------
    # SPEED SELECTOR
    # --------------------------------------------------------

    (
        speed_mode,
        speed_p7,
        speed_p4

    ) = read_speed_mode()


    # --------------------------------------------------------
    # STEERING
    # --------------------------------------------------------

    servo_target = (
        joystick_to_servo(
            x
        )
    )


    servo_us = (
        update_servo(
            servo_target,
            speed_mode,
            x
        )
    )


    # --------------------------------------------------------
    # THROTTLE ARMING
    # --------------------------------------------------------

    if not throttle_armed:

        if y <= (
            Y_CENTER
            + Y_DEADZONE
        ):

            throttle_armed = True


            print(
                "Throttle ARMED."
            )


    # --------------------------------------------------------
    # MOTOR TARGET
    # --------------------------------------------------------

    if throttle_armed:

        motor_target_v = (
            joystick_to_motor_voltage(
                y,
                speed_mode
            )
        )


    else:

        motor_target_v = 0.0


    # --------------------------------------------------------
    # MOTOR OUTPUT
    # --------------------------------------------------------

    motor_v = (
        update_motor_voltage(
            motor_target_v,
            speed_mode
        )
    )


    # --------------------------------------------------------
    # DIAGNOSTICS
    # --------------------------------------------------------

    diagnostic_counter += 1


    if diagnostic_counter >= 10:

        diagnostic_counter = 0


        throttle_fraction = (
            get_throttle_fraction(
                y
            )
        )


        full_forward = (
            y >= Y_FORWARD_FULL
        )


        if motor_v < LOW_MOTOR_VOLTAGE:

            active_motors = "NONE"


        elif all_motors_enabled:

            active_motors = "ALL"


        else:

            active_motors = "GP12 + GP17"


        print(
            "X:",
            x,

            "Servo:",
            servo_us,

            "ServoPWM:",
            servo_enabled,

            "| Y:",
            y,

            "Joy:",
            "{:.1f}%".format(
                throttle_fraction * 100
            ),

            "FULL:",
            full_forward,

            "| Speed:",
            speed_mode,

            "GP7:",
            speed_p7,

            "GP4:",
            speed_p4,

            "| Target:",
            "{:.3f} V".format(
                motor_target_v
            ),

            "Command:",
            "{:.3f} V".format(
                motor_v
            ),

            "| Slew:",
            "{:.4f}".format(
                motor_accel_step_v
            ),

            "| Motors:",
            active_motors,

            "| Armed:",
            throttle_armed
        )


    # --------------------------------------------------------
    # KEEP LOOP CLOSE TO 20 ms
    # --------------------------------------------------------

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