"""
Wheelchair control logic.

GPIO assignments
----------------
GP0       Handbrake button (active LOW)
GP1       Brake reed switch (0 = BRAKES ON)
GP2       Speed selector A
GP4       Mode selector A
GP7       Mode selector B
GP8       Motor reverse (open-drain)
GP9       Steering servo
GP10-13   Wheel Hall sensors (reserved)
GP14      Front-right motor PWM
GP15      Front-left motor PWM
GP17      Back-right motor PWM
GP18      Back-left motor PWM
GP19      Speed selector B
GP20      OLED SDA
GP21      OLED SCL
GP22      Handbrake servo (not implemented)
GP26      Wi-Fi button (owned by main.py)
GP27      Steering joystick
GP28      Throttle joystick

OLED
----
Mode and speed setting
Speed: 0km/h (placeholder)
Throttle: signed percentage
Steering: signed percentage
Brakes: ON/OFF
WiFi: ONLINE/OFFLINE

Logging
-------
Joystick X/Y
Mode and speed setting
Speed: 0km/h (placeholder)
Steering servo pulse
Brakes
Handbrake button
Handbrake servo pulse (placeholder)
Wi-Fi connection status

The module provides:

    run(stop_requested=..., log=...)
    stop_outputs()

for compatibility with main.py.
"""

from machine import ADC, Pin, PWM, I2C
from time import sleep_ms, ticks_ms, ticks_diff

try:
    import network
except ImportError:
    network = None


# ============================================================
# GENERAL SETTINGS
# ============================================================

LOOP_PERIOD_MS = 20

DIAGNOSTIC_INTERVAL_MS = 1000

DISPLAY_INTERVAL_MS = 700

DISPLAY_X_OFFSET = 8

# Wheel-speed measurement is not implemented yet.
SPEED_KMH = 0.0

# Handbrake servo control is not implemented yet.
HANDBRAKE_SERVO_PLACEHOLDER_US = 1500


# ============================================================
# GENERAL FUNCTIONS
# ============================================================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def get_wifi_status():

    if network is None:
        return "OFFLINE"

    try:
        wlan = network.WLAN(network.STA_IF)

        if wlan.isconnected():
            return "ONLINE"

    except Exception:
        pass

    return "OFFLINE"


# ============================================================
# JOYSTICK CALIBRATION
# ============================================================

X_LEFT = 52
X_CENTER = 35000
X_RIGHT = 65393
X_DEADZONE = 1600

Y_BACKWARD = 6632
Y_CENTER = 44581
Y_FORWARD = 65384
Y_FORWARD_FULL = Y_FORWARD
Y_DEADZONE = 1600

x_adc = ADC(27)
y_adc = ADC(28)


def get_x_percentage(x):

    if abs(x - X_CENTER) <= X_DEADZONE:
        return 0.0

    if x < X_CENTER:

        fraction = (
            X_CENTER - X_DEADZONE - x
        ) / (
            X_CENTER - X_DEADZONE - X_LEFT
        )

        return -100.0 * clamp(
            fraction, 0.0, 1.0
        )

    fraction = (
        x - X_CENTER - X_DEADZONE
    ) / (
        X_RIGHT - X_CENTER - X_DEADZONE
    )

    return 100.0 * clamp(
        fraction, 0.0, 1.0
    )


def get_y_percentage(y):

    if abs(y - Y_CENTER) <= Y_DEADZONE:
        return 0.0

    if y < Y_CENTER:

        fraction = (
            Y_CENTER - Y_DEADZONE - y
        ) / (
            Y_CENTER - Y_DEADZONE - Y_BACKWARD
        )

        return -100.0 * clamp(
            fraction, 0.0, 1.0
        )

    fraction = (
        y - Y_CENTER - Y_DEADZONE
    ) / (
        Y_FORWARD - Y_CENTER - Y_DEADZONE
    )

    return 100.0 * clamp(
        fraction, 0.0, 1.0
    )


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

    if p4 == 1 and p7 == 1:
        current_mode = "TERRAIN"

    elif p4 == 1 and p7 == 0:
        current_mode = "SNOW"

    elif p4 == 0 and p7 == 1:
        current_mode = "ROAD"

    return current_mode


# ============================================================
# PHYSICAL SPEED SWITCH
# ============================================================

speed_gp2 = Pin(2, Pin.IN, Pin.PULL_UP)
speed_gp19 = Pin(19, Pin.IN, Pin.PULL_UP)

current_speed = "LOW"


def read_speed():

    global current_speed

    p2 = speed_gp2.value()
    p19 = speed_gp19.value()

    if p2 == 1 and p19 == 0:
        current_speed = "LOW"

    elif p2 == 1 and p19 == 1:
        current_speed = "MEDIUM"

    elif p2 == 0 and p19 == 1:
        current_speed = "HIGH"

    else:
        current_speed = "LOW"

    return current_speed


# ============================================================
# BRAKE INPUTS
# ============================================================

reed_switch = Pin(
    1,
    Pin.IN,
    Pin.PULL_UP
)

handbrake_button = Pin(
    0,
    Pin.IN,
    Pin.PULL_UP
)


def read_reed_switch():
    return reed_switch.value()


def read_handbrake_button():
    return handbrake_button.value()


def get_brake_status():
    return (
        "ON"
        if read_reed_switch() == 0
        else "OFF"
    )


def get_handbrake_status():
    return (
        "ON"
        if read_handbrake_button() == 0
        else "OFF"
    )


def get_handbrake_servo_us():

    # Placeholder only.
    #
    # GP22 is not initialized as PWM here.
    # No handbrake servo movement is commanded.

    return HANDBRAKE_SERVO_PLACEHOLDER_US


# ============================================================
# MOTOR DIRECTION
# ============================================================

# External reverse input must have a suitable pull-up
# to a GPIO-safe voltage.
#
# OPEN_DRAIN(1) releases the line.
# OPEN_DRAIN(0) pulls it to GND.
#
# Do not connect signals above 3.3 V directly to GP8.

reverse_pin = Pin(
    8,
    Pin.OPEN_DRAIN,
    value=1
)


def select_motor_direction(direction):

    reverse_pin.value(
        0 if direction == "REVERSE" else 1
    )


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

MOTOR_PWM_FREQUENCY = 20000

motor_BR = PWM(Pin(17))
motor_BL = PWM(Pin(18))
motor_FR = PWM(Pin(14))
motor_FL = PWM(Pin(15))

MOTOR_PWMS = (
    motor_BR,
    motor_BL,
    motor_FR,
    motor_FL
)

for pwm in MOTOR_PWMS:
    pwm.freq(MOTOR_PWM_FREQUENCY)
    pwm.duty_u16(0)


def disable_motors():

    for pwm in MOTOR_PWMS:
        pwm.duty_u16(0)


FRONT_MOTOR_PWMS = (
    motor_FR,
    motor_FL
)

REAR_MOTOR_PWMS = (
    motor_BR,
    motor_BL
)

PICO_PWM_VOLTAGE = 3.3


# WARNING:
# These outputs command real traction motor controllers.
# This module does not implement a hardware emergency stop.
# Do not use for occupied driving without independent
# safety validation.


def voltage_to_duty(voltage):

    voltage = clamp(
        voltage,
        0.0,
        PICO_PWM_VOLTAGE
    )

    return int(
        round(
            voltage / PICO_PWM_VOLTAGE * 65535
        )
    )


def set_motor_voltages(front_voltage, rear_voltage):

    # Check brakes again immediately before writing PWM.

    if read_reed_switch() == 0:

        disable_motors()
        return

    if front_voltage < MIN_MOTOR_VOLTAGE:
        front_voltage = 0.0

    if rear_voltage < MIN_MOTOR_VOLTAGE:
        rear_voltage = 0.0

    front_duty = voltage_to_duty(front_voltage)
    rear_duty = voltage_to_duty(rear_voltage)

    for pwm in FRONT_MOTOR_PWMS:
        pwm.duty_u16(front_duty)

    for pwm in REAR_MOTOR_PWMS:
        pwm.duty_u16(rear_duty)


def set_all_motor_voltage(voltage):

    set_motor_voltages(
        voltage,
        voltage
    )


# ============================================================
# THROTTLE MAPPING
# ============================================================

def get_throttle_fraction(y):

    if abs(y - Y_CENTER) <= Y_DEADZONE:
        return 0.0

    if y > Y_CENTER:

        start = Y_CENTER + Y_DEADZONE

        return clamp(
            (y - start) / (Y_FORWARD - start),
            0.0,
            1.0
        )

    start = Y_CENTER - Y_DEADZONE

    return clamp(
        (start - y) / (start - Y_BACKWARD),
        0.0,
        1.0
    )


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

    max_v = get_front_max_voltage(
        mode,
        speed
    )

    remaining_fraction = (
        (throttle - MIN_VOLTAGE_REGION)
        / (1.0 - MIN_VOLTAGE_REGION)
    )

    remaining_fraction = clamp(
        remaining_fraction,
        0.0,
        1.0
    )

    shaped = (
        remaining_fraction ** THROTTLE_EXPONENT
    )

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

    multiplier = (
        TERRAIN_REAR_MULTIPLIER[speed]
    )

    rear_voltage = (
        front_voltage * multiplier
    )

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

        motor_accel_step_v = (
            MOTOR_START_SLEW_STEP_V
        )

        set_motor_voltages(0.0, 0.0)

        return (0.0, 0.0)

    if motor_current_v <= 0.0:

        motor_current_v = MIN_MOTOR_VOLTAGE

        motor_accel_step_v = (
            MOTOR_START_SLEW_STEP_V
        )

    elif motor_current_v < target_v:

        difference = (
            target_v - motor_current_v
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
            motor_current_v - target_v
        )

        motor_current_v -= min(
            MOTOR_MAX_SLEW_STEP_V,
            difference
        )

    motor_current_v = clamp(
        motor_current_v,
        0.0,
        PICO_PWM_VOLTAGE
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

    return (
        front_voltage,
        rear_voltage
    )


# ============================================================
# STEERING SERVO
# ============================================================

servo = PWM(Pin(9))

servo.freq(50)

SERVO_LEFT = 1100
SERVO_CENTER = 1500
SERVO_RIGHT = 1900

SERVO_RELEASE_DELAY_MS = 500

STEERING_STEP_US = {
    "TERRAIN": {
        "LOW": 8,
        "MEDIUM": 8,
        "HIGH": 8
    },
    "SNOW": {
        "LOW": 8,
        "MEDIUM": 4,
        "HIGH": 4
    },
    "ROAD": {
        "LOW": 8,
        "MEDIUM": 4,
        "HIGH": 4
    }
}

servo_current_us = SERVO_CENTER

servo_last_active_ms = ticks_ms()

servo_enabled = False


def set_servo_us(pulse_us):

    global servo_enabled

    pulse_us = clamp(
        pulse_us,
        SERVO_LEFT,
        SERVO_RIGHT
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


def steering_to_pulse(steering_percent):

    steering_percent = clamp(
        steering_percent,
        -100.0,
        100.0
    )

    if steering_percent < 0:

        return int(
            SERVO_CENTER
            + steering_percent / 100.0
            * (SERVO_CENTER - SERVO_LEFT)
        )

    return int(
        SERVO_CENTER
        + steering_percent / 100.0
        * (SERVO_RIGHT - SERVO_CENTER)
    )


def calculate_steering(x_percent, direction):

    if direction == "REVERSE":
        return -x_percent

    return x_percent


def update_servo(
    target_us,
    mode,
    speed,
    joystick_x,
    elapsed_ms=LOOP_PERIOD_MS
):

    global servo_current_us
    global servo_last_active_ms

    now = ticks_ms()

    joystick_active = (
        abs(joystick_x - X_CENTER)
        > X_DEADZONE
    )

    if joystick_active:
        servo_last_active_ms = now

    max_step = (
        STEERING_STEP_US[mode][speed]
        * (
            clamp(elapsed_ms, 1, 60)
            / LOOP_PERIOD_MS
        )
    )

    difference = (
        target_us - servo_current_us
    )

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
# DIRECTION
# ============================================================

def get_direction(y_percent):

    if y_percent > 0:
        return "FORWARD"

    if y_percent < 0:
        return "REVERSE"

    return "NEUTRAL"


# ============================================================
# OLED INITIALIZATION
# ============================================================

oled = None

try:

    import ssd1306

    i2c = I2C(
        0,
        sda=Pin(20),
        scl=Pin(21),
        freq=100000
    )

    if 0x3C in i2c.scan():

        oled = ssd1306.SSD1306_I2C(
            128,
            64,
            i2c,
            addr=0x3C
        )

except Exception as error:

    print(
        "OLED initialization failed:",
        error
    )


# ============================================================
# OLED FORMATTING
# ============================================================

def oled_write(text, row):

    if oled is None:
        return

    x = DISPLAY_X_OFFSET

    y = row * 10

    # Standard SSD1306 text uses an 8x8 font.
    # Keep all text within the visible width.

    max_characters = (
        (128 - x) // 8
    )

    oled.text(
        str(text)[:max_characters],
        x,
        y
    )


def display_information(
    mode,
    speed,
    throttle,
    steering,
    reed_value,
    wifi_status
):

    if oled is None:
        return

    try:

        oled.fill(0)

        # Row 0: Driving mode and speed setting

        oled_write(
            "{} {}".format(
                mode,
                speed
            ),
            0
        )

        # Row 1: Actual speed placeholder

        oled_write(
            "Speed: {}km/h".format(
                int(SPEED_KMH)
            ),
            1
        )

        # Row 2: Signed throttle

        oled_write(
            "Throttle:{:+.0f}%".format(
                throttle
            ),
            2
        )

        # Row 3: Signed steering

        oled_write(
            "Steering:{:+.0f}%".format(
                steering
            ),
            3
        )

        # Row 4: Brake status

        oled_write(
            "Brakes: {}".format(
                "ON" if reed_value == 0 else "OFF"
            ),
            4
        )

        # Row 5: Wi-Fi status

        oled_write(
            "WiFi: {}".format(
                wifi_status
            ),
            5
        )

        oled.show()

    except Exception as error:

        print(
            "OLED update failed:",
            error
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

    disable_motors()

    select_motor_direction("FORWARD")

    disable_servo()


# ============================================================
# MAIN CONTROL LOOP
# ============================================================

def run(stop_requested=None, log=print):

    global servo_current_us
    global servo_last_active_ms
    global motor_current_v
    global motor_accel_step_v

    stop_outputs()

    servo_current_us = SERVO_CENTER

    set_servo_us(SERVO_CENTER)

    servo_last_active_ms = ticks_ms()

    last_diagnostic_ms = ticks_ms()
    last_display_ms = ticks_ms()

    previous_mode = None
    previous_speed = None
    previous_direction = None
    previous_reed = None
    previous_handbrake = None
    previous_wifi = None

    throttle_armed = False

    neutral_since_ms = None

    THROTTLE_ARM_NEUTRAL_MS = 500
    DIRECTION_NEUTRAL_MS = 800

    selected_motor_direction = "FORWARD"

    neutral_for_direction_since = None
    neutral_direction_ready = False

    last_servo_update_ms = ticks_ms()

    filtered_x_percent = 0.0

    log("Wheelchair control started.")

    log("Steering servo: GP9")

    log("Traction motor PWM enabled.")

    log("Handbrake servo not implemented.")

    try:

        while True:

            loop_start = ticks_ms()

            # ============================================
            # SUPERVISOR
            # ============================================

            if (
                stop_requested is not None
                and stop_requested()
            ):

                stop_outputs()

                log(
                    "Wheelchair control stopped."
                )

                return

            # ============================================
            # INPUTS
            # ============================================

            x = x_adc.read_u16()
            y = y_adc.read_u16()

            x_percent = get_x_percentage(x)
            y_percent = get_y_percentage(y)

            mode = read_mode()
            speed = read_speed()

            reed_value = read_reed_switch()

            handbrake_value = (
                read_handbrake_button()
            )

            direction = get_direction(
                y_percent
            )

            # ============================================
            # STEERING
            # ============================================

            filtered_x_percent += 0.20 * (
                x_percent - filtered_x_percent
            )

            if (
                abs(x_percent) < 0.01
                and abs(filtered_x_percent) < 0.25
            ):

                filtered_x_percent = 0.0

            steering_percent = calculate_steering(
                filtered_x_percent,
                direction
            )

            servo_target = steering_to_pulse(
                steering_percent
            )

            elapsed_servo_ms = max(
                1,
                ticks_diff(
                    loop_start,
                    last_servo_update_ms
                )
            )

            last_servo_update_ms = loop_start

            servo_us = update_servo(
                servo_target,
                mode,
                speed,
                x,
                elapsed_servo_ms
            )

            # ============================================
            # THROTTLE REQUEST
            # ============================================

            requested_throttle = (
                100.0 * get_throttle_fraction(y)
            )

            # Signed value for display and logs.
            signed_throttle = (
                requested_throttle
                if direction != "REVERSE"
                else -requested_throttle
            )

            joystick_neutral = (
                abs(y - Y_CENTER) <= Y_DEADZONE
            )

            brakes_on = (
                reed_value == 0
            )

            # ============================================
            # BRAKE INTERLOCK
            # ============================================

            if brakes_on:

                throttle_armed = False

                neutral_since_ms = None

                disable_motors()

            # ============================================
            # THROTTLE ARMING
            # ============================================

            if (
                not throttle_armed
                and not brakes_on
            ):

                if joystick_neutral:

                    if neutral_since_ms is None:

                        neutral_since_ms = loop_start

                    elif ticks_diff(
                        loop_start,
                        neutral_since_ms
                    ) >= THROTTLE_ARM_NEUTRAL_MS:

                        throttle_armed = True

                else:

                    neutral_since_ms = None

            # ============================================
            # DIRECTION NEUTRAL DWELL
            # ============================================

            if joystick_neutral:

                if neutral_for_direction_since is None:

                    neutral_for_direction_since = (
                        loop_start
                    )

                if ticks_diff(
                    loop_start,
                    neutral_for_direction_since
                ) >= DIRECTION_NEUTRAL_MS:

                    neutral_direction_ready = True

            else:

                neutral_for_direction_since = None

            # ============================================
            # DIRECTION SELECTION
            # ============================================

            if (
                direction != "NEUTRAL"
                and direction != selected_motor_direction
                and neutral_direction_ready
                and not brakes_on
            ):

                disable_motors()

                motor_current_v = 0.0

                motor_accel_step_v = (
                    MOTOR_START_SLEW_STEP_V
                )

                select_motor_direction(
                    direction
                )

                selected_motor_direction = (
                    direction
                )

                neutral_direction_ready = False

            # ============================================
            # MOTOR CONTROL
            # ============================================

            drive_allowed = (
                throttle_armed
                and not brakes_on
                and direction == selected_motor_direction
                and direction != "NEUTRAL"
            )

            motor_target_v = (
                joystick_to_motor_voltage(
                    y,
                    mode,
                    speed
                )
                if drive_allowed
                else 0.0
            )

            front_v, rear_v = update_motor_voltage(
                motor_target_v,
                mode,
                speed
            )

            # ============================================
            # EVENT LOGGING
            # ============================================

            if mode != previous_mode:

                log(
                    "Mode: {}".format(mode)
                )

                previous_mode = mode

            if speed != previous_speed:

                log(
                    "Speed setting: {}".format(speed)
                )

                previous_speed = speed

            if direction != previous_direction:

                log(
                    "Joystick direction: {}".format(
                        direction
                    )
                )

                previous_direction = direction

            if reed_value != previous_reed:

                log(
                    "Brakes: {}".format(
                        "ON" if reed_value == 0 else "OFF"
                    )
                )

                previous_reed = reed_value

            if handbrake_value != previous_handbrake:

                log(
                    "Handbrake: {}".format(
                        "ON" if handbrake_value == 0 else "OFF"
                    )
                )

                previous_handbrake = handbrake_value

            # ============================================
            # PERIODIC STATUS
            # ============================================

            now = ticks_ms()

            diagnostic_due = (
                ticks_diff(
                    now,
                    last_diagnostic_ms
                ) >= DIAGNOSTIC_INTERVAL_MS
            )

            display_due = (
                ticks_diff(
                    now,
                    last_display_ms
                ) >= DISPLAY_INTERVAL_MS
            )

            if diagnostic_due or display_due:

                wifi_status = get_wifi_status()

                if wifi_status != previous_wifi:

                    log(
                        "WiFi: {}".format(
                            wifi_status
                        )
                    )

                    previous_wifi = wifi_status

            # ============================================
            # DIAGNOSTIC LOG
            # ============================================

            if diagnostic_due:

                last_diagnostic_ms = now

                handbrake_status = (
                    "ON"
                    if handbrake_value == 0
                    else "OFF"
                )

                brake_status = (
                    "ON"
                    if reed_value == 0
                    else "OFF"
                )

                handbrake_servo_us = (
                    get_handbrake_servo_us()
                )

                log(
                    "X:{:+.1f}% Y:{:+.1f}% | "
                    "Mode:{} Speed:{} | "
                    "Speed:{:.0f}km/h | "
                    "Servo:{}us | "
                    "Brakes:{} | "
                    "Handbrake:{} | "
                    "Handbrake servo:{}us | "
                    "WiFi:{}".format(
                        x_percent,
                        y_percent,
                        mode,
                        speed,
                        SPEED_KMH,
                        servo_us,
                        brake_status,
                        handbrake_status,
                        handbrake_servo_us,
                        wifi_status
                    )
                )

            # ============================================
            # OLED
            # ============================================

            if display_due:

                last_display_ms = now

                display_information(
                    mode,
                    speed,
                    signed_throttle,
                    steering_percent,
                    reed_value,
                    wifi_status
                )

            # ============================================
            # LOOP TIMING
            # ============================================

            elapsed = ticks_diff(
                ticks_ms(),
                loop_start
            )

            remaining = (
                LOOP_PERIOD_MS - elapsed
            )

            if remaining > 0:

                sleep_ms(remaining)

    finally:

        stop_outputs()


# ============================================================
# DIRECT EXECUTION
# ============================================================

if __name__ == "__main__":

    try:

        run()

    except KeyboardInterrupt:

        print("Interrupted.")

    finally:

        stop_outputs()

        print("Chair outputs stopped.")