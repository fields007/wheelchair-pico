"""
Wheelchair Hall Sensor and Handbrake Servo Diagnostic
====================================================

GPIO assignments
----------------
GP10    Front Right Hall sensor (FR)
GP11    Back Right Hall sensor (BR)
GP12    Front Left Hall sensor (FL)
GP13    Back Left Hall sensor (BL)

GP20    OLED SDA
GP21    OLED SCL

GP22    Handbrake servo PWM
GP28    Joystick Y-axis ADC

Hall sensor counting
--------------------
Both rising and falling edges are counted.

16 transitions = 1 wheel revolution.

Revolution counts accumulate since run() starts.

GPIO interrupts detect Hall sensor transitions
independently of the main loop.

Handbrake servo
---------------
Joystick Y controls servo position.

Fully backward: 1000 us
Center:         1500 us
Fully forward:  2000 us

Servo PWM frequency: 50 Hz

Logging
-------
Reports cumulative wheel revolutions and
handbrake servo position every second.

Compatibility
-------------
run(stop_requested=..., log=...)
stop_outputs()

Compatible with existing main.py.

Traction motors and steering servo remain disabled.
"""

from machine import Pin, PWM, ADC, I2C
from machine import disable_irq, enable_irq
from time import ticks_ms, ticks_diff, sleep_ms


# ============================================================
# GENERAL SETTINGS
# ============================================================

LOOP_PERIOD_MS = 20

LOG_INTERVAL_MS = 1000

DISPLAY_INTERVAL_MS = 200

TRANSITIONS_PER_REVOLUTION = 16

DISPLAY_X_OFFSET = 8

OLED_ADDRESS = 0x3C

OLED_WIDTH = 128
OLED_HEIGHT = 64


# ============================================================
# JOYSTICK CALIBRATION
# ============================================================

Y_BACKWARD = 6632

Y_CENTER = 44581

Y_FORWARD = 65384

Y_DEADZONE = 1600

y_adc = ADC(28)


# ============================================================
# HANDBRAKE SERVO SETTINGS
# ============================================================

HANDBRAKE_SERVO_PIN = 22

HANDBRAKE_SERVO_FREQUENCY = 50

HANDBRAKE_SERVO_MIN_US = 850

HANDBRAKE_SERVO_CENTER_US = 1500

HANDBRAKE_SERVO_MAX_US = 2000

HANDBRAKE_SERVO_PERIOD_US = 20000

handbrake_servo = PWM(
    Pin(HANDBRAKE_SERVO_PIN)
)

handbrake_servo.freq(
    HANDBRAKE_SERVO_FREQUENCY
)

handbrake_servo.duty_u16(0)

handbrake_servo_current_us = (
    HANDBRAKE_SERVO_CENTER_US
)


# ============================================================
# GENERAL FUNCTIONS
# ============================================================

def clamp(value, minimum, maximum):

    return max(
        minimum,
        min(maximum, value)
    )


# ============================================================
# SAFETY: DISABLE TRACTION AND STEERING
# ============================================================

def disable_drive_outputs():

    # Disable traction motor PWM signals.
    for pin_number in (14, 15, 17, 18):

        Pin(
            pin_number,
            Pin.OUT,
            value=0
        )

    # Disable steering servo.
    Pin(
        9,
        Pin.OUT,
        value=0
    )

    # Release motor reverse signal.
    Pin(
        8,
        Pin.OPEN_DRAIN,
        value=1
    )


def stop_outputs():

    disable_drive_outputs()

    # Disable handbrake servo PWM.
    handbrake_servo.duty_u16(0)


# ============================================================
# HALL SENSOR INITIALIZATION
# ============================================================

hall_FR = Pin(
    10,
    Pin.IN,
    Pin.PULL_UP
)

hall_BR = Pin(
    11,
    Pin.IN,
    Pin.PULL_UP
)

hall_FL = Pin(
    12,
    Pin.IN,
    Pin.PULL_UP
)

hall_BL = Pin(
    13,
    Pin.IN,
    Pin.PULL_UP
)


# ============================================================
# TRANSITION COUNTERS
# ============================================================

transitions_FR = 0
transitions_BR = 0
transitions_FL = 0
transitions_BL = 0


# ============================================================
# HALL INTERRUPT HANDLERS
# ============================================================

def irq_FR(pin):

    global transitions_FR

    transitions_FR += 1


def irq_BR(pin):

    global transitions_BR

    transitions_BR += 1


def irq_FL(pin):

    global transitions_FL

    transitions_FL += 1


def irq_BL(pin):

    global transitions_BL

    transitions_BL += 1


# ============================================================
# ENABLE HALL INTERRUPTS
# ============================================================

def enable_hall_interrupts():

    trigger = (
        Pin.IRQ_RISING |
        Pin.IRQ_FALLING
    )

    hall_FR.irq(
        handler=irq_FR,
        trigger=trigger
    )

    hall_BR.irq(
        handler=irq_BR,
        trigger=trigger
    )

    hall_FL.irq(
        handler=irq_FL,
        trigger=trigger
    )

    hall_BL.irq(
        handler=irq_BL,
        trigger=trigger
    )


# ============================================================
# DISABLE HALL INTERRUPTS
# ============================================================

def disable_hall_interrupts():

    hall_FR.irq(handler=None)

    hall_BR.irq(handler=None)

    hall_FL.irq(handler=None)

    hall_BL.irq(handler=None)


# ============================================================
# RESET COUNTERS
# ============================================================

def reset_counters():

    global transitions_FR
    global transitions_BR
    global transitions_FL
    global transitions_BL

    state = disable_irq()

    transitions_FR = 0
    transitions_BR = 0
    transitions_FL = 0
    transitions_BL = 0

    enable_irq(state)


# ============================================================
# READ COUNTERS
# ============================================================

def get_transition_counts():

    state = disable_irq()

    fr = transitions_FR
    br = transitions_BR
    fl = transitions_FL
    bl = transitions_BL

    enable_irq(state)

    return {
        "FR": fr,
        "BR": br,
        "FL": fl,
        "BL": bl
    }


# ============================================================
# CALCULATE REVOLUTIONS
# ============================================================

def get_revolutions():

    counts = get_transition_counts()

    return {
        name: counts[name] / TRANSITIONS_PER_REVOLUTION
        for name in counts
    }


# ============================================================
# JOYSTICK Y PERCENTAGE
# ============================================================

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
            fraction,
            0.0,
            1.0
        )

    fraction = (
        y - Y_CENTER - Y_DEADZONE
    ) / (
        Y_FORWARD - Y_CENTER - Y_DEADZONE
    )

    return 100.0 * clamp(
        fraction,
        0.0,
        1.0
    )


# ============================================================
# JOYSTICK TO SERVO PULSE
# ============================================================

def joystick_to_servo_us(y_percent):

    y_percent = clamp(
        y_percent,
        -100.0,
        100.0
    )

    if y_percent < 0:

        pulse = (
            HANDBRAKE_SERVO_CENTER_US
            + y_percent / 100.0
            * (
                HANDBRAKE_SERVO_CENTER_US
                - HANDBRAKE_SERVO_MIN_US
            )
        )

    else:

        pulse = (
            HANDBRAKE_SERVO_CENTER_US
            + y_percent / 100.0
            * (
                HANDBRAKE_SERVO_MAX_US
                - HANDBRAKE_SERVO_CENTER_US
            )
        )

    return int(
        clamp(
            pulse,
            HANDBRAKE_SERVO_MIN_US,
            HANDBRAKE_SERVO_MAX_US
        )
    )


# ============================================================
# SET HANDBRAKE SERVO
# ============================================================

def set_handbrake_servo_us(pulse_us):

    global handbrake_servo_current_us

    pulse_us = clamp(
        pulse_us,
        HANDBRAKE_SERVO_MIN_US,
        HANDBRAKE_SERVO_MAX_US
    )

    duty = int(
        pulse_us
        * 65535
        / HANDBRAKE_SERVO_PERIOD_US
    )

    handbrake_servo.duty_u16(duty)

    handbrake_servo_current_us = int(
        pulse_us
    )


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

    if OLED_ADDRESS in i2c.scan():

        oled = ssd1306.SSD1306_I2C(
            OLED_WIDTH,
            OLED_HEIGHT,
            i2c,
            addr=OLED_ADDRESS
        )

except Exception as error:

    print(
        "OLED initialization failed:",
        error
    )


# ============================================================
# OLED TEXT
# ============================================================

def oled_write(text, row):

    if oled is None:
        return

    x = DISPLAY_X_OFFSET

    y = row * 10

    max_characters = (
        (OLED_WIDTH - x) // 8
    )

    oled.text(
        str(text)[:max_characters],
        x,
        y
    )


# ============================================================
# OLED DISPLAY
# ============================================================

def display_information(
    revolutions,
    y_percent,
    servo_us
):

    if oled is None:
        return

    try:

        oled.fill(0)

        oled_write(
            "REVOLUTIONS",
            0
        )

        oled_write(
            "FR:{:.2f} BR:{:.2f}".format(
                revolutions["FR"],
                revolutions["BR"]
            ),
            1
        )

        oled_write(
            "FL:{:.2f} BL:{:.2f}".format(
                revolutions["FL"],
                revolutions["BL"]
            ),
            2
        )

        oled_write(
            "16 edges/rev",
            3
        )

        oled_write(
            "Joystick:{:+.0f}%".format(
                y_percent
            ),
            4
        )

        oled_write(
            "Brake:{}us".format(
                servo_us
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
# FORMAT LOG MESSAGE
# ============================================================

def format_log(
    revolutions,
    y_percent,
    servo_us
):

    return (
        "HALL | "
        "FR:{:.3f} | "
        "FL:{:.3f} | "
        "BR:{:.3f} | "
        "BL:{:.3f} | "
        "Joystick Y:{:+.1f}% | "
        "Handbrake servo:{}us"
    ).format(
        revolutions["FR"],
        revolutions["FL"],
        revolutions["BR"],
        revolutions["BL"],
        y_percent,
        servo_us
    )


# ============================================================
# MAIN LOOP
# ============================================================

def run(stop_requested=None, log=print):

    # Disable all outputs initially.
    stop_outputs()

    # Reset revolution counters.
    disable_hall_interrupts()

    reset_counters()

    # Enable Hall sensor interrupts.
    enable_hall_interrupts()

    log("Hall and handbrake diagnostic started.")

    log("FR: GP10")
    log("BR: GP11")
    log("FL: GP12")
    log("BL: GP13")

    log("16 transitions = 1 revolution.")

    log("Handbrake servo: GP22")

    log("Joystick Y: GP28")

    log("Servo range: 1000-2000us")

    log("Traction motors disabled.")

    last_log_ms = ticks_ms()

    last_display_ms = ticks_ms()

    try:

        while True:

            loop_start = ticks_ms()

            # ============================================
            # SUPERVISOR STOP REQUEST
            # ============================================

            if (
                stop_requested is not None
                and stop_requested()
            ):

                log(
                    "Diagnostic stopped."
                )

                return

            # ============================================
            # READ JOYSTICK
            # ============================================

            y_raw = y_adc.read_u16()

            y_percent = get_y_percentage(
                y_raw
            )

            # ============================================
            # UPDATE HANDBRAKE SERVO
            # ============================================

            servo_us = joystick_to_servo_us(
                y_percent
            )

            set_handbrake_servo_us(
                servo_us
            )

            # ============================================
            # TIMING
            # ============================================

            now = ticks_ms()

            log_due = (
                ticks_diff(
                    now,
                    last_log_ms
                ) >= LOG_INTERVAL_MS
            )

            display_due = (
                ticks_diff(
                    now,
                    last_display_ms
                ) >= DISPLAY_INTERVAL_MS
            )

            # ============================================
            # READ REVOLUTIONS
            # ============================================

            if log_due or display_due:

                revolutions = get_revolutions()

            # ============================================
            # LOGGING
            # ============================================

            if log_due:

                last_log_ms = now

                message = format_log(
                    revolutions,
                    y_percent,
                    servo_us
                )

                print(message)

                if log is not print:

                    log(message)

            # ============================================
            # OLED
            # ============================================

            if display_due:

                last_display_ms = now

                display_information(
                    revolutions,
                    y_percent,
                    servo_us
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

        disable_hall_interrupts()

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

        print("Diagnostic stopped.")