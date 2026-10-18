"""
Wheelchair Hall Sensor Revolution Counter
=========================================

GPIO assignments
----------------
GP10    Front Right Hall sensor (FR)
GP11    Back Right Hall sensor (BR)
GP12    Front Left Hall sensor (FL)
GP13    Back Left Hall sensor (BL)

GP20    OLED SDA
GP21    OLED SCL

Counting
--------
Both rising and falling edges are counted.

8 transitions = 1 revolution.

Revolution counts accumulate from program start.

The program uses GPIO interrupts to detect short
pulses without relying on the main loop frequency.

Display
-------
Shows cumulative revolutions for all four wheels.

Logging
-------
Prints and sends revolution counts every second
through the supplied log() function.

Compatibility
-------------
run(stop_requested=..., log=...)
stop_outputs()

Compatible with existing main.py.

All traction motors and servos remain disabled.
"""

from machine import Pin, I2C, disable_irq, enable_irq
from time import ticks_ms, ticks_diff, sleep_ms


# ============================================================
# SETTINGS
# ============================================================

LOOP_PERIOD_MS = 20

LOG_INTERVAL_MS = 1000

DISPLAY_INTERVAL_MS = 200

TRANSITIONS_PER_REVOLUTION = 8

DISPLAY_X_OFFSET = 8

OLED_ADDRESS = 0x3C

OLED_WIDTH = 128
OLED_HEIGHT = 64


# ============================================================
# SAFETY: DISABLE OUTPUTS
# ============================================================

def stop_outputs():

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

    # Disable handbrake servo.
    Pin(
        22,
        Pin.OUT,
        value=0
    )

    # Release reverse signal.
    Pin(
        8,
        Pin.OPEN_DRAIN,
        value=1
    )


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

# Each counter stores the total number of transitions
# detected since the diagnostic started.

transitions_FR = 0
transitions_BR = 0
transitions_FL = 0
transitions_BL = 0


# ============================================================
# INTERRUPT HANDLERS
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
# ENABLE INTERRUPTS
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
# DISABLE INTERRUPTS
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

    # Temporarily disable interrupts so that all
    # four counters are copied consistently.

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

def display_revolutions(revolutions):

    if oled is None:
        return

    try:

        oled.fill(0)

        oled_write(
            "WHEEL REVOLUTIONS",
            0
        )

        oled_write(
            "FR: {:.2f}".format(
                revolutions["FR"]
            ),
            1
        )

        oled_write(
            "FL: {:.2f}".format(
                revolutions["FL"]
            ),
            2
        )

        oled_write(
            "BR: {:.2f}".format(
                revolutions["BR"]
            ),
            3
        )

        oled_write(
            "BL: {:.2f}".format(
                revolutions["BL"]
            ),
            4
        )

        oled_write(
            "8 edges = 1 rev",
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

def format_revolution_log(revolutions):

    return (
        "HALL REVOLUTIONS | "
        "FR:{:.3f} | "
        "FL:{:.3f} | "
        "BR:{:.3f} | "
        "BL:{:.3f}"
    ).format(
        revolutions["FR"],
        revolutions["FL"],
        revolutions["BR"],
        revolutions["BL"]
    )


# ============================================================
# MAIN LOOP
# ============================================================

def run(stop_requested=None, log=print):

    # Disable motor and servo outputs.
    stop_outputs()

    # Start counting from zero.
    disable_hall_interrupts()

    reset_counters()

    # Enable edge detection.
    enable_hall_interrupts()

    log("Hall revolution counter started.")

    log("FR: GP10")
    log("BR: GP11")
    log("FL: GP12")
    log("BL: GP13")

    log("8 transitions = 1 revolution.")

    log("Counting rising and falling edges.")

    log("Motor outputs disabled.")

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
                    "Hall revolution counter stopped."
                )

                return

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

                message = format_revolution_log(
                    revolutions
                )

                # Print to serial console.
                print(message)

                # Send to main.py logging system.
                if log is not print:

                    log(message)

            # ============================================
            # OLED UPDATE
            # ============================================

            if display_due:

                last_display_ms = now

                display_revolutions(
                    revolutions
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

        print("Hall diagnostic stopped.")