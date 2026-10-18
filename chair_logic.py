from machine import Pin, PWM
from time import ticks_ms, ticks_diff, sleep_ms

# ============================================
# SETTINGS
# ============================================

BUTTON_PIN = 0
SERVO_PIN = 22

SERVO_FREQUENCY = 50

SERVO_ACTIVE_US = 900
SERVO_NEUTRAL_US = 1500

ACTIVE_DURATION_MS = 5000

# ============================================
# INITIALIZATION
# ============================================

button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)

servo = PWM(Pin(SERVO_PIN))
servo.freq(SERVO_FREQUENCY)


# ============================================
# SERVO CONTROL
# ============================================

def set_servo_us(pulse_us):
    duty = int(pulse_us * 65535 / 20000)
    servo.duty_u16(duty)


# ============================================
# MAIN LOOP
# ============================================

set_servo_us(SERVO_NEUTRAL_US)

print("Ready. Press handbrake button.")

active = False
waiting_for_release = False
activation_time = 0

try:
    while True:

        now = ticks_ms()
        pressed = button.value() == 0

        # Button pressed
        if pressed and not active and not waiting_for_release:

            print("Handbrake activated: 900 us")

            set_servo_us(SERVO_ACTIVE_US)

            activation_time = now
            active = True
            waiting_for_release = True

        # Return after 5 seconds
        if active and ticks_diff(now, activation_time) >= ACTIVE_DURATION_MS:

            print("Returning to 1500 us")

            set_servo_us(SERVO_NEUTRAL_US)

            active = False

        # Allow next activation after release
        if not pressed and not active:

            waiting_for_release = False

        sleep_ms(10)

except KeyboardInterrupt:

    print("Stopped.")

finally:

    set_servo_us(SERVO_NEUTRAL_US)