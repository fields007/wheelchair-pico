"""Wheelchair control logic for Raspberry Pi Pico 2 W (MicroPython).

GPIO: GP0 handbrake button, GP1 brake reed, GP2/19 speed selector,
GP4/7 mode selector, GP8 reverse (open drain), GP9 steering servo,
GP10 FR / GP11 BR / GP12 FL / GP13 BL Hall sensors,
GP14 FR / GP15 FL / GP17 BR / GP18 BL motor PWM,
GP20 SDA / GP21 SCL OLED + IMU, GP22 handbrake servo,
GP26 Wi-Fi button (owned by main.py), GP27 X / GP28 Y joysticks.

Modes: TERRAIN, DIFFERENTIAL (former SNOW; currently same motor/steering
behaviour as TERRAIN), ROAD.

IMPORTANT: This is experimental control software, NOT a certified mobility
controller. Test with wheels lifted and no occupant. The GP1 brake reed is
an interlock but does not replace an independent hardware emergency stop.
PWM voltage below is an *average* of a 3.3 V PWM signal, not a DAC output.
An MPU-6500-compatible device at 0x68 is expected; orientation must be
calibrated to the physical installation before pitch/roll can be trusted.
Yaw below is relative, gyro-integrated and drifts (no compass reference).

Changes in this revision: throttle deadzone 2200, onset 1.63 V, gentler
voltage slew (0.00025 V initial step, +0.000025 V/loop, 0.006 V max
step), and steering no longer reverses with travel direction.
Rear multiplier is retained, but rear output is now ramped independently
so TERRAIN HIGH does not jump directly from zero to 2.445 V.
IMU reference is rotated so the supplied stationary reading is level.
"""

from machine import ADC, Pin, PWM, I2C, disable_irq, enable_irq
from time import sleep_ms, ticks_ms, ticks_diff
from math import atan2, sqrt, pi

try:
    import network
except ImportError:
    network = None

# ============================================================
# SETTINGS / HELPERS
# ============================================================
LOOP_PERIOD_MS = 20
DIAGNOSTIC_INTERVAL_MS = 1000
DISPLAY_INTERVAL_MS = 700
DISPLAY_X_OFFSET = 8
PICO_PWM_VOLTAGE = 3.3

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def get_wifi_status():
    if network is None:
        return 'OFFLINE'
    try:
        return 'ONLINE' if network.WLAN(network.STA_IF).isconnected() else 'OFFLINE'
    except Exception:
        return 'OFFLINE'

# ============================================================
# JOYSTICK CALIBRATION (UNCHANGED)
# ============================================================
X_LEFT, X_CENTER, X_RIGHT, X_DEADZONE = 52, 35000, 65393, 1600
Y_BACKWARD, Y_CENTER, Y_FORWARD, Y_DEADZONE = 6632, 44581, 65384, 2200
Y_FORWARD_FULL = Y_FORWARD
x_adc, y_adc = ADC(27), ADC(28)

def get_x_percentage(x):
    if abs(x - X_CENTER) <= X_DEADZONE:
        return 0.0
    if x < X_CENTER:
        return -100.0 * clamp((X_CENTER - X_DEADZONE - x) /
                              (X_CENTER - X_DEADZONE - X_LEFT), 0, 1)
    return 100.0 * clamp((x - X_CENTER - X_DEADZONE) /
                         (X_RIGHT - X_CENTER - X_DEADZONE), 0, 1)

def get_y_percentage(y):
    if abs(y - Y_CENTER) <= Y_DEADZONE:
        return 0.0
    if y < Y_CENTER:
        return -100.0 * clamp((Y_CENTER - Y_DEADZONE - y) /
                              (Y_CENTER - Y_DEADZONE - Y_BACKWARD), 0, 1)
    return 100.0 * clamp((y - Y_CENTER - Y_DEADZONE) /
                         (Y_FORWARD - Y_CENTER - Y_DEADZONE), 0, 1)

# ============================================================
# MODE / SPEED SWITCHES
# ============================================================
mode_gp4 = Pin(4, Pin.IN, Pin.PULL_UP)
mode_gp7 = Pin(7, Pin.IN, Pin.PULL_UP)
speed_gp2 = Pin(2, Pin.IN, Pin.PULL_UP)
speed_gp19 = Pin(19, Pin.IN, Pin.PULL_UP)
current_mode, current_speed = 'TERRAIN', 'LOW'

def read_mode():
    global current_mode
    pins = (mode_gp4.value(), mode_gp7.value())
    if pins == (1, 1):
        current_mode = 'TERRAIN'
    elif pins == (1, 0):
        current_mode = 'DIFFERENTIAL'
    elif pins == (0, 1):
        current_mode = 'ROAD'
    # Invalid state retains previous setting (original behaviour).
    return current_mode

def read_speed():
    global current_speed
    pins = (speed_gp2.value(), speed_gp19.value())
    if pins == (1, 0):
        current_speed = 'LOW'
    elif pins == (1, 1):
        current_speed = 'MEDIUM'
    elif pins == (0, 1):
        current_speed = 'HIGH'
    else:
        current_speed = 'LOW'
    return current_speed

# ============================================================
# BRAKES AND HANDBRAKE SERVO
# ============================================================
reed_switch = Pin(1, Pin.IN, Pin.PULL_UP)
handbrake_button = Pin(0, Pin.IN, Pin.PULL_UP)
HANDBRAKE_REST_US = 1500
HANDBRAKE_ACTIVE_US = 900
HANDBRAKE_HOLD_MS = 5000
HANDBRAKE_DEBOUNCE_MS = 40
handbrake_pwm = PWM(Pin(22))
handbrake_pwm.freq(50)
handbrake_servo_us = HANDBRAKE_REST_US
handbrake_active_since = None
handbrake_press_latched = False
handbrake_last_raw = handbrake_button.value()
handbrake_raw_since = ticks_ms()
handbrake_stable = handbrake_last_raw

def read_reed_switch():
    return reed_switch.value()

def read_handbrake_button():
    return handbrake_button.value()

def get_brake_status():
    return 'ON' if read_reed_switch() == 0 else 'OFF'

def get_handbrake_status():
    # Historical meaning: button status, not verified mechanical engagement.
    return 'ON' if read_handbrake_button() == 0 else 'OFF'

def set_handbrake_servo_us(pulse_us):
    global handbrake_servo_us
    handbrake_servo_us = int(clamp(pulse_us, 900, 2100))
    handbrake_pwm.duty_u16(handbrake_servo_us * 65535 // 20000)

def get_handbrake_servo_us():
    return handbrake_servo_us

def reset_handbrake():
    global handbrake_active_since, handbrake_press_latched
    global handbrake_last_raw, handbrake_raw_since, handbrake_stable
    handbrake_active_since = None
    handbrake_last_raw = read_handbrake_button()
    handbrake_stable = handbrake_last_raw
    handbrake_raw_since = ticks_ms()
    # Require a fresh release after startup or supervisor interruption.
    handbrake_press_latched = (handbrake_stable == 0)
    set_handbrake_servo_us(HANDBRAKE_REST_US)

def update_handbrake(now, log=print):
    global handbrake_active_since, handbrake_press_latched
    global handbrake_last_raw, handbrake_raw_since, handbrake_stable
    raw = read_handbrake_button()
    if raw != handbrake_last_raw:
        handbrake_last_raw = raw
        handbrake_raw_since = now
    if ticks_diff(now, handbrake_raw_since) >= HANDBRAKE_DEBOUNCE_MS:
        if raw != handbrake_stable:
            handbrake_stable = raw
            if raw == 1:
                handbrake_press_latched = False
            elif not handbrake_press_latched and handbrake_active_since is None:
                handbrake_press_latched = True
                handbrake_active_since = now
                set_handbrake_servo_us(HANDBRAKE_ACTIVE_US)
                log('Handbrake servo: {}us (activated)'.format(handbrake_servo_us))
    if handbrake_active_since is not None and ticks_diff(now, handbrake_active_since) >= HANDBRAKE_HOLD_MS:
        handbrake_active_since = None
        set_handbrake_servo_us(HANDBRAKE_REST_US)
        log('Handbrake servo: {}us (returned)'.format(handbrake_servo_us))
    return handbrake_active_since is not None

# ============================================================
# MOTOR DIRECTION / PWM
# ============================================================
# GP8 OPEN_DRAIN(1) releases line; controller must have GPIO-safe pullup.
reverse_pin = Pin(8, Pin.OPEN_DRAIN, value=1)

def select_motor_direction(direction):
    reverse_pin.value(0 if direction == 'REVERSE' else 1)

MIN_MOTOR_VOLTAGE = 1.63
ROAD_MAX_VOLTAGE = {'LOW': 1.90, 'MEDIUM': 2.20, 'HIGH': 3.30}
# Retained legacy table name for compatibility.
SNOW_MAX_VOLTAGE = {'LOW': 1.90, 'MEDIUM': 2.20, 'HIGH': 3.30}
TERRAIN_FRONT_MAX_VOLTAGE = 1.80
TERRAIN_REAR_MULTIPLIER = {'LOW': 1.00, 'MEDIUM': 1.2, 'HIGH': 1.5}
THROTTLE_EXPONENT = 2.0
MIN_VOLTAGE_REGION = 0.05
MOTOR_MAX_SLEW_STEP_V = 0.006
MOTOR_START_SLEW_STEP_V = 0.00025
MOTOR_SLEW_ACCELERATION_V = 0.000025
motor_current_v = 0.0
rear_current_v = 0.0
motor_accel_step_v = MOTOR_START_SLEW_STEP_V
MOTOR_PWM_FREQUENCY = 20000
motor_BR, motor_BL = PWM(Pin(17)), PWM(Pin(18))
motor_FR, motor_FL = PWM(Pin(14)), PWM(Pin(15))
MOTOR_PWMS = (motor_BR, motor_BL, motor_FR, motor_FL)
FRONT_MOTOR_PWMS = (motor_FR, motor_FL)
REAR_MOTOR_PWMS = (motor_BR, motor_BL)
for pwm in MOTOR_PWMS:
    pwm.freq(MOTOR_PWM_FREQUENCY)
    pwm.duty_u16(0)

def disable_motors():
    for pwm in MOTOR_PWMS:
        pwm.duty_u16(0)

def voltage_to_duty(voltage):
    return int(round(clamp(voltage, 0.0, PICO_PWM_VOLTAGE) / PICO_PWM_VOLTAGE * 65535))

def set_motor_voltages(front_voltage, rear_voltage):
    if read_reed_switch() == 0:
        disable_motors()
        return
    if front_voltage < MIN_MOTOR_VOLTAGE:
        front_voltage = 0.0
    if rear_voltage < MIN_MOTOR_VOLTAGE:
        rear_voltage = 0.0
    front_duty, rear_duty = voltage_to_duty(front_voltage), voltage_to_duty(rear_voltage)
    for pwm in FRONT_MOTOR_PWMS:
        pwm.duty_u16(front_duty)
    for pwm in REAR_MOTOR_PWMS:
        pwm.duty_u16(rear_duty)

def set_all_motor_voltage(voltage):
    set_motor_voltages(voltage, voltage)

def get_throttle_fraction(y):
    if abs(y - Y_CENTER) <= Y_DEADZONE:
        return 0.0
    if y > Y_CENTER:
        start = Y_CENTER + Y_DEADZONE
        return clamp((y - start) / (Y_FORWARD - start), 0, 1)
    start = Y_CENTER - Y_DEADZONE
    return clamp((start - y) / (start - Y_BACKWARD), 0, 1)

def get_front_max_voltage(mode, speed):
    if mode in ('TERRAIN', 'DIFFERENTIAL'):
        return TERRAIN_FRONT_MAX_VOLTAGE
    return ROAD_MAX_VOLTAGE[speed]

def joystick_to_motor_voltage(y, mode, speed):
    throttle = get_throttle_fraction(y)
    if throttle <= 0.0:
        return 0.0
    if throttle <= MIN_VOLTAGE_REGION:
        return MIN_MOTOR_VOLTAGE
    max_v = get_front_max_voltage(mode, speed)
    fraction = clamp((throttle - MIN_VOLTAGE_REGION) / (1.0 - MIN_VOLTAGE_REGION), 0, 1)
    return MIN_MOTOR_VOLTAGE + fraction ** THROTTLE_EXPONENT * (max_v - MIN_MOTOR_VOLTAGE)

def get_rear_voltage(front_voltage, mode, speed):
    if front_voltage <= 0:
        return 0.0
    if mode not in ('TERRAIN', 'DIFFERENTIAL'):
        return front_voltage
    return min(PICO_PWM_VOLTAGE, front_voltage * TERRAIN_REAR_MULTIPLIER[speed])

def update_motor_voltage(target_v, mode, speed):
    global motor_current_v, rear_current_v, motor_accel_step_v
    if target_v <= 0.0:
        motor_current_v = 0.0
        rear_current_v = 0.0
        motor_accel_step_v = MOTOR_START_SLEW_STEP_V
        set_motor_voltages(0.0, 0.0)
        return (0.0, 0.0)
    if motor_current_v <= 0.0:
        motor_current_v = MIN_MOTOR_VOLTAGE
        motor_accel_step_v = MOTOR_START_SLEW_STEP_V
    elif motor_current_v < target_v:
        motor_current_v += min(motor_accel_step_v, target_v - motor_current_v)
        motor_accel_step_v = min(motor_accel_step_v + MOTOR_SLEW_ACCELERATION_V, MOTOR_MAX_SLEW_STEP_V)
    elif motor_current_v > target_v:
        motor_current_v -= min(MOTOR_MAX_SLEW_STEP_V, motor_current_v - target_v)
    motor_current_v = clamp(motor_current_v, 0.0, PICO_PWM_VOLTAGE)
    front_voltage = motor_current_v
    rear_target_v = get_rear_voltage(front_voltage, mode, speed)
    # The previous code jumped straight to 1.63*1.5 = 2.445 V at
    # TERRAIN HIGH startup. Start rear at 1.63 V and slew independently.
    if rear_current_v <= 0.0:
        rear_current_v = MIN_MOTOR_VOLTAGE
    elif rear_current_v < rear_target_v:
        rear_current_v += min(MOTOR_MAX_SLEW_STEP_V,
                              rear_target_v - rear_current_v)
    else:
        rear_current_v -= min(MOTOR_MAX_SLEW_STEP_V,
                              rear_current_v - rear_target_v)
    rear_voltage = clamp(rear_current_v, 0.0, PICO_PWM_VOLTAGE)
    set_motor_voltages(front_voltage, rear_voltage)
    return (front_voltage, rear_voltage)

# ============================================================
# STEERING SERVO
# ============================================================
servo = PWM(Pin(9))
servo.freq(50)
SERVO_LEFT, SERVO_CENTER, SERVO_RIGHT = 1100, 1500, 1900
SERVO_RELEASE_DELAY_MS = 500
STEERING_STEP_US = {
    'TERRAIN': {'LOW': 8, 'MEDIUM': 8, 'HIGH': 8},
    'DIFFERENTIAL': {'LOW': 8, 'MEDIUM': 8, 'HIGH': 8},
    'ROAD': {'LOW': 8, 'MEDIUM': 4, 'HIGH': 4},
}
# Use a small target filter, then a fixed rate limit (no second joystick
# percentage filter). This avoids double-filter lag / occasional jolts.
STEERING_TARGET_FILTER = 0.25
servo_current_us = float(SERVO_CENTER)
servo_target_filtered_us = float(SERVO_CENTER)
servo_last_active_ms = ticks_ms()
servo_enabled = False

def set_servo_us(pulse_us):
    global servo_enabled
    pulse_us = clamp(pulse_us, SERVO_LEFT, SERVO_RIGHT)
    servo.duty_u16(int(pulse_us * 65535 / 20000))
    servo_enabled = True

def disable_servo():
    global servo_enabled
    servo.duty_u16(0)
    servo_enabled = False

def steering_to_pulse(steering_percent):
    p = clamp(steering_percent, -100.0, 100.0)
    if p < 0:
        return int(SERVO_CENTER + p / 100.0 * (SERVO_CENTER - SERVO_LEFT))
    return int(SERVO_CENTER + p / 100.0 * (SERVO_RIGHT - SERVO_CENTER))

def calculate_steering(x_percent, direction):
    # Steering direction remains the same when reversing.
    return x_percent

def update_servo(target_us, mode, speed, joystick_x, elapsed_ms=LOOP_PERIOD_MS):
    global servo_current_us, servo_target_filtered_us, servo_last_active_ms
    now = ticks_ms()
    joystick_active = abs(joystick_x - X_CENTER) > X_DEADZONE
    if joystick_active:
        servo_last_active_ms = now
    # Filter target, but snap to center when joystick is released to avoid
    # very long asymptotic tails that prevent automatic PWM release.
    if not joystick_active:
        servo_target_filtered_us = float(SERVO_CENTER)
    else:
        servo_target_filtered_us += STEERING_TARGET_FILTER * (target_us - servo_target_filtered_us)
    max_step = STEERING_STEP_US[mode][speed] * (clamp(elapsed_ms, 1, 60) / LOOP_PERIOD_MS)
    difference = servo_target_filtered_us - servo_current_us
    servo_current_us += clamp(difference, -max_step, max_step)
    if abs(servo_current_us - SERVO_CENTER) < 0.5 and not joystick_active:
        servo_current_us = float(SERVO_CENTER)
    if (not joystick_active and servo_current_us == SERVO_CENTER and
            ticks_diff(now, servo_last_active_ms) >= SERVO_RELEASE_DELAY_MS):
        disable_servo()
    else:
        set_servo_us(servo_current_us)
    return int(round(servo_current_us))

def get_direction(y_percent):
    if y_percent > 0:
        return 'FORWARD'
    if y_percent < 0:
        return 'REVERSE'
    return 'NEUTRAL'

# ============================================================
# WHEEL HALL COUNTERS: 8 MAGNETS, 16 TRANSITIONS PER REVOLUTION
# ============================================================
WHEEL_RADIUS_M = 0.225
HALL_TRANSITIONS_PER_REV = 16
HALL_MIN_EDGE_US = 1500  # reject extremely fast electrical glitches
HALL_STALE_MS = 1500
HALL_SAMPLE_MS = 200
# ISR only updates fixed integer counters; no allocation/logging in IRQ.
from time import ticks_us
hall_pins = (Pin(10, Pin.IN, Pin.PULL_UP), Pin(11, Pin.IN, Pin.PULL_UP),
             Pin(12, Pin.IN, Pin.PULL_UP), Pin(13, Pin.IN, Pin.PULL_UP))
hall_counts = [0, 0, 0, 0]
hall_last_edge_us = [0, 0, 0, 0]
hall_last_edge_ms = [0, 0, 0, 0]
hall_rpm = [0.0, 0.0, 0.0, 0.0]
hall_prev_counts = [0, 0, 0, 0]
hall_last_sample_ms = ticks_ms()
SPEED_KMH = 0.0

def make_hall_irq(index):
    def handler(pin):
        now = ticks_us()
        if ticks_diff(now, hall_last_edge_us[index]) >= HALL_MIN_EDGE_US:
            hall_counts[index] += 1
            hall_last_edge_us[index] = now
            hall_last_edge_ms[index] = ticks_ms()
    return handler

_hall_handlers = tuple(make_hall_irq(i) for i in range(4))
for i in range(4):
    hall_pins[i].irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_hall_handlers[i])

def update_wheel_speed(now):
    global hall_last_sample_ms, SPEED_KMH
    dt_ms = ticks_diff(now, hall_last_sample_ms)
    if dt_ms < HALL_SAMPLE_MS:
        return
    state = disable_irq()
    counts = tuple(hall_counts)
    last_edges = tuple(hall_last_edge_ms)
    enable_irq(state)
    for i in range(4):
        delta = counts[i] - hall_prev_counts[i]
        rpm = delta * 60000.0 / (HALL_TRANSITIONS_PER_REV * dt_ms)
        if ticks_diff(now, last_edges[i]) >= HALL_STALE_MS:
            rpm = 0.0
        hall_rpm[i] = rpm
        hall_prev_counts[i] = counts[i]
    hall_last_sample_ms = now
    # Hall sensors do not report rotation direction. Use mean magnitude.
    SPEED_KMH = sum(hall_rpm) / 4.0 * (2.0 * pi * WHEEL_RADIUS_M) * 60.0 / 1000.0

# ============================================================
# SHARED I2C: SSD1306 0x3C AND MPU-6500-COMPATIBLE 0x68
# ============================================================
i2c = None
oled = None
imu_available = False
imu_address = 0x68
imu_whoami = None
try:
    i2c = I2C(0, sda=Pin(20), scl=Pin(21), freq=100000)
    devices = i2c.scan()
    if 0x3C in devices:
        import ssd1306
        oled = ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3C)
    if imu_address in devices:
        imu_whoami = i2c.readfrom_mem(imu_address, 0x75, 1)[0]
        if imu_whoami in (0x70, 0x68, 0x71, 0x73):
            i2c.writeto_mem(imu_address, 0x6B, b'\x01')  # wake, PLL X
            i2c.writeto_mem(imu_address, 0x1B, b'\x00')  # gyro +/-250 deg/s
            i2c.writeto_mem(imu_address, 0x1C, b'\x00')  # accel +/-2 g
            i2c.writeto_mem(imu_address, 0x1A, b'\x03')  # DLPF
            imu_available = True
except Exception as error:
    print('I2C/OLED/IMU initialization failed:', error)

# Reference acceleration recorded with the CHAIR LEVEL AND STATIONARY.
# This defines level, but does NOT identify which way is physically forward.
# We retain projected sensor X as nominal forward. Verify signs manually.
IMU_LEVEL_ACCEL = (-5.11, 0.22, 8.41)

def _unit(v):
    length = sqrt(sum(c * c for c in v))
    return tuple(c / length for c in v)

# New Z is the measured upward specific-force vector when level.
_level_z = _unit(IMU_LEVEL_ACCEL)
# New X is the projection of the original sensor X into the level plane.
_dot = _level_z[0]
_level_x = _unit((1.0 - _dot * _level_z[0],
                  -_dot * _level_z[1], -_dot * _level_z[2]))
# New Y = Z cross X (right-handed orthonormal coordinate frame).
_level_y = (_level_z[1]*_level_x[2] - _level_z[2]*_level_x[1],
            _level_z[2]*_level_x[0] - _level_z[0]*_level_x[2],
            _level_z[0]*_level_x[1] - _level_z[1]*_level_x[0])

def imu_to_level(x, y, z):
    return (x*_level_x[0] + y*_level_x[1] + z*_level_x[2],
            x*_level_y[0] + y*_level_y[1] + z*_level_y[2],
            x*_level_z[0] + y*_level_z[1] + z*_level_z[2])
ACCEL_SCALE = 9.80665 / 16384.0
GYRO_SCALE = 1.0 / 131.0
IMU_FILTER_TAU_S = 0.6
imu_ax = imu_ay = imu_az = 0.0
imu_gx = imu_gy = imu_gz = 0.0
imu_pitch = imu_roll = imu_yaw = 0.0
imu_last_ms = None
imu_initialized = False
imu_last_good_ms = None
IMU_RETRY_MS = 2000
imu_last_retry_ms = ticks_ms()

def _signed16(msb, lsb):
    value = (msb << 8) | lsb
    return value - 65536 if value >= 32768 else value

def update_imu(now):
    global imu_available, imu_ax, imu_ay, imu_az
    global imu_gx, imu_gy, imu_gz, imu_pitch, imu_roll, imu_yaw
    global imu_last_ms, imu_initialized, imu_last_good_ms, imu_last_retry_ms
    if not imu_available:
        if i2c is None or ticks_diff(now, imu_last_retry_ms) < IMU_RETRY_MS:
            return
        imu_last_retry_ms = now
        try:
            who = i2c.readfrom_mem(imu_address, 0x75, 1)[0]
            if who not in (0x70, 0x68, 0x71, 0x73):
                return
            i2c.writeto_mem(imu_address, 0x6B, b'\x01')
            i2c.writeto_mem(imu_address, 0x1B, b'\x00')
            i2c.writeto_mem(imu_address, 0x1C, b'\x00')
            i2c.writeto_mem(imu_address, 0x1A, b'\x03')
            imu_available = True
            imu_initialized = False
        except Exception:
            return
    try:
        raw = i2c.readfrom_mem(imu_address, 0x3B, 14)
        ax = _signed16(raw[0], raw[1]) * ACCEL_SCALE
        ay = _signed16(raw[2], raw[3]) * ACCEL_SCALE
        az = _signed16(raw[4], raw[5]) * ACCEL_SCALE
        gx = _signed16(raw[8], raw[9]) * GYRO_SCALE
        gy = _signed16(raw[10], raw[11]) * GYRO_SCALE
        gz = _signed16(raw[12], raw[13]) * GYRO_SCALE
        # Rotate both acceleration and angular velocity into level frame.
        ax, ay, az = imu_to_level(ax, ay, az)
        gx, gy, gz = imu_to_level(gx, gy, gz)
        roll_acc = atan2(ay, az) * 180.0 / pi
        pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / pi
        if not imu_initialized:
            imu_roll, imu_pitch, imu_yaw = roll_acc, pitch_acc, 0.0
            imu_initialized = True
        else:
            dt = clamp(ticks_diff(now, imu_last_ms) / 1000.0, 0.0, 0.1)
            alpha = IMU_FILTER_TAU_S / (IMU_FILTER_TAU_S + dt)
            imu_roll = alpha * (imu_roll + gx * dt) + (1.0 - alpha) * roll_acc
            imu_pitch = alpha * (imu_pitch + gy * dt) + (1.0 - alpha) * pitch_acc
            imu_yaw += gz * dt  # unbounded relative yaw; gyro drift
        imu_ax, imu_ay, imu_az = ax, ay, az
        imu_gx, imu_gy, imu_gz = gx, gy, gz
        imu_last_ms = now
        imu_last_good_ms = now
    except Exception as error:
        # Sensor failure does not silently produce a valid tilt measurement.
        imu_available = False
        imu_initialized = False
        imu_last_retry_ms = now
        print('IMU read failed; retrying:', error)

# ============================================================
# OLED (6 ORIGINAL ROWS, PITCH REPLACES BRAKES)
# ============================================================
def oled_write(text, row):
    if oled is None:
        return
    x = DISPLAY_X_OFFSET
    oled.text(str(text)[:(128 - x) // 8], x, row * 10)

def display_information(mode, speed, throttle, steering, reed_value, wifi_status):
    if oled is None:
        return
    try:
        oled.fill(0)
        oled_write('{} {}'.format(mode, speed), 0)
        oled_write('Speed:{:.1f}km/h'.format(SPEED_KMH), 1)
        oled_write('Throttle:{:+.0f}%'.format(throttle), 2)
        oled_write('Steering:{:+.0f}%'.format(steering), 3)
        oled_write('Pitch:{:+.1f}deg'.format(imu_pitch) if imu_available and imu_initialized else 'Pitch: RETRY', 4)
        oled_write('WiFi: {}'.format(wifi_status), 5)
        oled.show()
    except Exception as error:
        print('OLED update failed:', error)

# ============================================================
# OUTPUT STOP / MAIN LOOP
# ============================================================
def stop_outputs():
    global motor_current_v, rear_current_v, motor_accel_step_v
    motor_current_v = 0.0
    rear_current_v = 0.0
    motor_accel_step_v = MOTOR_START_SLEW_STEP_V
    disable_motors()
    select_motor_direction('FORWARD')
    disable_servo()
    # Return handbrake actuator to rest when supervisor stops control.
    # This is NOT a guaranteed safe brake position; depends on mechanics.
    set_handbrake_servo_us(HANDBRAKE_REST_US)

def run(stop_requested=None, log=print):
    global servo_current_us, servo_target_filtered_us, servo_last_active_ms
    global motor_current_v, rear_current_v, motor_accel_step_v
    stop_outputs()
    reset_handbrake()
    servo_current_us = float(SERVO_CENTER)
    servo_target_filtered_us = float(SERVO_CENTER)
    set_servo_us(SERVO_CENTER)
    servo_last_active_ms = ticks_ms()
    last_diagnostic_ms = ticks_ms()
    last_display_ms = ticks_ms()
    last_servo_update_ms = ticks_ms()
    previous_mode = previous_speed = previous_direction = None
    previous_reed = previous_handbrake = previous_wifi = None
    throttle_armed = False
    neutral_since_ms = None
    THROTTLE_ARM_NEUTRAL_MS = 500
    DIRECTION_NEUTRAL_MS = 800
    selected_motor_direction = 'FORWARD'
    neutral_for_direction_since = None
    neutral_direction_ready = False
    log('Wheelchair control started.')
    log('Steering servo: GP9; handbrake servo: GP22')
    log('Traction motor PWM enabled.')
    log('IMU WHO_AM_I: {}; active: {}'.format(imu_whoami, imu_available))
    try:
        while True:
            loop_start = ticks_ms()
            if stop_requested is not None and stop_requested():
                stop_outputs()
                log('Wheelchair control stopped.')
                return

            x, y = x_adc.read_u16(), y_adc.read_u16()
            x_percent, y_percent = get_x_percentage(x), get_y_percentage(y)
            mode, speed = read_mode(), read_speed()
            reed_value, handbrake_value = read_reed_switch(), read_handbrake_button()
            direction = get_direction(y_percent)

            # Nonblocking peripherals. I2C operations may still consume time.
            handbrake_actuating = update_handbrake(loop_start, log)
            update_wheel_speed(loop_start)
            update_imu(loop_start)

            steering_percent = calculate_steering(x_percent, direction)
            servo_target = steering_to_pulse(steering_percent)
            elapsed_servo_ms = max(1, ticks_diff(loop_start, last_servo_update_ms))
            last_servo_update_ms = loop_start
            servo_us = update_servo(servo_target, mode, speed, x, elapsed_servo_ms)

            requested_throttle = 100.0 * get_throttle_fraction(y)
            signed_throttle = -requested_throttle if direction == 'REVERSE' else requested_throttle
            joystick_neutral = abs(y - Y_CENTER) <= Y_DEADZONE
            # During servo actuation, inhibit traction as an additional
            # precaution. GP1 reed remains the physical brake interlock.
            brakes_on = (reed_value == 0) or handbrake_actuating
            if brakes_on:
                throttle_armed = False
                neutral_since_ms = None
                disable_motors()
            if not throttle_armed and not brakes_on:
                if joystick_neutral:
                    if neutral_since_ms is None:
                        neutral_since_ms = loop_start
                    elif ticks_diff(loop_start, neutral_since_ms) >= THROTTLE_ARM_NEUTRAL_MS:
                        throttle_armed = True
                else:
                    neutral_since_ms = None

            if joystick_neutral:
                if neutral_for_direction_since is None:
                    neutral_for_direction_since = loop_start
                if ticks_diff(loop_start, neutral_for_direction_since) >= DIRECTION_NEUTRAL_MS:
                    neutral_direction_ready = True
            else:
                neutral_for_direction_since = None
            if (direction != 'NEUTRAL' and direction != selected_motor_direction
                    and neutral_direction_ready and not brakes_on):
                disable_motors()
                motor_current_v = 0.0
                rear_current_v = 0.0
                motor_accel_step_v = MOTOR_START_SLEW_STEP_V
                select_motor_direction(direction)
                selected_motor_direction = direction
                neutral_direction_ready = False

            drive_allowed = (throttle_armed and not brakes_on and
                             direction == selected_motor_direction and direction != 'NEUTRAL')
            motor_target_v = joystick_to_motor_voltage(y, mode, speed) if drive_allowed else 0.0
            front_v, rear_v = update_motor_voltage(motor_target_v, mode, speed)
            # Extra immediate brake check after motor command (original
            # set_motor_voltages() also checks GP1).
            if read_reed_switch() == 0 or handbrake_actuating:
                disable_motors()

            if mode != previous_mode:
                log('Mode: {}'.format(mode))
                previous_mode = mode
            if speed != previous_speed:
                log('Speed setting: {}'.format(speed))
                previous_speed = speed
            if direction != previous_direction:
                log('Joystick direction: {}'.format(direction))
                previous_direction = direction
            if reed_value != previous_reed:
                log('Brakes: {}'.format('ON' if reed_value == 0 else 'OFF'))
                previous_reed = reed_value
            if handbrake_value != previous_handbrake:
                log('Handbrake button: {}'.format('ON' if handbrake_value == 0 else 'OFF'))
                previous_handbrake = handbrake_value

            now = ticks_ms()
            diagnostic_due = ticks_diff(now, last_diagnostic_ms) >= DIAGNOSTIC_INTERVAL_MS
            display_due = ticks_diff(now, last_display_ms) >= DISPLAY_INTERVAL_MS
            if diagnostic_due or display_due:
                wifi_status = get_wifi_status()
                if wifi_status != previous_wifi:
                    log('WiFi: {}'.format(wifi_status))
                    previous_wifi = wifi_status
            if diagnostic_due:
                last_diagnostic_ms = now
                log(('X:{:+.1f}% Y:{:+.1f}% | Mode:{} Speed:{} | '
                     'Speed:{:.2f}km/h | Wheel RPM FR:{:.1f} BR:{:.1f} FL:{:.1f} BL:{:.1f} | '
                     'Servo:{}us | Brakes:{} | Handbrake:{} | Handbrake servo:{}us | '
                     'Accel X:{:+.2f} Y:{:+.2f} Z:{:+.2f}m/s2 | '
                     'Pitch:{:+.1f} Roll:{:+.1f} Yaw:{:+.1f}deg | IMU:{} | WiFi:{}').format(
                    x_percent, y_percent, mode, speed, SPEED_KMH,
                    hall_rpm[0], hall_rpm[1], hall_rpm[2], hall_rpm[3], servo_us,
                    'ON' if reed_value == 0 else 'OFF',
                    'ON' if handbrake_value == 0 else 'OFF', get_handbrake_servo_us(),
                    imu_ax, imu_ay, imu_az, imu_pitch, imu_roll, imu_yaw,
                    'OK' if imu_available and imu_initialized else 'RETRY', wifi_status))
            if display_due:
                last_display_ms = now
                display_information(mode, speed, signed_throttle, steering_percent,
                                    reed_value, wifi_status)
            remaining = LOOP_PERIOD_MS - ticks_diff(ticks_ms(), loop_start)
            if remaining > 0:
                sleep_ms(remaining)
    finally:
        stop_outputs()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print('Interrupted.')
    finally:
        stop_outputs()
        print('Chair outputs stopped.')
