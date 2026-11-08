"""Wheelchair control + OLED + IMU for Pico 2 W, SSD1306 128x64.

EXPERIMENTAL ACTUATOR CONTROLLER. Motor PWM, steering and handbrake servo
are enabled when run() is called. Do not test with an occupant.
Test with traction power disconnected, then wheels lifted, and provide
an independent hardware emergency stop / motor-enable interlock.
Supervisor interface: run(stop_requested, log), stop_outputs().

Layout uses x=8..119 and y=7..56 to avoid rounded/cut-off corners.
Calibrate IMU only when wheelchair is LEVEL and STILL.
"""
from time import sleep_ms, ticks_diff, ticks_ms, ticks_us

import framebuf
import network
from machine import ADC, I2C, PWM, Pin, disable_irq, enable_irq

import ssd1306

# ============================================================
# 1. HARDWARE MAP AND CALIBRATED INPUT VALUES
# ============================================================
PIN_HAND_BRAKE_BUTTON = 0
PIN_BRAKE_REED = 1               # LOW = brakes engaged
PIN_SPEED_A, PIN_SPEED_B = 2, 19
PIN_MODE_A, PIN_MODE_B = 4, 7
PIN_REVERSE = 8
PIN_STEERING_SERVO = 9
PIN_HALL_FR, PIN_HALL_BR, PIN_HALL_FL, PIN_HALL_BL = 10, 11, 12, 13
PIN_MOTOR_FR, PIN_MOTOR_FL, PIN_MOTOR_BR, PIN_MOTOR_BL = 14, 15, 17, 18
PIN_I2C_SDA, PIN_I2C_SCL = 20, 21
PIN_HANDBRAKE_SERVO = 22
PIN_WIFI_BUTTON = 26            # Owned by supervisor
PIN_JOYSTICK_X, PIN_JOYSTICK_Y = 27, 28

X_LEFT, X_CENTER, X_RIGHT, X_DEADZONE = 0, 33815, 65351, 2500
Y_BACKWARD, Y_CENTER, Y_FORWARD, Y_DEADZONE = 5401, 43529, 65340, 1750
IMU_I2C_ADDRESS = 0x68
OLED_I2C_ADDRESS = 0x3C

# ============================================================
# 2. IMU LOGIC -- gyro tracking + stationary gravity correction
# ============================================================
# Calibrate only when the wheelchair is LEVEL and STILL.
# Track rotation with the gyro while driving. Use gravity to correct drift
# only after the chair stops. This is not a safety-rated tilt measurement.
from math import atan2, pi, sqrt

G = 9.80665
ACC_SCALE = G / 16384.0
GYRO_SCALE = pi / (180.0 * 131.0)  # rad/s per LSB at +/-250 deg/s
IMU_REST_CORRECTION = 0.06
IMU_ZERO_ANGLE_DEG = 0.25
IMU_CAL_SAMPLES = 80
IMU_CAL_MAX_ATTEMPTS = 600
IMU_MIN_G = 8.5
IMU_MAX_G = 11.1
IMU_SAMPLE_INTERVAL_MS = 20
IMU_MAX_GAP_MS = 100

_imu_i2c = None
_imu_basis = None
_imu_filtered = None
_imu_gyro_bias = None
_imu_pitch_deg = 0.0
_imu_ok = False
_imu_last_sample = 0
_imu_last_good_sample = None
_imu_error = None

def signed16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


def magnitude(v):
    return sqrt(v[0]**2 + v[1]**2 + v[2]**2)


def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1],
            a[2]*b[0]-a[0]*b[2],
            a[0]*b[1]-a[1]*b[0])


def normalize(v):
    n = magnitude(v)
    if n < 1e-8:
        raise ValueError('Invalid reference vector')
    return tuple(t/n for t in v)


def create_basis(reference):
    z = normalize(reference)
    x_guess = (1-z[0]*z[0], -z[0]*z[1], -z[0]*z[2])
    if magnitude(x_guess) < 0.1:
        y = normalize((-z[1]*z[0], 1-z[1]*z[1], -z[1]*z[2]))
        x = normalize(cross(y, z))
    else:
        x = normalize(x_guess)
        y = normalize(cross(z, x))
    return x, y, z



def imu_calibrate(stop_requested=None):
    """Capture the level reference and gyro bias while stationary."""
    global _imu_basis, _imu_filtered, _imu_gyro_bias
    accel_sums = [0.0, 0.0, 0.0]
    gyro_sums = [0.0, 0.0, 0.0]
    good = 0
    for _ in range(IMU_CAL_MAX_ATTEMPTS):
        if stop_requested is not None and stop_requested():
            return False
        try:
            accel, gyro = imu_read_motion()
            if IMU_MIN_G <= magnitude(accel) <= IMU_MAX_G:
                for i in range(3):
                    accel_sums[i] += accel[i]
                    gyro_sums[i] += gyro[i]
                good += 1
                if good >= IMU_CAL_SAMPLES:
                    break
        except OSError:
            pass
        sleep_ms(30)
    if good < IMU_CAL_SAMPLES:
        raise RuntimeError(f"Only {good} valid IMU samples")
    _imu_basis = create_basis(tuple(accel_sums))
    _imu_gyro_bias = tuple(value / good for value in gyro_sums)
    _imu_filtered = (0.0, 0.0, 1.0)
    return True


def imu_read_motion():
    if _imu_i2c is None:
        raise OSError("IMU I2C is not initialized")
    # One burst contains acceleration, temperature, then angular velocity.
    raw = _imu_i2c.readfrom_mem(IMU_I2C_ADDRESS, 0x3B, 14)
    if len(raw) != 14:
        raise OSError("Short IMU read")
    accel = tuple(signed16(raw[i], raw[i+1]) * ACC_SCALE for i in (0, 2, 4))
    gyro = tuple(signed16(raw[i], raw[i+1]) * GYRO_SCALE for i in (8, 10, 12))
    return accel, gyro


def imu_rotate(v):
    return tuple(dot(v, axis) for axis in _imu_basis) # type: ignore


def imu_initialize(i2c, stop_requested=None, log=print):
    """Calibrate on first start and retain the level reference on a restart."""
    global _imu_i2c, _imu_ok, _imu_error, _imu_filtered
    global _imu_last_sample, _imu_last_good_sample
    _imu_i2c = i2c
    _imu_ok = False
    _imu_filtered = None
    _imu_last_good_sample = None
    try:
        devices = i2c.scan()
        if IMU_I2C_ADDRESS not in devices:
            raise OSError("IMU 0x68 not found")
        who = i2c.readfrom_mem(IMU_I2C_ADDRESS, 0x75, 1)[0]
        log(f"IMU WHO_AM_I: 0x{who:02X}")
        imu_configure()
        if _imu_basis is None:
            if not imu_calibrate(stop_requested):
                return False
            _imu_last_good_sample = ticks_ms()
            _imu_ok = True
            log("IMU level and gyro calibration successful")
        _imu_last_sample = ticks_ms()
        _imu_error = None
        return True
    except Exception as exc:  # noqa: BLE001
        _imu_ok = False
        _imu_error = repr(exc)
        log(f"IMU initialization failed: {_imu_error}")
        return False


def imu_configure():
    i2c = _imu_i2c
    if i2c is None:
        raise OSError("IMU I2C is not initialized")
    i2c.writeto_mem(IMU_I2C_ADDRESS, 0x6B, b"\x01")
    sleep_ms(100)
    i2c.writeto_mem(IMU_I2C_ADDRESS, 0x1C, b"\x00")
    i2c.writeto_mem(IMU_I2C_ADDRESS, 0x1B, b"\x00")
    i2c.writeto_mem(IMU_I2C_ADDRESS, 0x1A, b"\x03")
    sleep_ms(100)


def imu_update(now, drive_idle):
    """Track the gravity direction with gyro rotation while moving."""
    global _imu_filtered, _imu_pitch_deg, _imu_ok, _imu_error
    global _imu_last_sample, _imu_last_good_sample
    if _imu_basis is None or _imu_gyro_bias is None:
        return _imu_pitch_deg
    if ticks_diff(now, _imu_last_sample) < IMU_SAMPLE_INTERVAL_MS:
        return _imu_pitch_deg
    _imu_last_sample = now

    dt_ms = (ticks_diff(now, _imu_last_good_sample)
             if _imu_last_good_sample is not None else 0)
    if dt_ms > IMU_MAX_GAP_MS:
        _imu_filtered = None
    try:
        accel, gyro = imu_read_motion()
    except OSError as exc:
        _imu_ok = False
        _imu_error = repr(exc)
        return _imu_pitch_deg
    _imu_last_good_sample = now

    gravity = _imu_filtered
    if gravity is not None:
        rate = imu_rotate(tuple(gyro[i] - _imu_gyro_bias[i] for i in range(3)))
        # Gravity rotates opposite the chair in the chair's coordinate frame.
        change = cross(gravity, rate)
        gravity = normalize(tuple(gravity[i] + change[i] * dt_ms / 1000.0
                                  for i in range(3)))

    at_rest = (
        drive_idle and SPEED_KMH == 0.0
        and all(ticks_diff(now, edge) >= HALL_STALE_MS for edge in hall_last_edge_ms)
    )
    if at_rest and IMU_MIN_G <= magnitude(accel) <= IMU_MAX_G:
        reference = normalize(imu_rotate(accel))
        if gravity is None:
            gravity = reference
        else:
            alpha = IMU_REST_CORRECTION
            gravity = normalize(tuple(gravity[i] + alpha * (reference[i] - gravity[i])
                                      for i in range(3)))

    _imu_filtered = gravity
    _imu_ok = gravity is not None
    if gravity is None:
        _imu_error = "Waiting for stationary tilt reference"
        return _imu_pitch_deg

    ax, ay, az = gravity
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / pi
    _imu_pitch_deg = 0.0 if abs(pitch) < IMU_ZERO_ANGLE_DEG else pitch
    _imu_error = None
    return _imu_pitch_deg


def get_pitch_degrees():
    return _imu_pitch_deg


def imu_is_healthy():
    return _imu_ok


# ============================================================
# 3. THROTTLE LOGIC -- joystick, ramps, motor PWM
# ============================================================

PICO_PWM_VOLTAGE = 3.3
LOOP_PERIOD_MS = 5

def clamp(value, low, high):
    return max(low, min(high, value))

x_adc = ADC(Pin(PIN_JOYSTICK_X))
y_adc = ADC(Pin(PIN_JOYSTICK_Y))
mode_gp4 = Pin(PIN_MODE_A, Pin.IN, Pin.PULL_UP)
mode_gp7 = Pin(PIN_MODE_B, Pin.IN, Pin.PULL_UP)
speed_gp2 = Pin(PIN_SPEED_A, Pin.IN, Pin.PULL_UP)
speed_gp19 = Pin(PIN_SPEED_B, Pin.IN, Pin.PULL_UP)
current_mode = 'TERRAIN'
current_speed = 'LOW'

def read_mode():
    global current_mode
    pins = (mode_gp4.value(), mode_gp7.value())
    if pins == (1, 1):
        current_mode = 'TERRAIN'
    elif pins == (1, 0):
        current_mode = 'DIFFERENTIAL'
    elif pins == (0, 1):
        current_mode = 'ROAD'
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

def get_x_percentage(x):
    if abs(x - X_CENTER) <= X_DEADZONE:
        return 0.0
    if x < X_CENTER:
        return -100.0 * clamp((X_CENTER-X_DEADZONE-x) /
                              (X_CENTER-X_DEADZONE-X_LEFT), 0, 1)
    return 100.0 * clamp((x-X_CENTER-X_DEADZONE) /
                         (X_RIGHT-X_CENTER-X_DEADZONE), 0, 1)

def get_y_percentage(y):
    if abs(y - Y_CENTER) <= Y_DEADZONE:
        return 0.0
    if y < Y_CENTER:
        return -100.0 * clamp((Y_CENTER-Y_DEADZONE-y) /
                              (Y_CENTER-Y_DEADZONE-Y_BACKWARD), 0, 1)
    return 100.0 * clamp((y-Y_CENTER-Y_DEADZONE) /
                         (Y_FORWARD-Y_CENTER-Y_DEADZONE), 0, 1)

def get_throttle_request(y=None):
    if y is None:
        y = y_adc.read_u16()
    return abs(get_y_percentage(y)) / 100.0

def get_direction(y_percent):
    if y_percent > 0:
        return 'FORWARD'
    if y_percent < 0:
        return 'REVERSE'
    return 'NEUTRAL'

# GP8: LOW = REVERSE, released = FORWARD.
# Requires a suitable external 3.3 V pull-up.
reverse_pin = Pin(PIN_REVERSE, Pin.OPEN_DRAIN, value=1)

def select_motor_direction(direction):
    if direction == 'REVERSE':
        reverse_pin.value(0)
    elif direction == 'FORWARD':
        reverse_pin.value(1)
    else:
        raise ValueError('Invalid motor direction')

MIN_MOTOR_VOLTAGE = 1.60
STARTUP_SPEED_KMH = 2.0
STARTUP_MAX_STEP_FRACTION = 0.5
ROAD_MAX_VOLTAGE = {'LOW': 1.90, 'MEDIUM': 2.20, 'HIGH': 3.30}
AXLE_BIASED_FRONT_MAX_VOLTAGE = 1.80
AXLE_BIAS_MULTIPLIER = {'LOW': 1.00, 'MEDIUM': 1.2, 'HIGH': 1.5}
THROTTLE_EXPONENT = 2.0
MIN_VOLTAGE_REGION = 0.05

MOTOR_RAMP_SETTINGS = {
    'LOW': {
        'max_step': 0.003,
        'start_step': 0.00010,
        'acceleration': 0.000010,
    },
    'MEDIUM': {
        'max_step': 0.002,
        'start_step': 0.00008,
        'acceleration': 0.000006,
    },
    'HIGH': {
        'max_step': 0.0015,
        'start_step': 0.00006,
        'acceleration': 0.000004,
    },
}

MOTOR_RAMP_MODE_MULTIPLIER = {
    'TERRAIN': 4.0,
    'DIFFERENTIAL': 4.0,
    'ROAD': 1.0,
}

# Default initialization value.
MOTOR_START_SLEW_STEP_V = MOTOR_RAMP_SETTINGS['LOW']['start_step']

MOTOR_PWM_FREQUENCY = 20000
motor_current_v = 0.0
rear_current_v = 0.0
motor_accel_step_v = MOTOR_START_SLEW_STEP_V
motor_BR = PWM(Pin(PIN_MOTOR_BR))
motor_BL = PWM(Pin(PIN_MOTOR_BL))
motor_FR = PWM(Pin(PIN_MOTOR_FR))
motor_FL = PWM(Pin(PIN_MOTOR_FL))
MOTOR_PWMS = (motor_BR, motor_BL, motor_FR, motor_FL)
FRONT_MOTOR_PWMS = (motor_FR, motor_FL)
REAR_MOTOR_PWMS = (motor_BR, motor_BL)
for pwm in MOTOR_PWMS:
    pwm.freq(MOTOR_PWM_FREQUENCY)
    pwm.duty_u16(0)

def disable_motors():
    for pwm in MOTOR_PWMS:
        pwm.duty_u16(0)


def reset_motor_ramp():
    global motor_current_v, rear_current_v, motor_accel_step_v

    motor_current_v = 0.0
    rear_current_v = 0.0

    # Reset to the initial ramp step.
    # update_motor_voltage() selects the correct speed-specific
    # value when the next throttle request begins.
    motor_accel_step_v = MOTOR_RAMP_SETTINGS['LOW']['start_step']

    disable_motors()

def voltage_to_duty(voltage):
    return round(clamp(voltage, 0, PICO_PWM_VOLTAGE) /
                     PICO_PWM_VOLTAGE * 65535)

def set_motor_voltages(front_voltage, rear_voltage):
    # This function is a last-line software interlock, not an independent E-stop.
    if read_reed_switch() == 0 or handbrake_active_since is not None:
        reset_motor_ramp()
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

# Verify that nose-up reads positive after a level calibration.
UPHILL_PITCH_SIGN = 1

def get_uphill_gear(mode, speed, direction, uphill_deg):
    if mode != 'DIFFERENTIAL' or direction != 'FORWARD':
        return speed
    if uphill_deg > 17.0:
        return 'HIGH'
    if uphill_deg > 10.0 and speed == 'LOW':
        return 'MEDIUM'
    return speed

def get_front_max_voltage(mode, speed):
    if mode in ('TERRAIN', 'DIFFERENTIAL'):
        return AXLE_BIASED_FRONT_MAX_VOLTAGE
    return ROAD_MAX_VOLTAGE[speed]

def joystick_to_motor_voltage(y, mode, speed, direction):
    throttle = get_throttle_request(y)
    if throttle <= 0.0:
        return 0.0
    if throttle <= MIN_VOLTAGE_REGION:
        return MIN_MOTOR_VOLTAGE
    maximum = (
        PICO_PWM_VOLTAGE if direction == 'REVERSE'
        else get_front_max_voltage(mode, speed)
    )
    fraction = clamp((throttle - MIN_VOLTAGE_REGION) /
                     (1.0 - MIN_VOLTAGE_REGION), 0, 1)
    return MIN_MOTOR_VOLTAGE + fraction**THROTTLE_EXPONENT * (maximum-MIN_MOTOR_VOLTAGE)

def get_other_axle_voltage(base_voltage, mode, speed, direction):
    if mode == 'ROAD':
        return base_voltage
    multiplier = AXLE_BIAS_MULTIPLIER[speed]
    if direction == 'REVERSE':
        # Keep the axle following the stick stronger at full reverse.
        return max(MIN_MOTOR_VOLTAGE, base_voltage / multiplier)
    return min(PICO_PWM_VOLTAGE, base_voltage * multiplier)

def update_motor_voltage(target_v, mode, speed, direction, slow_start):
    global motor_current_v, rear_current_v, motor_accel_step_v

    # Select acceleration parameters according to speed setting.
    ramp = MOTOR_RAMP_SETTINGS[speed]

    multiplier = MOTOR_RAMP_MODE_MULTIPLIER[mode]

    max_step = ramp['max_step'] * multiplier
    start_step = ramp['start_step'] * multiplier
    acceleration = ramp['acceleration'] * multiplier

    # Only TERRAIN reverse takes its primary command from the rear axle.
    rear_is_primary = mode == 'TERRAIN' and direction == 'REVERSE'
    if rear_is_primary:
        primary_v, other_v = rear_current_v, motor_current_v
    else:
        primary_v, other_v = motor_current_v, rear_current_v

    # Start at the lowest command that produces PWM.
    if target_v > 0.0 and primary_v < MIN_MOTOR_VOLTAGE:
        primary_v = MIN_MOTOR_VOLTAGE
        motor_accel_step_v = start_step

    elif primary_v < target_v:
        step = max_step
        if slow_start:
            startup_limit = max_step * STARTUP_MAX_STEP_FRACTION
            step = min(motor_accel_step_v, startup_limit)
            motor_accel_step_v = min(step + acceleration, startup_limit)
        primary_v += min(step, target_v - primary_v)

    elif primary_v > target_v:
        primary_v -= min(
            max_step,
            primary_v - target_v
        )

    primary_v = clamp(
        primary_v, 0, PICO_PWM_VOLTAGE
    )

    # The other axle follows the primary command with the mode's bias.
    other_target_v = (
        get_other_axle_voltage(primary_v, mode, speed, direction)
        if target_v > 0.0 else 0.0
    )

    if target_v > 0.0 and other_v < MIN_MOTOR_VOLTAGE:
        other_v = MIN_MOTOR_VOLTAGE

    elif other_v < other_target_v:
        other_v += min(
            max_step,
            other_target_v - other_v
        )

    else:
        other_v -= min(
            max_step,
            other_v - other_target_v
        )

    other_v = clamp(
        other_v, 0, PICO_PWM_VOLTAGE
    )

    if rear_is_primary:
        motor_current_v, rear_current_v = other_v, primary_v
    else:
        motor_current_v, rear_current_v = primary_v, other_v
    if mode == 'TERRAIN':
        # A mode change must not carry a front bias into TERRAIN.
        motor_current_v = min(motor_current_v, rear_current_v)

    set_motor_voltages(motor_current_v, rear_current_v)
    return motor_current_v, rear_current_v

# ============================================================
# 4. STEERING LOGIC -- constant slew, speed-dependent travel
# ============================================================

servo = PWM(Pin(PIN_STEERING_SERVO))
servo.freq(50)

SERVO_LEFT = 1100
SERVO_CENTER = 1500
SERVO_RIGHT = 1900

SERVO_RELEASE_DELAY_MS = 500

# Servo slew rate in microseconds of pulse width per second.
STEERING_SPEED_US_PER_SEC = 1600.0
# Measured speed and fraction of normal steering travel in ROAD.
ROAD_STEERING_TRAVEL = ((7.0, 1.0), (10.0, 0.8), (15.0, 0.7), (20.0, 0.5))

# Steering position updated at 50 Hz.
# The rest of the control loop remains at approximately 200 Hz.
STEERING_UPDATE_INTERVAL_MS = 20

servo_current_us = float(SERVO_CENTER)
servo_last_active_ms = ticks_ms()
servo_last_update_ms = ticks_ms()
servo_enabled = False


def set_servo_us(pulse_us):
    global servo_enabled

    pulse_us = clamp(
        pulse_us,
        SERVO_LEFT,
        SERVO_RIGHT
    )

    servo.duty_u16(
        int(pulse_us * 65535 / 20000)
    )

    servo_enabled = True


def disable_servo():
    global servo_enabled

    servo.duty_u16(0)
    servo_enabled = False


def steering_to_pulse(steering_percent):
    p = clamp(steering_percent, -100.0, 100.0)

    if p < 0:
        return int(
            SERVO_CENTER
            + p / 100.0 * (SERVO_CENTER - SERVO_LEFT)
        )

    return int(
        SERVO_CENTER
        + p / 100.0 * (SERVO_RIGHT - SERVO_CENTER)
    )


def calculate_steering(x_percent, mode, wheelchair_kmh):
    if mode != 'ROAD' or wheelchair_kmh <= ROAD_STEERING_TRAVEL[0][0]:
        return x_percent

    low_speed, low_travel = ROAD_STEERING_TRAVEL[0]
    for high_speed, high_travel in ROAD_STEERING_TRAVEL[1:]:
        if wheelchair_kmh <= high_speed:
            fraction = (wheelchair_kmh - low_speed) / (high_speed - low_speed)
            travel = low_travel + fraction * (high_travel - low_travel)
            break
        low_speed, low_travel = high_speed, high_travel
    else:
        travel = low_travel

    return x_percent * travel


def update_servo(target_us, joystick_x):

    global servo_current_us
    global servo_last_active_ms
    global servo_last_update_ms

    now = ticks_ms()

    joystick_active = (
        abs(joystick_x - X_CENTER) > X_DEADZONE
    )

    if joystick_active:
        servo_last_active_ms = now

    # Time elapsed since the previous steering update.
    dt_ms = ticks_diff(now, servo_last_update_ms)

    # Keep the main loop fast, but update steering at 50 Hz.
    if dt_ms < STEERING_UPDATE_INTERVAL_MS:
        return round(servo_current_us)

    servo_last_update_ms = now

    max_step = STEERING_SPEED_US_PER_SEC * dt_ms / 1000.0

    # Difference between target and current steering position.
    difference = target_us - servo_current_us

    # Constant-speed movement toward target.
    servo_current_us += clamp(
        difference,
        -max_step,
        max_step
    )

    # Snap to center when sufficiently close.
    if (
        abs(servo_current_us - SERVO_CENTER) < 0.5
        and not joystick_active
    ):
        servo_current_us = float(SERVO_CENTER)

    # Release servo after returning to center.
    if (
        not joystick_active
        and servo_current_us == SERVO_CENTER
        and ticks_diff(now, servo_last_active_ms)
            >= SERVO_RELEASE_DELAY_MS
    ):
        disable_servo()
    else:
        set_servo_us(servo_current_us)

    return round(servo_current_us)

# ============================================================
# 5. BRAKE LOGIC -- reed interlock + handbrake servo
# ============================================================
reed_switch = Pin(PIN_BRAKE_REED, Pin.IN, Pin.PULL_UP)
handbrake_button = Pin(PIN_HAND_BRAKE_BUTTON, Pin.IN, Pin.PULL_UP)
HANDBRAKE_REST_US = 1500
HANDBRAKE_ACTIVE_US = 900
HANDBRAKE_HOLD_MS = 5000
HANDBRAKE_DEBOUNCE_MS = 40
handbrake_pwm = PWM(Pin(PIN_HANDBRAKE_SERVO))
handbrake_pwm.freq(50)
handbrake_servo_us = HANDBRAKE_REST_US
handbrake_active_since = None
handbrake_press_latched = False
handbrake_last_raw = handbrake_button.value()
handbrake_raw_since = ticks_ms()
handbrake_stable = handbrake_last_raw

def read_reed_switch():
    return reed_switch.value()

def get_brake_status():
    return 'ON' if read_reed_switch() == 0 else 'OFF'

def set_handbrake_servo_us(pulse_us):
    global handbrake_servo_us
    handbrake_servo_us = int(clamp(pulse_us, 900, 2100))
    handbrake_pwm.duty_u16(handbrake_servo_us*65535//20000)

def get_handbrake_servo_us():
    return handbrake_servo_us

def reset_handbrake():
    global handbrake_active_since, handbrake_press_latched
    global handbrake_last_raw, handbrake_raw_since, handbrake_stable
    handbrake_active_since = None
    handbrake_last_raw = handbrake_button.value()
    handbrake_stable = handbrake_last_raw
    handbrake_raw_since = ticks_ms()
    handbrake_press_latched = (handbrake_stable == 0)
    set_handbrake_servo_us(HANDBRAKE_REST_US)

def update_handbrake(now, log=print):
    global handbrake_active_since, handbrake_press_latched
    global handbrake_last_raw, handbrake_raw_since, handbrake_stable
    raw = handbrake_button.value()
    if raw != handbrake_last_raw:
        handbrake_last_raw = raw
        handbrake_raw_since = now
    if ticks_diff(now, handbrake_raw_since) >= HANDBRAKE_DEBOUNCE_MS:  # noqa: SIM102
        if raw != handbrake_stable:
            handbrake_stable = raw
            if raw == 1:
                handbrake_press_latched = False
            elif not handbrake_press_latched and handbrake_active_since is None:
                handbrake_press_latched = True
                handbrake_active_since = now
                set_handbrake_servo_us(HANDBRAKE_ACTIVE_US)
                log(f'Handbrake servo: {handbrake_servo_us}us (activated)')
    if (handbrake_active_since is not None and
            ticks_diff(now, handbrake_active_since) >= HANDBRAKE_HOLD_MS):
        handbrake_active_since = None
        set_handbrake_servo_us(HANDBRAKE_REST_US)
        log(f'Handbrake servo: {handbrake_servo_us}us (returned)')
    return handbrake_active_since is not None

# ============================================================
# 6. WHEEL SPEED LOGIC -- four Hall sensors
# ============================================================
WHEEL_RADIUS_M = 0.225
HALL_TRANSITIONS_PER_REV = 16
HALL_MIN_EDGE_US = 1500
HALL_STALE_MS = 1500
HALL_SAMPLE_MS = 200
hall_pins = (Pin(PIN_HALL_FR, Pin.IN, Pin.PULL_UP),
             Pin(PIN_HALL_BR, Pin.IN, Pin.PULL_UP),
             Pin(PIN_HALL_FL, Pin.IN, Pin.PULL_UP),
             Pin(PIN_HALL_BL, Pin.IN, Pin.PULL_UP))
hall_counts = [0, 0, 0, 0]
hall_last_edge_us = [ticks_us() - HALL_MIN_EDGE_US] * 4
hall_last_edge_ms = [ticks_ms()] * 4
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
    hall_pins[i].irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
                     handler=_hall_handlers[i])

def update_wheel_speed(now):
    global hall_last_sample_ms, SPEED_KMH
    dt_ms = ticks_diff(now, hall_last_sample_ms)
    if dt_ms < HALL_SAMPLE_MS:
        return
    irq_state = disable_irq()
    try:
        counts = tuple(hall_counts)
        last_edges = tuple(hall_last_edge_ms)
    finally:
        enable_irq(irq_state)
    for i in range(4):
        delta = counts[i]-hall_prev_counts[i]
        rpm = delta*60000.0/(HALL_TRANSITIONS_PER_REV*dt_ms)
        if ticks_diff(now, last_edges[i]) >= HALL_STALE_MS:
            rpm = 0.0
        hall_rpm[i] = rpm
        hall_prev_counts[i] = counts[i]
    hall_last_sample_ms = now
    SPEED_KMH = (sum(hall_rpm)/4.0)*(2.0*pi*WHEEL_RADIUS_M)*60.0/1000.0

# ============================================================
# 7. DISPLAY STATE -- future logic should update these fields
# ============================================================
class DisplayState:
    def __init__(self):
        self.pitch_deg = 0.0
        self.imu_ok = False
        self.mode = 'TERRAIN'     # TERRAIN / DIFFERENTIAL / ROAD
        self.speed_setting = 'LOW'  # LOW / MED / HIGH
        self.speed_kmh = 0.0
        self.throttle_requested = False
        self.brake_on = False
        self.mode_changed = False
        self.speed_changed = False
        self.online = False

# ============================================================
# 8. OLED FONT AND DRAWING -- display-only functions
# ============================================================
WIDTH, HEIGHT = 128, 64
SAFE_LEFT, SAFE_RIGHT = 8, 119
SAFE_TOP, SAFE_BOTTOM = 7, 56
DISPLAY_INTERVAL_MS = 200

# Use built-in 8x8 framebuf font; scale pixels in software, no extra fonts.
_glyph = bytearray(8)
_glyph_fb = framebuf.FrameBuffer(_glyph, 8, 8, framebuf.MONO_HLSB)


def draw_text(oled, value, x, y, scale=1):
    value = str(value)

    if scale == 1:
        oled.text(value, x, y, 1)
        return

    for index, ch in enumerate(value):
        _glyph_fb.fill(0)
        _glyph_fb.text(ch, 0, 0, 1)

        char_x = x + index * 8 * scale

        for py in range(8):
            for px in range(8):
                if _glyph_fb.pixel(px, py):
                    oled.fill_rect(
                        char_x + px * scale,
                        y + py * scale,
                        scale,
                        scale,
                        1
                    )


def centered(oled, value, y, scale=1):
    value = str(value)
    width = len(value) * 8 * scale
    x = (WIDTH - width) // 2
    if x < SAFE_LEFT or x + width - 1 > SAFE_RIGHT:
        # Shrink rather than draw into the clipped corners.
        scale = 1
        width = len(value) * 8
        x = (WIDTH - width) // 2
    if x < SAFE_LEFT or x + width - 1 > SAFE_RIGHT:
        max_chars = (SAFE_RIGHT - SAFE_LEFT + 1) // 8
        value = value[:max_chars]
        width = len(value) * 8
        x = (WIDTH - width) // 2
    draw_text(oled, value, x, y, scale)

def draw_degree(oled, x, y):
    oled.rect(x, y, 3, 3, 1)

def mode_short(mode):
    return {'TERRAIN': 'TERRAIN', 'DIFFERENTIAL': 'DIFF',
            'ROAD': 'ROAD'}.get(mode, str(mode)[:10])


def draw_idle(oled, state):
    # Four evenly spaced rows within the OLED safe area.
    # Each row uses the standard 8-pixel font.

    # Row 1: Slope
    if state.imu_ok:
        text = f'slope: {state.pitch_deg:+.1f}'
        x = (WIDTH - (len(text) * 8 + 4)) // 2
        draw_text(oled, text, x, 8)
        draw_degree(oled, x + len(text) * 8, 8)
    else:
        centered(oled, 'slope: N/A', 8)

    # Row 2: Mode
    centered(oled, f'mode: {mode_short(state.mode)}', 20)

    # Row 3: Speed setting
    centered(oled, f'speed: {state.speed_setting}', 32)

    # Row 4: Network status
    centered(oled, 'ONLINE' if state.online else 'OFFLINE', 44)


def draw_driving(oled, state):
    centered(oled, 'SPEED', 8, 1)
    # e.g. '12.4' at 2x => 64 px wide; legible without corner clipping.
    centered(oled, f'{state.speed_kmh:.1f}', 20, 2)
    centered(oled, 'km/h', 39, 1)
    centered(oled, f'{mode_short(state.mode)}  {state.speed_setting}', 49, 1)


def draw_change(oled, label, value):
    centered(oled, label, 10, 1)
    # Long names stay at 1x; shorter names become 2x.
    scale = 2 if len(value) <= 7 else 1
    centered(oled, value, 29 if scale == 2 else 32, scale)


def draw_brake_warning(oled):
    centered(oled, 'THROTTLE', 9, 1)
    centered(oled, 'OFF', 21, 2)
    centered(oled, 'BRAKES ON', 44, 1)


def draw_screen(oled, state):
    oled.fill(0)
    # Warning wins over all other screens.
    if state.brake_on and state.throttle_requested:
        draw_brake_warning(oled)
    elif state.mode_changed:
        draw_change(oled, 'MODE', mode_short(state.mode))
    elif state.speed_changed:
        draw_change(oled, 'SPEED', state.speed_setting)
    elif state.throttle_requested or state.speed_kmh >= 0.5:
        draw_driving(oled, state)
    else:
        draw_idle(oled, state)
    oled.show()

# ============================================================
# 9. REAL DISPLAY STATE -- hardware values, no mock data
# ============================================================
DISPLAY_CHANGE_MS = 1800
DISPLAY_SPEED_FILTER = 0.25
_display_last_mode = None
_display_last_speed = None
_display_mode_changed_at = None
_display_speed_changed_at = None
_display_smoothed_kmh = 0.0


def initialize_display_inputs():
    # Inputs are initialized by Sections 3 and 5.
    pass


def update_display_state(state, now, mode=None, speed=None, y=None):
    global _display_last_mode, _display_last_speed
    global _display_mode_changed_at, _display_speed_changed_at
    global _display_smoothed_kmh
    state.online = network.WLAN(network.STA_IF).isconnected()
    state.pitch_deg = get_pitch_degrees()
    state.imu_ok = imu_is_healthy() and _imu_basis is not None
    state.brake_on = (read_reed_switch() == 0) or (handbrake_active_since is not None)
    if y is None:
        y = y_adc.read_u16()
    state.throttle_requested = get_throttle_request(y) > 0.0
    if mode is None:
        mode = read_mode()
    if speed is None:
        speed = read_speed()
    state.mode = mode
    state.speed_setting = speed

    if _display_last_mode is None:
        _display_last_mode = mode
    elif mode != _display_last_mode:
        _display_last_mode = mode
        _display_mode_changed_at = now

    if _display_last_speed is None:
        _display_last_speed = speed
    elif speed != _display_last_speed:
        _display_last_speed = speed
        _display_speed_changed_at = now

    state.mode_changed = (_display_mode_changed_at is not None and
                          ticks_diff(now, _display_mode_changed_at) < DISPLAY_CHANGE_MS)
    state.speed_changed = (_display_speed_changed_at is not None and
                           ticks_diff(now, _display_speed_changed_at) < DISPLAY_CHANGE_MS)
    _display_smoothed_kmh += DISPLAY_SPEED_FILTER * (SPEED_KMH - _display_smoothed_kmh)
    state.speed_kmh = max(0.0, _display_smoothed_kmh)


# ============================================================
# 10. SUPERVISOR COMPATIBILITY + SAFE CONTROL LOOP
# ============================================================
_i2c = None
_oled = None
# Time for the motor controllers to recognize a new direction.
DIRECTION_SETTLE_MS = 0

# Minimum time with throttle disabled before switching direction.
DIRECTION_PWM_OFF_MS = 0

# Maximum allowed wheel speed before switching direction.
DIRECTION_MAX_SWITCH_RPM = 0.5

DIAGNOSTIC_INTERVAL_MS = 1000


def stop_outputs():
    # Disable motor torque without unnecessarily changing direction.
    reset_motor_ramp()
    disable_servo()

    # Disable handbrake servo signal.
    handbrake_pwm.duty_u16(0)


def run(stop_requested=None, log=print):
    global _i2c, _oled, servo_current_us
    global servo_last_active_ms, servo_last_update_ms
    global _display_last_mode, _display_last_speed
    global _display_mode_changed_at, _display_speed_changed_at
    global _display_smoothed_kmh

    stop_outputs()

    # No direction is considered ready at startup.
    selected_direction = None
    requested_direction = None

    direction_state = 'WAIT_REQUEST'
    direction_state_since = ticks_ms()

    # Traction remains disabled until direction selection is complete.

    last_display_ms = ticks_ms() - DISPLAY_INTERVAL_MS
    last_log_ms = ticks_ms()
    previous_mode = None
    previous_speed = None
    state = DisplayState()
    _display_last_mode = None
    _display_last_speed = None
    _display_mode_changed_at = None
    _display_speed_changed_at = None
    _display_smoothed_kmh = 0.0

    try:
        _i2c = I2C(0, sda=Pin(PIN_I2C_SDA), scl=Pin(PIN_I2C_SCL), freq=400000)
        _oled = ssd1306.SSD1306_I2C(WIDTH, HEIGHT, _i2c,
                                    addr=OLED_I2C_ADDRESS)
        if stop_requested is not None and stop_requested():
            return
        log('Place wheelchair LEVEL and STILL for IMU calibration')
        imu_initialize(_i2c, stop_requested=stop_requested, log=log)
        if stop_requested is not None and stop_requested():
            return

        # Initialize handbrake state and center steering while traction is OFF.
        reset_handbrake()

        servo_current_us = float(SERVO_CENTER)
        servo_last_active_ms = ticks_ms()
        servo_last_update_ms = ticks_ms()

        set_servo_us(SERVO_CENTER)
        log('Wheelchair control loop started; direction interlock active')

        while True:
            loop_start = ticks_ms()
            if stop_requested is not None and stop_requested():
                break

            x = x_adc.read_u16()
            y = y_adc.read_u16()
            x_percent = get_x_percentage(x)
            y_percent = get_y_percentage(y)
            mode = read_mode()
            speed = read_speed()
            direction = get_direction(y_percent)
            joystick_neutral = (direction == 'NEUTRAL')

            # Run the brake actuator first, so its active state inhibits traction.
            handbrake_actuating = update_handbrake(loop_start, log)
            brakes_on = (read_reed_switch() == 0 or handbrake_actuating)

            update_wheel_speed(loop_start)
            drive_idle = (
                joystick_neutral and motor_current_v < MIN_MOTOR_VOLTAGE
                and rear_current_v < MIN_MOTOR_VOLTAGE
            )
            imu_update(loop_start, drive_idle)

            steering_percent = calculate_steering(x_percent, mode, SPEED_KMH)
            update_servo(steering_to_pulse(steering_percent), x)

            if brakes_on:
                reset_motor_ramp()

            # ============================================================
            # ROBUST MOTOR DIRECTION CONTROL
            # ============================================================

            # 1. Read joystick direction without temporal debounce.
            requested_direction = (
                direction if direction in ('FORWARD', 'REVERSE')
                else None
            )


            # 2. Begin a direction-change sequence whenever necessary.
            if (
                requested_direction is not None
                and requested_direction != selected_direction
                and direction_state in ('READY', 'WAIT_REQUEST')
            ):
                reset_motor_ramp()
                direction_state = 'PWM_OFF'
                direction_state_since = loop_start


            # 3. Direction-change state machine.
            if direction_state == 'PWM_OFF':

                reset_motor_ramp()

                # Do not switch direction while wheels are moving.
                wheels_stopped = all(
                    abs(rpm) <= DIRECTION_MAX_SWITCH_RPM
                    for rpm in hall_rpm
                )

                pwm_off_long_enough = (
                    ticks_diff(
                        loop_start, direction_state_since
                    ) >= DIRECTION_PWM_OFF_MS
                )

                if requested_direction is None:
                    if selected_direction is None:
                        direction_state = 'WAIT_REQUEST'
                    else:
                        direction_state = 'READY'

                elif wheels_stopped and pwm_off_long_enough:

                    select_motor_direction(requested_direction)

                    selected_direction = requested_direction

                    direction_state = 'SETTLING'
                    direction_state_since = loop_start

                    log(f'Motor direction selected: {selected_direction}')


            elif direction_state == 'SETTLING':

                reset_motor_ramp()

                # If the joystick requests the opposite direction,
                # restart the safe switching sequence.
                if (
                    requested_direction is not None
                    and requested_direction != selected_direction
                ):
                    direction_state = 'PWM_OFF'
                    direction_state_since = loop_start

                elif ticks_diff(
                    loop_start, direction_state_since
                ) >= DIRECTION_SETTLE_MS:

                    direction_state = 'READY'


            elif direction_state == 'WAIT_REQUEST':

                reset_motor_ramp()


            # 4. Enable traction only when direction selection is complete.
            drive_allowed = (
                direction_state == 'READY'
                and not brakes_on
                and direction != 'NEUTRAL'
                and direction == selected_direction
                and requested_direction == selected_direction
            )

            rolling = SPEED_KMH > STARTUP_SPEED_KMH
            rolling_neutral = (
                joystick_neutral and rolling and selected_direction == 'FORWARD'
                and direction_state == 'READY' and not brakes_on
            )
            drive_speed = speed
            if drive_allowed:
                if imu_is_healthy():
                    drive_speed = get_uphill_gear(
                        mode, speed, direction, UPHILL_PITCH_SIGN * get_pitch_degrees()
                    )
                target = joystick_to_motor_voltage(y, mode, drive_speed, direction)
                update_motor_voltage(
                    target, mode, drive_speed, direction,
                    slow_start=(direction == 'FORWARD' and not rolling),
                )
            elif rolling_neutral:
                update_motor_voltage(0.0, mode, speed, selected_direction, slow_start=False)
            else:
                reset_motor_ramp()

            # Final brake interlock after the motor update.
            if read_reed_switch() == 0 or handbrake_active_since is not None:
                reset_motor_ramp()

            if mode != previous_mode:
                log(f'Mode: {mode}')
                previous_mode = mode
            if speed != previous_speed:
                log(f'Speed setting: {speed}')
                previous_speed = speed

            now = ticks_ms()
            if ticks_diff(now, last_display_ms) >= DISPLAY_INTERVAL_MS:
                update_display_state(state, now, mode, speed, y)
                draw_screen(_oled, state)
                last_display_ms = now

            if ticks_diff(now, last_log_ms) >= DIAGNOSTIC_INTERVAL_MS:
                log(
                    'Mode:{} Speed:{} | v={:.2f}km/h | pitch={} | '
                    'brake:{} | direction:{} | state:{} | drive gear:{}'.format(
                        mode,
                        speed,
                        SPEED_KMH,
                        f'{get_pitch_degrees():+.1f}'
                        if imu_is_healthy() else 'N/A',
                        'ON' if brakes_on else 'OFF',
                        selected_direction,
                        direction_state,
                        drive_speed
                    )
                )
                last_log_ms = now

            remaining = LOOP_PERIOD_MS - ticks_diff(ticks_ms(), loop_start)
            if remaining > 0:
                sleep_ms(remaining)
    finally:
        stop_outputs()
        if _oled is not None:
            try:
                _oled.fill(0)
                _oled.show()
            except Exception:  # noqa: BLE001, S110
                pass


if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        stop_outputs()
