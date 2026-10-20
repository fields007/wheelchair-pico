"""Stationary IMU + brake diagnostic ONLY for Pico 2 W.

Save as imu_stable.py and run directly in Thonny. Does NOT drive motors,
steering, reverse or handbrake. Do not replace the production chair_logic.py
while using the wheelchair.

IMU: I2C0 SDA GP20 SCL GP21, address 0x68.
OLED: same bus, address 0x3C, requires ssd1306.py on Pico.
Brake reed: GP1, active LOW (LOW means BRAKE ON).

Place wheelchair LEVEL and STILL when starting. Calibration is deliberately
retained across transient I2C errors. Display shows filtered accelerometer-
derived tilt; it is NOT a safety-rated tilt sensor.
"""
from machine import Pin, I2C
from time import sleep_ms, ticks_ms, ticks_diff
from math import sqrt, atan2, pi

IMU_ADDR = 0x68
OLED_ADDR = 0x3C
G = 9.80665
ACC_SCALE = G / 16384.0
CAL_SAMPLES = 80
CAL_MAX_ATTEMPTS = 600
CAL_MIN_G = 8.5
CAL_MAX_G = 11.1
FILTER_ALPHA = 0.06  # ~1.5 s smoothing with 100 ms sampling
ZERO_ANGLE_DEG = 0.25  # cosmetic display rounding only
ZERO_XY_MS2 = 0.08     # cosmetic display rounding only
READ_PERIOD_MS = 100
DISPLAY_PERIOD_MS = 400
LOG_PERIOD_MS = 1000

brake = Pin(1, Pin.IN, Pin.PULL_UP)
i2c = I2C(0, sda=Pin(20), scl=Pin(21), freq=50000)
oled = None
basis = None
filtered = None
successes = 0
failures = 0
last_error = ''


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


def rotate(v):
    return tuple(dot(v, axis) for axis in basis)


def read_accel():
    global successes, failures, last_error
    try:
        raw = i2c.readfrom_mem(IMU_ADDR, 0x3B, 6)
        if len(raw) != 6:
            raise OSError('Short accelerometer read')
        a = tuple(signed16(raw[i], raw[i+1])*ACC_SCALE for i in (0, 2, 4))
        successes += 1
        return a
    except Exception as exc:
        failures += 1
        last_error = repr(exc)
        raise


def setup_imu():
    devices = i2c.scan()
    print('I2C devices:', [hex(x) for x in devices])
    if IMU_ADDR not in devices:
        raise OSError('IMU at 0x68 not found')
    who = i2c.readfrom_mem(IMU_ADDR, 0x75, 1)[0]
    print('WHO_AM_I: 0x{:02X}'.format(who))
    i2c.writeto_mem(IMU_ADDR, 0x6B, b'\x01')
    sleep_ms(100)
    i2c.writeto_mem(IMU_ADDR, 0x1C, b'\x00')  # +/-2g
    i2c.writeto_mem(IMU_ADDR, 0x1B, b'\x00')  # +/-250 deg/s
    i2c.writeto_mem(IMU_ADDR, 0x1A, b'\x03')
    sleep_ms(100)


def setup_oled():
    global oled
    try:
        if OLED_ADDR in i2c.scan():
            import ssd1306
            oled = ssd1306.SSD1306_I2C(128, 64, i2c, addr=OLED_ADDR)
            print('OLED connected')
        else:
            print('OLED not found; serial output only')
    except Exception as exc:
        print('OLED setup error:', repr(exc))
        oled = None


def display(lines):
    if oled is None:
        return
    try:
        oled.fill(0)
        for i, line in enumerate(lines[:6]):
            oled.text(str(line)[:16], 0, i*10)
        oled.show()
    except Exception as exc:
        print('OLED error:', repr(exc))


def calibrate():
    global basis, filtered
    print('Place wheelchair LEVEL and STILL; calibrating...')
    display(['IMU calibration', 'Chair LEVEL', 'Do not move'])
    sums = [0.0, 0.0, 0.0]
    good = 0
    for attempt in range(CAL_MAX_ATTEMPTS):
        try:
            a = read_accel()
            if CAL_MIN_G <= magnitude(a) <= CAL_MAX_G:
                for i in range(3):
                    sums[i] += a[i]
                good += 1
                if good >= CAL_SAMPLES:
                    break
        except Exception:
            pass
        sleep_ms(30)
    if good < CAL_SAMPLES:
        raise RuntimeError('Calibration: only {} valid readings'.format(good))
    reference = tuple(s/good for s in sums)
    basis = create_basis(reference)
    filtered = rotate(reference)
    print('CALIBRATED reference =', reference)
    print('Reference magnitude = {:.3f} m/s2'.format(magnitude(reference)))
    print('Level axes X/Y/Z =', basis)


def show_zero(v, tolerance):
    return 0.0 if abs(v) < tolerance else v


def run(stop_requested=None, log=print):
    global filtered, last_error
    log('IMU + brake DIAGNOSTIC ONLY; NO motor or servo control.')
    setup_oled()
    # Initial configuration and calibration are retried; no output is enabled.
    while basis is None:
        if stop_requested is not None and stop_requested():
            return
        try:
            setup_imu()
            calibrate()
        except Exception as exc:
            log('Setup/calibration error: {}'.format(repr(exc)))
            display(['IMU setup ERROR', 'Check wiring', 'Retrying...',
                     'Brake:{}'.format('ON' if brake.value()==0 else 'OFF')])
            sleep_ms(1500)
    last_display = last_log = ticks_ms()
    while True:
        if stop_requested is not None and stop_requested():
            return
        now = ticks_ms()
        valid = False
        try:
            raw = read_accel()
            n = magnitude(raw)
            if CAL_MIN_G <= n <= CAL_MAX_G:
                leveled = rotate(raw)
                filtered = tuple((1-FILTER_ALPHA)*filtered[i] + FILTER_ALPHA*leveled[i]
                                 for i in range(3))
                valid = True
            else:
                last_error = 'Rejected |a|={:.2f}'.format(n)
        except Exception:
            pass
        # Never replace calibration or the filtered value with invalid data.
        ax, ay, az = filtered
        pitch = atan2(-ax, sqrt(ay*ay + az*az))*180/pi
        roll = atan2(ay, az)*180/pi
        pitch_display = show_zero(pitch, ZERO_ANGLE_DEG)
        roll_display = show_zero(roll, ZERO_ANGLE_DEG)
        ax_display = show_zero(ax, ZERO_XY_MS2)
        ay_display = show_zero(ay, ZERO_XY_MS2)
        brake_text = 'ON' if brake.value() == 0 else 'OFF'
        if ticks_diff(now, last_display) >= DISPLAY_PERIOD_MS:
            display(['IMU:{}'.format('OK' if valid else 'ERROR'),
                     'Pitch:{:+.1f}deg'.format(pitch_display),
                     'Roll: {:+.1f}deg'.format(roll_display),
                     'Ax:{:+.2f} Ay:{:+.2f}'.format(ax_display, ay_display),
                     'Az:{:+.2f} m/s2'.format(az),
                     'Brake:{}'.format(brake_text)])
            last_display = now
        if ticks_diff(now, last_log) >= LOG_PERIOD_MS:
            log('IMU:{} | LEVEL filtered a=({:+.3f},{:+.3f},{:+.3f}) m/s2 | '
                'Pitch={:+.2f} Roll={:+.2f} deg | Brake:{} | reads OK={} failed={} | {}'.format(
                    'OK' if valid else 'ERROR', ax, ay, az, pitch, roll,
                    brake_text, successes, failures, last_error))
            last_log = now
        sleep_ms(READ_PERIOD_MS)


if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print('IMU diagnostic stopped')
