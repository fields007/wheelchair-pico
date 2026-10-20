"""TEMPORARY IMU calibration-only replacement for chair_logic.py.

NO motor, steering, reverse, brake, or handbrake GPIO is initialized.
Save as chair_logic.py temporarily; restore original for driving.
Place chair level and motionless during initial 4-second calibration.
Uses SSD1306 at 0x3C and IMU at 0x68 on I2C0 GP20/GP21.
"""
from machine import Pin, I2C
from time import ticks_ms, ticks_diff, sleep_ms
from math import sqrt, atan2, pi

I2C_ADDRESS = 0x68
G = 9.80665
ACCEL_SCALE = G / 16384.0  # configured +/-2g
GYRO_SCALE = 1.0 / 131.0  # configured +/-250 deg/s
SAMPLES = 100
SAMPLE_DELAY_MS = 30
READ_INTERVAL_MS = 100
RECONNECT_MS = 1000

bus = None
oled = None
reference = None
basis = None
last_error = None
successful_reads = 0
failed_reads = 0


def signed16(a, b):
    v = (a << 8) | b
    return v - 65536 if v & 0x8000 else v


def norm(v):
    return sqrt(sum(x*x for x in v))


def unit(v):
    m = norm(v)
    if m < 1e-6:
        raise ValueError('zero acceleration vector')
    return tuple(x/m for x in v)


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def level_basis(a):
    # Level Z aligns with measured upward specific force.
    z = unit(a)
    # Preserve as much of the sensor X heading as possible.
    raw_x = (1-z[0]*z[0], -z[0]*z[1], -z[0]*z[2])
    if norm(raw_x) < 0.1:
        # If sensor X is nearly vertical, project Y instead.
        raw_y = (-z[1]*z[0], 1-z[1]*z[1], -z[1]*z[2])
        y = unit(raw_y)
        x = unit(cross(y, z))
    else:
        x = unit(raw_x)
    y = cross(z, x)
    return (x, y, z)


def rotate(v):
    return tuple(dot(v, axis) for axis in basis)


def setup_bus():
    global bus, oled
    bus = I2C(0, sda=Pin(20), scl=Pin(21), freq=100000)
    devices = bus.scan()
    print('I2C scan:', [hex(d) for d in devices])
    if 0x3C in devices:
        try:
            import ssd1306
            oled = ssd1306.SSD1306_I2C(128, 64, bus, addr=0x3C)
        except Exception as exc:
            print('OLED initialization error:', repr(exc))
            oled = None
    if I2C_ADDRESS not in devices:
        raise OSError('IMU 0x68 not found')
    who = bus.readfrom_mem(I2C_ADDRESS, 0x75, 1)[0]
    print('WHO_AM_I: 0x{:02X}'.format(who))
    if who not in (0x68, 0x70, 0x71, 0x73):
        raise ValueError('Unexpected WHO_AM_I: 0x{:02X}'.format(who))
    bus.writeto_mem(I2C_ADDRESS, 0x6B, b'\x01')
    bus.writeto_mem(I2C_ADDRESS, 0x1B, b'\x00')
    bus.writeto_mem(I2C_ADDRESS, 0x1C, b'\x00')
    bus.writeto_mem(I2C_ADDRESS, 0x1A, b'\x03')
    sleep_ms(100)


def read_sensor():
    global successful_reads, failed_reads, last_error
    try:
        raw = bus.readfrom_mem(I2C_ADDRESS, 0x3B, 14)
        if len(raw) != 14:
            raise OSError('short IMU read')
        a = tuple(signed16(raw[i], raw[i+1])*ACCEL_SCALE for i in (0, 2, 4))
        w = tuple(signed16(raw[i], raw[i+1])*GYRO_SCALE for i in (8, 10, 12))
        successful_reads += 1
        last_error = None
        return a, w
    except Exception as exc:
        failed_reads += 1
        last_error = repr(exc)
        raise


def screen(lines):
    if oled is None:
        return
    try:
        oled.fill(0)
        for row, line in enumerate(lines[:6]):
            oled.text(str(line)[:16], 0, row*10)
        oled.show()
    except Exception as exc:
        print('OLED error:', repr(exc))


def calibrate(log=print):
    global reference, basis
    reference = None
    basis = None
    log('Keep wheelchair LEVEL and STILL for calibration.')
    screen(['IMU CALIBRATION', 'Keep chair LEVEL', 'and STILL', 'Sampling...'])
    sums = [0.0, 0.0, 0.0]
    good = 0
    attempts = 0
    while good < SAMPLES and attempts < SAMPLES*3:
        attempts += 1
        try:
            a, _ = read_sensor()
            if 7.0 <= norm(a) <= 12.0:
                for i in range(3):
                    sums[i] += a[i]
                good += 1
        except Exception as exc:
            if attempts % 10 == 1:
                log('Calibration read error: {}'.format(repr(exc)))
        if good % 20 == 0 and good > 0:
            screen(['IMU CALIBRATION', 'Keep chair STILL', '{} / {}'.format(good, SAMPLES)])
        sleep_ms(SAMPLE_DELAY_MS)
    if good < SAMPLES:
        raise RuntimeError('Only {} valid samples out of {} attempts'.format(good, attempts))
    reference = tuple(v/good for v in sums)
    basis = level_basis(reference)
    log('IMU_LEVEL_ACCEL = ({:.5f}, {:.5f}, {:.5f})'.format(*reference))
    log('Reference magnitude: {:.3f} m/s2 (expected ~9.81)'.format(norm(reference)))
    log('LEVEL_X = {}'.format(basis[0]))
    log('LEVEL_Y = {}'.format(basis[1]))
    log('LEVEL_Z = {}'.format(basis[2]))
    log('Calibration successful. These constants are NOT saved automatically.')


def run(stop_requested=None, log=print):
    global last_error
    log('IMU-ONLY MODE: traction and steering are NOT controlled by this file.')
    next_reconnect = 0
    connected = False
    last_report = ticks_ms()
    while True:
        if stop_requested is not None and stop_requested():
            log('IMU calibration stopped.')
            return
        now = ticks_ms()
        if not connected:
            if ticks_diff(now, next_reconnect) < 0:
                sleep_ms(100)
                continue
            next_reconnect = now + RECONNECT_MS
            try:
                setup_bus()
                calibrate(log)
                connected = True
            except Exception as exc:
                last_error = repr(exc)
                log('IMU SETUP/CALIBRATION ERROR: {}'.format(last_error))
                screen(['IMU: ERROR', str(exc)[:16], 'Retrying...'])
                sleep_ms(200)
                continue
        try:
            a_raw, w_raw = read_sensor()
            a = rotate(a_raw)
            w = rotate(w_raw)
            pitch = atan2(-a[0], sqrt(a[1]*a[1]+a[2]*a[2]))*180/pi
            roll = atan2(a[1], a[2])*180/pi
            magnitude = norm(a_raw)
            # Gravity-only angles; during motion these include linear acceleration.
            screen(['IMU: OK', 'Pitch:{:+.1f} deg'.format(pitch),
                    'Roll: {:+.1f} deg'.format(roll),
                    'Ax:{:+.2f}'.format(a[0]), 'Ay:{:+.2f}'.format(a[1]),
                    'Az:{:+.2f} |g|:{:.1f}'.format(a[2], magnitude)])
            if ticks_diff(ticks_ms(), last_report) >= 1000:
                log('IMU:OK | RAW a=({:+.3f},{:+.3f},{:+.3f}) | LEVEL a=({:+.3f},{:+.3f},{:+.3f}) m/s2 | |a|={:.3f} | Pitch={:+.2f} Roll={:+.2f} deg | Gyro=({:+.2f},{:+.2f},{:+.2f}) deg/s | reads OK={} failed={}'.format(
                    *(a_raw + a), magnitude, pitch, roll, *w, successful_reads, failed_reads))
                last_report = ticks_ms()
        except Exception as exc:
            connected = False
            log('IMU READ ERROR: {} | successes={} failures={}'.format(repr(exc), successful_reads, failed_reads))
            screen(['IMU: READ ERROR', repr(exc)[:16], 'Retrying...'])
        sleep_ms(READ_INTERVAL_MS)


if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print('IMU test stopped.')
