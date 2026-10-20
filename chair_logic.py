"""TEMPORARY joystick calibration replacement for chair_logic.py.

No PWM outputs are initialized or commanded. Keep the chair immobilized,
traction power disconnected, and nobody seated during calibration.

Compatible with supervisor calls run(stop_requested=..., log=...).
Starts only when Wi-Fi station reports connected; that does NOT prove MQTT
is connected or that the supervisor's log callback forwards to MQTT.
The complete report is passed to log() once, and also printed locally.
"""
from machine import ADC, Pin, I2C
from time import ticks_ms, ticks_diff, sleep_ms
from math import sqrt
try:
    import network
except ImportError:
    network = None

X = ADC(27)
Y = ADC(28)
BRAKE = Pin(1, Pin.IN, Pin.PULL_UP)
WAIT_MS = 5000
MEASURE_MS = 5000
SAMPLE_MS = 20
OLED_ADDR = 0x3C
OLED = None
try:
    import ssd1306
    bus = I2C(0, sda=Pin(20), scl=Pin(21), freq=50000)
    if OLED_ADDR in bus.scan():
        OLED = ssd1306.SSD1306_I2C(128, 64, bus, addr=OLED_ADDR)
except Exception as exc:
    print('OLED unavailable:', repr(exc))

# Sampling instructions: joystick should be held at the requested limit
# for the entire 5-second measurement interval.
STEPS = (
    ('CENTER', 'Release joystick'),
    ('LEFT', 'Hold full LEFT'),
    ('RIGHT', 'Hold full RIGHT'),
    ('FORWARD', 'Hold full FORWARD'),
    ('BACKWARD', 'Hold full BACK'),
)


def stop_outputs():
    """No motor/servo PWM exists in this diagnostic module."""
    return


def online():
    try:
        return network is not None and network.WLAN(network.STA_IF).isconnected()
    except Exception:
        return False


def display(*lines):
    if OLED is None:
        return
    try:
        OLED.fill(0)
        for i, line in enumerate(lines[:6]):
            OLED.text(str(line)[:16], 0, i * 10)
        OLED.show()
    except Exception as exc:
        print('OLED error:', repr(exc))


def stats(values):
    n = len(values)
    if not n:
        return None
    avg = sum(values) / n
    var = sum((v - avg) ** 2 for v in values) / n
    s = sorted(values)
    return {'n': n, 'mean': avg, 'std': sqrt(var), 'min': s[0],
            'max': s[-1], 'p05': s[int((n - 1) * 0.05)],
            'median': s[n // 2], 'p95': s[int((n - 1) * 0.95)]}


def format_stats(label, data):
    return ('  {}: n={} mean={:.1f} std={:.1f} min={} p05={} '
            'median={} p95={} max={}').format(
        label, data['n'], data['mean'], data['std'], data['min'],
        data['p05'], data['median'], data['p95'], data['max'])


def wait_for_interval(ms, step_name, instruction, phase, stop_requested):
    start = ticks_ms()
    last_second = -1
    while ticks_diff(ticks_ms(), start) < ms:
        if stop_requested is not None and stop_requested():
            return False
        if not online():
            return False
        elapsed = ticks_diff(ticks_ms(), start)
        remaining = max(0, (ms - elapsed + 999) // 1000)
        if remaining != last_second:
            last_second = remaining
            display('JOYSTICK CAL', step_name, instruction,
                    '{}: {}s'.format(phase, remaining),
                    'Brake:{}'.format('ON' if BRAKE.value() == 0 else 'OFF'),
                    'WiFi:ONLINE')
            print('{}: {} - {} ({}s)'.format(phase, step_name, instruction, remaining))
        sleep_ms(40)
    return True


def measure(step_name, instruction, stop_requested):
    xs, ys = [], []
    start = ticks_ms()
    last_second = -1
    while ticks_diff(ticks_ms(), start) < MEASURE_MS:
        if stop_requested is not None and stop_requested():
            return None
        if not online():
            return None
        xs.append(X.read_u16())
        ys.append(Y.read_u16())
        elapsed = ticks_diff(ticks_ms(), start)
        remaining = max(0, (MEASURE_MS - elapsed + 999) // 1000)
        if remaining != last_second:
            last_second = remaining
            display('JOYSTICK CAL', step_name, instruction,
                    'MEASURE: {}s'.format(remaining),
                    'X:{} Y:{}'.format(xs[-1], ys[-1]),
                    'Brake:{}'.format('ON' if BRAKE.value() == 0 else 'OFF'))
        sleep_ms(SAMPLE_MS)
    return stats(xs), stats(ys)


def run(stop_requested=None, log=print):
    """Wait for Wi-Fi, then perform five 5s prepare + 5s measure steps.

    The supervisor must pass an MQTT-forwarding log callback if remote
    delivery is desired. Wi-Fi ONLINE alone does not establish MQTT status.
    """
    stop_outputs()
    print('Joystick calibration ready. No motor or servo outputs enabled.')
    display('JOYSTICK CAL', 'Waiting for WiFi', 'NO MOTOR OUTPUT')
    while not online():
        if stop_requested is not None and stop_requested():
            return
        sleep_ms(200)
    print('Wi-Fi connected. Beginning calibration.')
    results = {}
    for name, instruction in STEPS:
        if not wait_for_interval(WAIT_MS, name, instruction, 'PREPARE', stop_requested):
            print('Calibration aborted (stop request or Wi-Fi disconnected).')
            display('CAL ABORTED', 'No report sent')
            return
        sample = measure(name, instruction, stop_requested)
        if sample is None:
            print('Calibration aborted (stop request or Wi-Fi disconnected).')
            display('CAL ABORTED', 'No report sent')
            return
        results[name] = sample
        print('{} complete: X mean {:.1f}, Y mean {:.1f}'.format(
            name, sample[0]['mean'], sample[1]['mean']))

    lines = ['=== JOYSTICK CALIBRATION REPORT ===',
             'WiFi connected at report time: {}'.format(online()),
             'GP27 = X, GP28 = Y; ADC 0..65535',
             'Preparation: {}s; measurement: {}s per position'.format(
                 WAIT_MS // 1000, MEASURE_MS // 1000),
             'Brake GP1: {}'.format('ON' if BRAKE.value() == 0 else 'OFF')]
    for name, _ in STEPS:
        xstat, ystat = results[name]
        lines.append(name)
        lines.append(format_stats('X', xstat))
        lines.append(format_stats('Y', ystat))
    lines.extend([
        'SUGGESTED JOYSTICK CALIBRATION (rounded means):',
        'X_LEFT = {}'.format(round(results['LEFT'][0]['mean'])),
        'X_CENTER = {}'.format(round(results['CENTER'][0]['mean'])),
        'X_RIGHT = {}'.format(round(results['RIGHT'][0]['mean'])),
        'Y_BACKWARD = {}'.format(round(results['BACKWARD'][1]['mean'])),
        'Y_CENTER = {}'.format(round(results['CENTER'][1]['mean'])),
        'Y_FORWARD = {}'.format(round(results['FORWARD'][1]['mean'])),
        'NOTE: Choose deadzones after inspecting CENTER spread.',
        '=== END JOYSTICK CALIBRATION REPORT ==='])
    report = '\n'.join(lines)
    print(report)
    display('CAL COMPLETE', 'Report sent to', 'log callback', 'See console')
    # Single log callback invocation as requested. Some MQTT pipelines
    # limit message size; check actual receipt at broker/subscriber.
    try:
        log(report)
    except Exception as exc:
        print('LOG CALLBACK FAILED:', repr(exc))
        display('LOG SEND FAILED', 'See console')
    return


if __name__ == '__main__':
    run()
