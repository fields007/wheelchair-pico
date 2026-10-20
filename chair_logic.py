"""Wheelchair OLED DESIGN MOCK for Pico 2 W, SSD1306 128x64.

Display only: NO traction/steering/brake actuator GPIO is configured.
Run directly in Thonny with ssd1306.py on the Pico, or temporarily use
as chair_logic.py with existing supervisor calling run(stop_requested,log).
All numbers are fictional and change automatically every few seconds.

Layout uses x=8..119 and y=7..56 to avoid rounded/cut-off corners.
The functional sections below are intentionally empty placeholders.
"""
from machine import Pin, I2C
from time import sleep_ms, ticks_ms, ticks_diff
import framebuf
import ssd1306

# ============================================================
# 1. HARDWARE MAP AND CALIBRATED INPUT VALUES (NOT USED IN DEMO)
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

X_LEFT, X_CENTER, X_RIGHT, X_DEADZONE = 0, 33815, 65351, 2941
Y_BACKWARD, Y_CENTER, Y_FORWARD, Y_DEADZONE = 5401, 43529, 65340, 599
IMU_I2C_ADDRESS = 0x68
OLED_I2C_ADDRESS = 0x3C

# ============================================================
# 2. FUTURE IMU LOGIC -- paste stable calibration/filter here
# ============================================================
# Interface idea: get_pitch_degrees() -> float
# Stationary-only outlier rejection must NOT be reused as driving safety logic.

# ============================================================
# 3. FUTURE THROTTLE LOGIC -- joystick, ramps, motor PWM
# ============================================================
# Interface idea: get_throttle_request() -> float (0..1)
# Motor enable must be guarded by the actual brake input and failsafe logic.

# ============================================================
# 4. FUTURE STEERING LOGIC -- joystick and steering servo
# ============================================================

# ============================================================
# 5. FUTURE BRAKE LOGIC -- reed interlock + handbrake servo
# ============================================================
# Actual braking interlock must disable traction independently of the OLED.
# Define brake_on and throttle_requested separately for display warning.

# ============================================================
# 6. FUTURE WHEEL SPEED LOGIC -- four Hall sensors
# ============================================================
# 8 magnets/revolution, 16 transitions/revolution; wheel radius ~0.225 m.

# ============================================================
# 7. DISPLAY STATE -- future logic should update these fields
# ============================================================
class DisplayState:
    def __init__(self):
        self.pitch_deg = 0.0
        self.mode = 'TERRAIN'     # TERRAIN / DIFFERENTIAL / ROAD
        self.speed_setting = 'LOW'  # LOW / MED / HIGH
        self.speed_kmh = 0.0
        self.throttle_requested = False
        self.brake_on = False
        self.mode_changed = False
        self.speed_changed = False

# ============================================================
# 8. OLED FONT AND DRAWING -- display-only functions
# ============================================================
WIDTH, HEIGHT = 128, 64
SAFE_LEFT, SAFE_RIGHT = 8, 119
SAFE_TOP, SAFE_BOTTOM = 7, 56
DISPLAY_INTERVAL_MS = 100

# Use built-in 8x8 framebuf font; scale pixels in software, no extra fonts.
_glyph = bytearray(8)
_glyph_fb = framebuf.FrameBuffer(_glyph, 8, 8, framebuf.MONO_HLSB)


def draw_text(oled, value, x, y, scale=1):
    value = str(value)
    for index, ch in enumerate(value):
        _glyph_fb.fill(0)
        _glyph_fb.text(ch, 0, 0, 1)
        for py in range(8):
            for px in range(8):
                if _glyph_fb.pixel(px, py):
                    oled.fill_rect(x + index*8*scale + px*scale,
                                   y + py*scale, scale, scale, 1)


def centered(oled, value, y, scale=1):
    value = str(value)
    width = len(value) * 8 * scale
    x = (WIDTH - width) // 2
    if x < SAFE_LEFT or x + width - 1 > SAFE_RIGHT:
        # Shrink rather than draw into the clipped corners.
        scale = 1
        width = len(value) * 8
        x = (WIDTH - width) // 2
    draw_text(oled, value, x, y, scale)


def mode_short(mode):
    return {'TERRAIN': 'TERRAIN', 'DIFFERENTIAL': 'DIFF',
            'ROAD': 'ROAD'}.get(mode, str(mode)[:10])


def draw_idle(oled, state):
    centered(oled, 'SLOPE', 8, 1)
    # 16 px tall; enough room for a signed pitch angle.
    pitch = '{:+.1f}'.format(state.pitch_deg)
    centered(oled, pitch, 22, 2)
    centered(oled, 'deg', 40, 1)
    centered(oled, '{}  {}'.format(mode_short(state.mode),
                                   state.speed_setting), 49, 1)


def draw_driving(oled, state):
    centered(oled, 'SPEED', 8, 1)
    # e.g. '12.4' at 2x => 64 px wide; legible without corner clipping.
    centered(oled, '{:.1f}'.format(state.speed_kmh), 20, 2)
    centered(oled, 'km/h', 39, 1)
    centered(oled, '{}  {}'.format(mode_short(state.mode),
                                   state.speed_setting), 49, 1)


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
        draw_change(oled, 'SETTING', state.speed_setting)
    elif state.throttle_requested:
        draw_driving(oled, state)
    else:
        draw_idle(oled, state)
    oled.show()

# ============================================================
# 9. MOCK SCENES -- replace this with real sensor/logic state later
# ============================================================
# (duration in seconds, pitch, mode, setting, km/h, throttle, brake,
#  mode-change, speed-change)
MOCK_SCENES = (
    (5,  0.0, 'TERRAIN', 'LOW',  0.0, False, False, False, False),
    (5, +8.4, 'TERRAIN', 'LOW',  0.0, False, False, False, False),
    (5, +8.4, 'TERRAIN', 'LOW',  7.2, True,  False, False, False),
    (3, +8.4, 'ROAD',    'LOW',  7.2, True,  False, True,  False),
    (5, +8.4, 'ROAD',    'LOW', 12.4, True,  False, False, False),
    (3, +8.4, 'ROAD',    'HIGH',12.4, True,  False, False, True),
    (5, +8.4, 'ROAD',    'HIGH',18.6, True,  False, False, False),
    (5, +8.4, 'ROAD',    'HIGH', 0.0, True,  True,  False, False),
    (5, -3.2, 'ROAD',    'HIGH', 0.0, False, True,  False, False),
    (3, -3.2, 'DIFFERENTIAL', 'MED', 0.0, False, False, True, False),
    (5, -3.2, 'DIFFERENTIAL', 'MED', 0.0, False, False, False, False),
)


def mock_state(elapsed_ms):
    cycle_ms = sum(scene[0] for scene in MOCK_SCENES) * 1000
    t = elapsed_ms % cycle_ms
    chosen = MOCK_SCENES[-1]
    for scene in MOCK_SCENES:
        duration_ms = scene[0] * 1000
        if t < duration_ms:
            chosen = scene
            break
        t -= duration_ms
    s = DisplayState()
    (_, s.pitch_deg, s.mode, s.speed_setting, s.speed_kmh,
     s.throttle_requested, s.brake_on, s.mode_changed,
     s.speed_changed) = chosen
    return s

# ============================================================
# 10. SUPERVISOR COMPATIBILITY -- stop_requested/log signatures
# ============================================================
_i2c = None
_oled = None


def stop_outputs():
    """No actuator outputs are created by this display-only demo."""
    pass


def run(stop_requested=None, log=print):
    global _i2c, _oled
    # Display-only: initialize GP20/21, no other pins.
    _i2c = I2C(0, sda=Pin(PIN_I2C_SDA), scl=Pin(PIN_I2C_SCL), freq=50000)
    _oled = ssd1306.SSD1306_I2C(WIDTH, HEIGHT, _i2c, addr=OLED_I2C_ADDRESS)
    start = ticks_ms()
    last_frame = start - DISPLAY_INTERVAL_MS
    log('OLED design mock started; motors and servos NOT initialized')
    try:
        while True:
            if stop_requested is not None and stop_requested():
                break
            now = ticks_ms()
            if ticks_diff(now, last_frame) >= DISPLAY_INTERVAL_MS:
                state = mock_state(ticks_diff(now, start))
                draw_screen(_oled, state)
                last_frame = now
            sleep_ms(20)
    finally:
        _oled.fill(0)
        _oled.show()
        stop_outputs()


if __name__ == '__main__':
    run()
