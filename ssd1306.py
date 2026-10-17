"""
SSD1306 OLED driver for MicroPython.

Designed for Raspberry Pi Pico / Pico 2 W.

Features:
- I2C communication
- Chunked framebuffer transmission
- Delayed initialization for reliable cold boot
- Explicit horizontal addressing
- I2C write retries
- Compatible with existing supervisor code
"""

from micropython import const
from time import sleep_ms
import framebuf


# ============================================================
# SSD1306 COMMANDS
# ============================================================

SET_CONTRAST = const(0x81)
SET_ENTIRE_ON = const(0xA4)
SET_NORM_INV = const(0xA6)
SET_DISP = const(0xAE)
SET_MEM_ADDR = const(0x20)
SET_COL_ADDR = const(0x21)
SET_PAGE_ADDR = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP = const(0xA0)
SET_MUX_RATIO = const(0xA8)
SET_COM_OUT_DIR = const(0xC0)
SET_DISP_OFFSET = const(0xD3)
SET_COM_PIN_CFG = const(0xDA)
SET_DISP_CLK_DIV = const(0xD5)
SET_PRECHARGE = const(0xD9)
SET_VCOM_DESEL = const(0xDB)
SET_CHARGE_PUMP = const(0x8D)


# ============================================================
# SETTINGS
# ============================================================

# Allow the OLED power supply to stabilize.
OLED_STARTUP_DELAY_MS = 500

# Delay after enabling the charge pump.
OLED_CHARGE_PUMP_DELAY_MS = 100

# Delay before first framebuffer transmission.
OLED_FIRST_FRAME_DELAY_MS = 50

# Number of attempts for individual I2C writes.
I2C_WRITE_ATTEMPTS = 3

# Delay between failed I2C writes.
I2C_RETRY_DELAY_MS = 10


# ============================================================
# BASE SSD1306 CLASS
# ============================================================

class SSD1306:

    def __init__(
        self,
        width,
        height,
        external_vcc=False
    ):

        self.width = width
        self.height = height
        self.external_vcc = external_vcc

        self.pages = height // 8

        self.buffer = bytearray(
            self.pages * self.width
        )

        self.framebuf = framebuf.FrameBuffer(
            self.buffer,
            self.width,
            self.height,
            framebuf.MONO_VLSB
        )

        # Give the OLED time to power up.
        sleep_ms(OLED_STARTUP_DELAY_MS)

        self.init_display()


    # --------------------------------------------------------
    # INITIALIZATION
    # --------------------------------------------------------

    def init_display(self):

        # Turn display off.
        self.write_cmd(
            SET_DISP | 0x00
        )

        # Explicit horizontal addressing mode.
        self.write_cmd(SET_MEM_ADDR)
        self.write_cmd(0x00)

        # Display start line.
        self.write_cmd(
            SET_DISP_START_LINE | 0x00
        )

        # Segment remapping.
        self.write_cmd(
            SET_SEG_REMAP | 0x00
        )

        # Multiplex ratio.
        self.write_cmd(SET_MUX_RATIO)
        self.write_cmd(self.height - 1)

        # COM output scan direction.
        self.write_cmd(
            SET_COM_OUT_DIR | 0x00
        )

        # Display offset.
        self.write_cmd(SET_DISP_OFFSET)
        self.write_cmd(0x00)

        # COM pin configuration.
        self.write_cmd(SET_COM_PIN_CFG)

        self.write_cmd(
            0x02 if self.height == 32 else 0x12
        )

        # Display clock.
        self.write_cmd(SET_DISP_CLK_DIV)
        self.write_cmd(0x80)

        # Precharge.
        self.write_cmd(SET_PRECHARGE)

        self.write_cmd(
            0x22 if self.external_vcc else 0xF1
        )

        # VCOM deselect.
        self.write_cmd(SET_VCOM_DESEL)
        self.write_cmd(0x30)

        # Contrast.
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(0xFF)

        # Entire display follows RAM.
        self.write_cmd(SET_ENTIRE_ON)

        # Normal display mode.
        self.write_cmd(SET_NORM_INV)

        # Enable charge pump.
        self.write_cmd(SET_CHARGE_PUMP)

        self.write_cmd(
            0x10 if self.external_vcc else 0x14
        )

        # Wait for charge pump stabilization.
        sleep_ms(OLED_CHARGE_PUMP_DELAY_MS)

        # Turn display on.
        self.write_cmd(
            SET_DISP | 0x01
        )

        # Clear framebuffer.
        self.fill(0)

        sleep_ms(OLED_FIRST_FRAME_DELAY_MS)

        # Clear physical display.
        self.show()


    # --------------------------------------------------------
    # DISPLAY CONTROL
    # --------------------------------------------------------

    def poweroff(self):

        self.write_cmd(
            SET_DISP | 0x00
        )


    def poweron(self):

        self.write_cmd(
            SET_DISP | 0x01
        )


    def contrast(self, contrast):

        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)


    def invert(self, invert):

        self.write_cmd(
            SET_NORM_INV | (invert & 1)
        )


    # --------------------------------------------------------
    # DRAWING
    # --------------------------------------------------------

    def fill(self, color):

        self.framebuf.fill(color)


    def pixel(self, x, y, color=None):

        if color is None:

            return self.framebuf.pixel(x, y)

        self.framebuf.pixel(
            x,
            y,
            color
        )


    def text(self, string, x, y, color=1):

        self.framebuf.text(
            string,
            x,
            y,
            color
        )


    def hline(self, x, y, width, color=1):

        self.framebuf.hline(
            x,
            y,
            width,
            color
        )


    def vline(self, x, y, height, color=1):

        self.framebuf.vline(
            x,
            y,
            height,
            color
        )


    def line(
        self,
        x1,
        y1,
        x2,
        y2,
        color=1
    ):

        self.framebuf.line(
            x1,
            y1,
            x2,
            y2,
            color
        )


    def rect(
        self,
        x,
        y,
        width,
        height,
        color=1
    ):

        self.framebuf.rect(
            x,
            y,
            width,
            height,
            color
        )


    def fill_rect(
        self,
        x,
        y,
        width,
        height,
        color=1
    ):

        self.framebuf.fill_rect(
            x,
            y,
            width,
            height,
            color
        )


    def scroll(self, dx, dy):

        self.framebuf.scroll(
            dx,
            dy
        )


    def blit(
        self,
        framebuffer,
        x,
        y,
        key=-1
    ):

        self.framebuf.blit(
            framebuffer,
            x,
            y,
            key
        )


# ============================================================
# I2C SSD1306 CLASS
# ============================================================

class SSD1306_I2C(SSD1306):

    def __init__(
        self,
        width,
        height,
        i2c,
        addr=0x3C,
        external_vcc=False,
        chunk_size=16
    ):

        self.i2c = i2c
        self.addr = addr

        self.chunk_size = max(
            1,
            int(chunk_size)
        )

        super().__init__(
            width,
            height,
            external_vcc
        )


    # --------------------------------------------------------
    # RELIABLE I2C WRITE
    # --------------------------------------------------------

    def _write(self, data):

        """
        Attempt an I2C transmission up to three times.

        If all attempts fail, raise the last exception.
        """

        for attempt in range(
            I2C_WRITE_ATTEMPTS
        ):

            try:

                self.i2c.writeto(
                    self.addr,
                    data
                )

                return

            except OSError:

                if (
                    attempt
                    == I2C_WRITE_ATTEMPTS - 1
                ):

                    raise

                sleep_ms(
                    I2C_RETRY_DELAY_MS
                )


    # --------------------------------------------------------
    # SEND COMMAND
    # --------------------------------------------------------

    def write_cmd(self, cmd):

        self._write(
            bytes([0x80, cmd])
        )


    # --------------------------------------------------------
    # UPDATE DISPLAY
    # --------------------------------------------------------

    def show(self):

        """
        Transfer framebuffer to OLED.

        Uses horizontal addressing mode.

        Data is transmitted in small chunks to
        reduce I2C transaction size.
        """

        # Explicitly select horizontal addressing.
        self.write_cmd(SET_MEM_ADDR)
        self.write_cmd(0x00)

        # Set column range.
        self.write_cmd(SET_COL_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.width - 1)

        # Set page range.
        self.write_cmd(SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)

        # Send framebuffer.
        for start in range(
            0,
            len(self.buffer),
            self.chunk_size
        ):

            chunk = self.buffer[
                start:start + self.chunk_size
            ]

            self._write(
                b'\x40' + chunk
            )