import spidev
import numpy


class Freenove_SPI_LedPixel(object):
    # Initialize LED configuration, SPI bus, and clear strip.
    def __init__(self, count=8, bright=255, sequence='GRB', bus=0, device=0):
        self.set_led_type(sequence)
        self.set_led_count(count)
        self.set_led_brightness(bright)
        self.led_begin(bus, device)
        self.set_all_led_color(0, 0, 0)

    # Open SPI device and set mode. Marks init state.
    def led_begin(self, bus=0, device=0):
        self.bus = bus
        self.device = device
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.bus, self.device)
            self.spi.mode = 0
            self.led_init_state = 1
        except OSError:
            print("SPI init failed. Ensure SPI is enabled and overlays configured.")
            self.led_init_state = 0

    # Return 1 if SPI initialized, else 0.
    def check_spi_state(self):
        return self.led_init_state

    # Turn off all LEDs and close SPI device.
    def led_close(self):
        self.set_all_led_rgb([0, 0, 0])
        self.spi.close()

    # Set number of LEDs and allocate color buffers.
    def set_led_count(self, count):
        self.led_count = count
        self.led_color = [0, 0, 0] * self.led_count
        self.led_original_color = [0, 0, 0] * self.led_count

    # Get the configured number of LEDs.
    def get_led_count(self):
        return self.led_count

    # Set color component ordering (e.g., GRB) and compute offsets.
    def set_led_type(self, rgb_type):
        try:
            led_type = ['RGB', 'RBG', 'GRB', 'GBR', 'BRG', 'BGR']
            led_type_offset = [0x06, 0x09, 0x12, 0x21, 0x18, 0x24]
            index = led_type.index(rgb_type)
            self.led_red_offset = (led_type_offset[index] >> 4) & 0x03
            self.led_green_offset = (led_type_offset[index] >> 2) & 0x03
            self.led_blue_offset = (led_type_offset[index] >> 0) & 0x03
            return index
        except ValueError:
            self.led_red_offset = 1
            self.led_green_offset = 0
            self.led_blue_offset = 2
            return -1

    # Set global brightness scaling (0..255) and reapply colors.
    def set_led_brightness(self, brightness):
        self.led_brightness = brightness
        for i in range(self.get_led_count()):
            self.set_led_rgb_data(i, self.led_original_color)

    # Set a single LED color (original and scaled buffers).
    def set_ledpixel(self, index, r, g, b):
        p = [0, 0, 0]
        p[self.led_red_offset] = round(r * self.led_brightness / 255)
        p[self.led_green_offset] = round(g * self.led_brightness / 255)
        p[self.led_blue_offset] = round(b * self.led_brightness / 255)
        self.led_original_color[index * 3 + self.led_red_offset] = r
        self.led_original_color[index * 3 + self.led_green_offset] = g
        self.led_original_color[index * 3 + self.led_blue_offset] = b
        for i in range(3):
            self.led_color[index * 3 + i] = p[i]

    # Set scaled color buffer for one LED.
    def set_led_color_data(self, index, r, g, b):
        p = [0, 0, 0]
        p[self.led_red_offset] = r
        p[self.led_green_offset] = g
        p[self.led_blue_offset] = b
        for i in range(3):
            self.led_color[index * 3 + i] = p[i]

    # Convenience: set LED color from [r,g,b] list.
    def set_led_rgb_data(self, index, color):
        self.set_led_color_data(index, color[0], color[1], color[2])

    # Set all LEDs to the same [r,g,b] color and update the strip.
    def set_all_led_color(self, r, g, b):
        for i in range(self.get_led_count()):
            self.set_led_color_data(i, r, g, b)
        self.show()

    # Set all LEDs from a [r,g,b] list and update the strip.
    def set_all_led_rgb(self, color):
        for i in range(self.get_led_count()):
            self.set_led_rgb_data(i, color)
        self.show()

    # Encode color buffer as SPI bitstream (8-bit per color) and transmit.
    def write_ws2812_numpy8(self):
        d = numpy.array(self.led_color).ravel()
        tx = numpy.zeros(len(d) * 8, dtype=numpy.uint8)
        for ibit in range(8):
            tx[7 - ibit::8] = ((d >> ibit) & 1) * 0x78 + 0x80
        if self.led_init_state != 0:
            if self.bus == 0:
                self.spi.xfer(tx.tolist(), int(8 / 1.25e-6))
            else:
                self.spi.xfer(tx.tolist(), int(8 / 1.0e-6))

    # Encode color buffer as SPI bitstream (4-bit mode) and transmit.
    def write_ws2812_numpy4(self):
        d = numpy.array(self.led_color).ravel()
        tx = numpy.zeros(len(d) * 4, dtype=numpy.uint8)
        for ibit in range(4):
            tx[3 - ibit::4] = ((d >> (2 * ibit + 1)) & 1) * 0x60 + ((d >> (2 * ibit + 0)) & 1) * 0x06 + 0x88
        if self.led_init_state != 0:
            if self.bus == 0:
                self.spi.xfer(tx.tolist(), int(4 / 1.25e-6))
            else:
                self.spi.xfer(tx.tolist(), int(4 / 1.0e-6))

    # Update the strip by writing prepared SPI bitstream.
    def show(self, mode=1):
        if mode == 1:
            write_ws2812 = self.write_ws2812_numpy8
        else:
            write_ws2812 = self.write_ws2812_numpy4
        write_ws2812()
