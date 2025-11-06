#!/usr/bin/python
import time
import math
import smbus

class PCA9685:
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    # Initialize I2C device at address and wake the controller.
    def __init__(self, address: int = 0x40, debug: bool = False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)

    # Write 8-bit value to a register over I2C.
    def write(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.address, reg, value)

    # Read 8-bit value from a register over I2C.
    def read(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

    # Configure global PWM frequency.
    def set_pwm_freq(self, freq: float) -> None:
        prescaleval = 25000000.0
        prescaleval /= 4096.0
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = math.floor(prescaleval + 0.5)

        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10
        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    # Set raw PWM on/off counts for a given channel.
    def set_pwm(self, channel: int, on: int, off: int) -> None:
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)

    # Convenience: set duty cycle for motor channel.
    def set_motor_pwm(self, channel: int, duty: int) -> None:
        self.set_pwm(channel, 0, duty)

    # Close I2C bus handle.
    def close(self) -> None:
        self.bus.close()
