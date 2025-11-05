#!/usr/bin/env python3
"""
PCA9685 PWM driver for motor control.
This module interfaces with the PCA9685 16-channel PWM driver via I2C.
"""

import time
import math


class PCA9685:
    """Driver for PCA9685 16-channel 12-bit PWM controller."""
    
    # PCA9685 register addresses
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1 = 0x00
    __MODE2 = 0x01
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    def __init__(self, address=0x40, debug=False):
        """Initialize PCA9685 with I2C address."""
        self.address = address
        self.debug = debug
        try:
            import smbus
            self.bus = smbus.SMBus(1)  # I2C bus 1 on Raspberry Pi
        except ImportError:
            if self.debug:
                print("Warning: smbus not available, running in simulation mode")
            self.bus = None
        
        if self.bus:
            self.write(self.__MODE1, 0x00)
            time.sleep(0.01)

    def write(self, reg, value):
        """Write a byte to a register."""
        if self.bus:
            try:
                self.bus.write_byte_data(self.address, reg, value)
            except Exception as e:
                if self.debug:
                    print(f"I2C write error: {e}")
        elif self.debug:
            print(f"[SIM] Write reg 0x{reg:02x} = 0x{value:02x}")

    def read(self, reg):
        """Read a byte from a register."""
        if self.bus:
            try:
                return self.bus.read_byte_data(self.address, reg)
            except Exception as e:
                if self.debug:
                    print(f"I2C read error: {e}")
                return 0
        elif self.debug:
            print(f"[SIM] Read reg 0x{reg:02x}")
            return 0

    def set_pwm_freq(self, freq):
        """Set PWM frequency (typically 50Hz for servos, 1000Hz for motors)."""
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        if self.debug:
            print(f"Setting PWM frequency to {freq} Hz")
            print(f"Estimated pre-scale: {prescaleval}")
        
        prescale = int(math.floor(prescaleval + 0.5))
        if self.debug:
            print(f"Final pre-scale: {prescale}")

        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10    # sleep
        self.write(self.__MODE1, newmode)    # go to sleep
        self.write(self.__PRESCALE, prescale)
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def set_pwm(self, channel, on, off):
        """Set PWM for a specific channel."""
        if self.debug:
            print(f"Channel {channel}: on={on}, off={off}")
        
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)

    def set_motor_pwm(self, channel, duty):
        """Set motor PWM duty cycle.
        
        Args:
            channel: PWM channel (0-15)
            duty: Duty cycle value (0-4095, or 0 to stop)
        """
        if duty == 0:
            # Stop the motor
            self.set_pwm(channel, 0, 0)
        elif duty > 0:
            # Forward direction
            duty = min(duty, 4095)
            self.set_pwm(channel, 0, duty)
        else:
            # This shouldn't happen in normal motor control
            # but handle it gracefully
            self.set_pwm(channel, 0, 0)

    def close(self):
        """Clean shutdown - stop all PWM outputs."""
        if self.debug:
            print("Closing PCA9685 - stopping all PWM outputs")
        
        # Turn off all PWM outputs
        for i in range(16):
            self.set_pwm(i, 0, 0)
        
        if self.bus:
            try:
                self.bus.close()
            except:
                pass