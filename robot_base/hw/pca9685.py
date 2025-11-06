"""
PCA9685 PWM Driver Module

This module provides interface for PCA9685 16-channel PWM driver
commonly used for servo and motor control in robotics.
Enhanced with actual I2C communication and motor control features.
"""

import time
import math
import logging
from typing import Optional

logger = logging.getLogger(__name__)

try:
    import smbus
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False
    logger.warning("smbus not available - running in simulation mode")


class PCA9685:
    """PCA9685 16-channel PWM driver interface with real I2C communication."""
    
    # PCA9685 register addresses
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
    
    def __init__(self, address: int = 0x40, bus_number: int = 1, frequency: int = 50):
        """
        Initialize PCA9685 PWM driver.
        
        Args:
            address: I2C address of the PCA9685 chip
            bus_number: I2C bus number (default 1 for Raspberry Pi)
            frequency: PWM frequency in Hz
        """
        self.address = address
        self.frequency = frequency
        self.channels = {}
        self.bus = None
        self._simulation_mode = not I2C_AVAILABLE
        
        if I2C_AVAILABLE:
            try:
                self.bus = smbus.SMBus(bus_number)
                # Wake up the PCA9685
                self._write_register(self.__MODE1, 0x00)
                self.set_pwm_freq(frequency)
                logger.info(f"PCA9685 initialized at address 0x{address:02x}, frequency={frequency}Hz")
            except Exception as e:
                logger.error(f"Failed to initialize PCA9685: {e}")
                self._simulation_mode = True
        
        if self._simulation_mode:
            logger.warning("PCA9685 running in simulation mode")
    
    def _write_register(self, reg: int, value: int) -> None:
        """Write 8-bit value to a register over I2C."""
        if self._simulation_mode:
            return
            
        try:
            self.bus.write_byte_data(self.address, reg, value)
        except Exception as e:
            logger.error(f"I2C write error: {e}")
    
    def _read_register(self, reg: int) -> int:
        """Read 8-bit value from a register over I2C."""
        if self._simulation_mode:
            return 0
            
        try:
            return self.bus.read_byte_data(self.address, reg)
        except Exception as e:
            logger.error(f"I2C read error: {e}")
            return 0
    
    def set_pwm_freq(self, freq: float) -> None:
        """Set the PWM frequency for all channels."""
        # Calculate prescaler value
        prescaleval = 25000000.0  # 25MHz oscillator
        prescaleval /= 4096.0     # 12-bit resolution
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        
        # Save current mode and enter sleep
        oldmode = self._read_register(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10  # Sleep mode
        self._write_register(self.__MODE1, newmode)
        
        # Set prescaler
        self._write_register(self.__PRESCALE, prescale)
        
        # Restore mode and enable auto-increment
        self._write_register(self.__MODE1, oldmode)
        time.sleep(0.005)  # Wait for oscillator to stabilize
        self._write_register(self.__MODE1, oldmode | 0x80)
    
    def set_pwm(self, channel: int, on: int, off: int) -> None:
        """
        Set PWM values for a specific channel.
        
        Args:
            channel: Channel number (0-15)
            on: PWM on time (0-4095)
            off: PWM off time (0-4095)
        """
        if not 0 <= channel <= 15:
            raise ValueError(f"Channel must be 0-15, got {channel}")
        
        if not 0 <= on <= 4095:
            raise ValueError(f"PWM on value must be 0-4095, got {on}")
            
        if not 0 <= off <= 4095:
            raise ValueError(f"PWM off value must be 0-4095, got {off}")
        
        self.channels[channel] = {'on': on, 'off': off}
        logger.debug(f"Set PWM channel {channel}: on={on}, off={off}")
        
        # Write PWM values to hardware registers
        self._write_pwm_registers(channel, on, off)
    
    def set_motor_pwm(self, channel: int, duty: int) -> None:
        """
        Set PWM duty cycle for motor control (Freenove style).
        
        Args:
            channel: PWM channel (0-15)
            duty: Duty cycle value (-4095 to 4095, 0 = brake)
        """
        # Clamp duty to valid range
        duty = max(min(duty, 4095), -4095)
        
        if duty == 0:
            # Full brake - both pins high
            self.set_pwm(channel, 4095, 4095)
        else:
            # Normal PWM operation
            self.set_pwm(channel, 0, abs(duty))
    
    def set_duty_cycle(self, channel: int, duty_cycle: float) -> None:
        """
        Set duty cycle for a channel (0.0 to 1.0).
        
        Args:
            channel: Channel number (0-15)
            duty_cycle: Duty cycle as fraction (0.0 to 1.0)
        """
        if not 0.0 <= duty_cycle <= 1.0:
            raise ValueError(f"Duty cycle must be 0.0-1.0, got {duty_cycle}")
        
        # Convert duty cycle to PWM values
        off_value = int(duty_cycle * 4095)
        self.set_pwm(channel, 0, off_value)
    
    def set_servo_angle(self, channel: int, angle: float, 
                       min_pulse: int = 150, max_pulse: int = 600) -> None:
        """
        Set servo angle (0 to 180 degrees).
        
        Args:
            channel: Channel number (0-15)
            angle: Angle in degrees (0-180)
            min_pulse: Minimum pulse width in clock ticks
            max_pulse: Maximum pulse width in clock ticks
        """
        if not 0 <= angle <= 180:
            raise ValueError(f"Angle must be 0-180 degrees, got {angle}")
        
        # Map angle to pulse width
        pulse_range = max_pulse - min_pulse
        pulse_width = min_pulse + (angle / 180.0) * pulse_range
        
        self.set_pwm(channel, 0, int(pulse_width))
    
    def _write_pwm_registers(self, channel: int, on: int, off: int) -> None:
        """
        Write PWM values to hardware registers.
        
        Args:
            channel: Channel number
            on: PWM on value
            off: PWM off value
        """
        # Calculate register addresses for this channel
        on_l_reg = self.__LED0_ON_L + 4 * channel
        on_h_reg = self.__LED0_ON_H + 4 * channel
        off_l_reg = self.__LED0_OFF_L + 4 * channel
        off_h_reg = self.__LED0_OFF_H + 4 * channel
        
        # Split 12-bit values into bytes and write to registers
        self._write_register(on_l_reg, on & 0xFF)
        self._write_register(on_h_reg, on >> 8)
        self._write_register(off_l_reg, off & 0xFF)
        self._write_register(off_h_reg, off >> 8)
    
    def reset(self) -> None:
        """Reset all channels to off state."""
        # Set all channels to 0 PWM
        self._write_register(self.__ALLLED_ON_L, 0)
        self._write_register(self.__ALLLED_ON_H, 0)
        self._write_register(self.__ALLLED_OFF_L, 0)
        self._write_register(self.__ALLLED_OFF_H, 0)
        
        self.channels.clear()
        logger.info("PCA9685 reset - all channels off")
    
    def close(self) -> None:
        """Clean shutdown - stop all outputs and close I2C bus."""
        self.reset()
        
        if self.bus and not self._simulation_mode:
            try:
                self.bus.close()
                logger.info("PCA9685 closed successfully")
            except Exception as e:
                logger.error(f"Error closing PCA9685: {e}")
    
    def get_channel_status(self, channel: int) -> Optional[dict]:
        """
        Get status of a specific channel.
        
        Args:
            channel: Channel number (0-15)
            
        Returns:
            Dictionary with channel status or None if not configured
        """
        return self.channels.get(channel)
    
    def get_all_channels(self) -> dict:
        """Get status of all configured channels."""
        return self.channels.copy()