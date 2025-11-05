"""
PCA9685 PWM Driver Module

This module provides interface for PCA9685 16-channel PWM driver
commonly used for servo and motor control in robotics.
"""

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class PCA9685:
    """PCA9685 16-channel PWM driver interface."""
    
    def __init__(self, address: int = 0x40, frequency: int = 50):
        """
        Initialize PCA9685 PWM driver.
        
        Args:
            address: I2C address of the PCA9685 chip
            frequency: PWM frequency in Hz
        """
        self.address = address
        self.frequency = frequency
        self.channels = {}
        logger.info(f"PCA9685 initialized at address 0x{address:02x}, frequency={frequency}Hz")
    
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
        
        # Placeholder for actual I2C communication
        self._write_pwm_registers(channel, on, off)
    
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
        Write PWM values to hardware registers (placeholder).
        
        Args:
            channel: Channel number
            on: PWM on value
            off: PWM off value
        """
        # Placeholder for actual I2C register writes
        # In real implementation, this would:
        # - Calculate register addresses
        # - Split 12-bit values into bytes
        # - Write to I2C bus
        pass
    
    def reset(self) -> None:
        """Reset all channels to off state."""
        for channel in range(16):
            self.set_pwm(channel, 0, 0)
        self.channels.clear()
        logger.info("PCA9685 reset - all channels off")
    
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