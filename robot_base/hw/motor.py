"""
Motor Control Hardware Module

This module provides hardware abstraction for motor control using PWM.
Enhanced with Freenove-style motor control and actual PCA9685 integration.
Supports differential drive robots with proper motor mapping and safety features.
"""

import logging
from typing import Optional, Tuple
from .pca9685 import PCA9685

logger = logging.getLogger(__name__)


class Motor:
    """Enhanced motor control class for differential drive robots with PCA9685."""
    
    def __init__(self, max_duty: int = 4095, i2c_address: int = 0x40, 
                 motor_pins: Optional[dict] = None, pwm_freq: int = 1000):
        """
        Initialize motor controller with PCA9685 PWM driver.
        
        Args:
            max_duty: Maximum duty cycle value for motor control (0-4095)
            i2c_address: I2C address of PCA9685 (default 0x40)
            motor_pins: Dictionary mapping motor positions to PWM channels
            pwm_freq: PWM frequency in Hz (higher for motors, lower for servos)
        """
        self.max_duty = max_duty
        self.current_speed = 0.0
        self.current_direction = 0.0
        
        # Default motor pin mapping (Freenove-style 4WD)
        if motor_pins is None:
            self.motor_pins = {
                'front_left_1': 0,   # FL motor pin 1
                'front_left_2': 1,   # FL motor pin 2
                'back_left_1': 2,    # BL motor pin 1
                'back_left_2': 3,    # BL motor pin 2
                'front_right_1': 6,  # FR motor pin 1
                'front_right_2': 7,  # FR motor pin 2
                'back_right_1': 4,   # BR motor pin 1
                'back_right_2': 5    # BR motor pin 2
            }
        else:
            self.motor_pins = motor_pins
        
        # Initialize PCA9685 PWM driver
        try:
            self.pwm = PCA9685(address=i2c_address, frequency=pwm_freq)
            logger.info(f"Motor initialized with max_duty={max_duty}, PWM freq={pwm_freq}Hz")
        except Exception as e:
            logger.error(f"Failed to initialize PCA9685: {e}")
            raise
        
        # Stop all motors on initialization
        self.stop()
    
    def _drive_motor(self, pin1: int, pin2: int, duty: int) -> None:
        """
        Drive a single motor with direction control (Freenove style).
        
        Args:
            pin1: First PWM pin for this motor
            pin2: Second PWM pin for this motor  
            duty: Motor duty cycle (-max_duty to +max_duty, 0 = brake)
        """
        if duty > 0:
            # Forward direction
            self.pwm.set_motor_pwm(pin1, 0)
            self.pwm.set_motor_pwm(pin2, duty)
        elif duty < 0:
            # Reverse direction
            self.pwm.set_motor_pwm(pin2, 0)
            self.pwm.set_motor_pwm(pin1, abs(duty))
        else:
            # Brake - both pins high for active braking
            self.pwm.set_motor_pwm(pin1, 4095)
            self.pwm.set_motor_pwm(pin2, 4095)
    
    def _clamp_duty(self, *duties: int) -> Tuple[int, ...]:
        """Clamp duty cycle values to valid range."""
        return tuple(max(min(duty, self.max_duty), -self.max_duty) for duty in duties)
    
    def set_velocity(self, linear_x: float, angular_z: float) -> None:
        """
        Set motor velocities based on linear and angular commands.
        Enhanced with proper differential drive mathematics.
        
        Args:
            linear_x: Forward/backward velocity (-1.0 to 1.0)
            angular_z: Rotational velocity (-1.0 to 1.0)
        """
        # Clamp input values to safe range
        linear_x = max(-1.0, min(1.0, linear_x))
        angular_z = max(-1.0, min(1.0, angular_z))
        
        # Store current commands
        self.current_speed = linear_x
        self.current_direction = angular_z
        
        # Calculate differential drive motor speeds
        # Left side subtracts angular for turning, right side adds
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z
        
        # Normalize if any speed exceeds limits
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 1.0:
            left_speed /= max_speed
            right_speed /= max_speed
        
        # Convert to duty cycle values
        left_duty = int(left_speed * self.max_duty)
        right_duty = int(right_speed * self.max_duty)
        
        # Clamp final duty values
        left_duty, right_duty = self._clamp_duty(left_duty, right_duty)
        
        logger.debug(f"Velocity command: linear={linear_x:.3f}, angular={angular_z:.3f}")
        logger.debug(f"Motor duties: left={left_duty}, right={right_duty}")
        
        # Apply to all motors (4WD setup)
        self.set_motor_model(left_duty, left_duty, right_duty, right_duty)
    
    def set_motor_model(self, front_left: int, back_left: int, 
                       front_right: int, back_right: int) -> None:
        """
        Apply motor duties to all four wheels (Freenove style).
        
        Args:
            front_left: Front left motor duty (-max_duty to +max_duty)
            back_left: Back left motor duty (-max_duty to +max_duty)
            front_right: Front right motor duty (-max_duty to +max_duty)
            back_right: Back right motor duty (-max_duty to +max_duty)
        """
        # Clamp all duty values
        fl, bl, fr, br = self._clamp_duty(front_left, back_left, front_right, back_right)
        
        # Drive each motor with direction control
        self._drive_motor(
            self.motor_pins['front_left_1'], 
            self.motor_pins['front_left_2'], 
            fl
        )
        self._drive_motor(
            self.motor_pins['back_left_1'], 
            self.motor_pins['back_left_2'], 
            bl
        )
        self._drive_motor(
            self.motor_pins['front_right_1'], 
            self.motor_pins['front_right_2'], 
            fr
        )
        self._drive_motor(
            self.motor_pins['back_right_1'], 
            self.motor_pins['back_right_2'], 
            br
        )
        
        logger.debug(f"Motor model: FL={fl}, BL={bl}, FR={fr}, BR={br}")
    
    def turn_left(self, speed: float = 0.5) -> None:
        """Turn left in place at specified speed."""
        speed = max(0.0, min(1.0, speed))
        duty = int(speed * self.max_duty)
        self.set_motor_model(-duty, -duty, duty, duty)
    
    def turn_right(self, speed: float = 0.5) -> None:
        """Turn right in place at specified speed.""" 
        speed = max(0.0, min(1.0, speed))
        duty = int(speed * self.max_duty)
        self.set_motor_model(duty, duty, -duty, -duty)
    
    def move_forward(self, speed: float = 0.5) -> None:
        """Move forward at specified speed."""
        speed = max(0.0, min(1.0, speed))
        duty = int(speed * self.max_duty)
        self.set_motor_model(duty, duty, duty, duty)
    
    def move_backward(self, speed: float = 0.5) -> None:
        """Move backward at specified speed."""
        speed = max(0.0, min(1.0, speed))
        duty = int(speed * self.max_duty)
        self.set_motor_model(-duty, -duty, -duty, -duty)
    
    def stop(self) -> None:
        """Stop all motors immediately with active braking."""
        self.set_motor_model(0, 0, 0, 0)
        self.current_speed = 0.0
        self.current_direction = 0.0
        logger.info("Motors stopped")
    
    def close(self) -> None:
        """Clean shutdown - stop motors and close PWM driver."""
        self.stop()
        try:
            self.pwm.close()
            logger.info("Motor controller closed successfully")
        except Exception as e:
            logger.error(f"Error closing motor controller: {e}")
    
    def get_status(self) -> dict:
        """
        Get current motor status with enhanced information.
        
        Returns:
            Dictionary containing current motor state
        """
        return {
            'max_duty': self.max_duty,
            'current_speed': self.current_speed,
            'current_direction': self.current_direction,
            'is_moving': abs(self.current_speed) > 0.01 or abs(self.current_direction) > 0.01,
            'motor_pins': self.motor_pins,
            'pwm_ready': hasattr(self, 'pwm') and self.pwm is not None
        }