from .pca9685 import PCA9685


class Ordinary_Car:
    # Initialize PCA9685 and set PWM frequency.
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.set_pwm_freq(50)

    # Clamp four duty values to the valid range [-4095, 4095].
    def duty_range(self, duty1, duty2, duty3, duty4):
        duty1 = max(min(duty1, 4095), -4095)
        duty2 = max(min(duty2, 4095), -4095)
        duty3 = max(min(duty3, 4095), -4095)
        duty4 = max(min(duty4, 4095), -4095)
        return duty1, duty2, duty3, duty4

    # Drive left upper wheel forward/backward or brake when duty == 0.
    def left_upper_wheel(self, duty):
        if duty > 0:
            self.pwm.set_motor_pwm(0, 0)
            self.pwm.set_motor_pwm(1, duty)
        elif duty < 0:
            self.pwm.set_motor_pwm(1, 0)
            self.pwm.set_motor_pwm(0, abs(duty))
        else:
            self.pwm.set_motor_pwm(0, 4095)
            self.pwm.set_motor_pwm(1, 4095)

    # Drive left lower wheel forward/backward or brake when duty == 0.
    def left_lower_wheel(self, duty):
        if duty > 0:
            self.pwm.set_motor_pwm(3, 0)
            self.pwm.set_motor_pwm(2, duty)
        elif duty < 0:
            self.pwm.set_motor_pwm(2, 0)
            self.pwm.set_motor_pwm(3, abs(duty))
        else:
            self.pwm.set_motor_pwm(2, 4095)
            self.pwm.set_motor_pwm(3, 4095)

    # Drive right upper wheel forward/backward or brake when duty == 0.
    def right_upper_wheel(self, duty):
        if duty > 0:
            self.pwm.set_motor_pwm(6, 0)
            self.pwm.set_motor_pwm(7, duty)
        elif duty < 0:
            self.pwm.set_motor_pwm(7, 0)
            self.pwm.set_motor_pwm(6, abs(duty))
        else:
            self.pwm.set_motor_pwm(6, 4095)
            self.pwm.set_motor_pwm(7, 4095)

    # Drive right lower wheel forward/backward or brake when duty == 0.
    def right_lower_wheel(self, duty):
        if duty > 0:
            self.pwm.set_motor_pwm(4, 0)
            self.pwm.set_motor_pwm(5, duty)
        elif duty < 0:
            self.pwm.set_motor_pwm(5, 0)
            self.pwm.set_motor_pwm(4, abs(duty))
        else:
            self.pwm.set_motor_pwm(4, 4095)
            self.pwm.set_motor_pwm(5, 4095)

    # Apply four wheel duty values: FL, BL, FR, BR.
    def set_motor_model(self, duty1, duty2, duty3, duty4):
        duty1, duty2, duty3, duty4 = self.duty_range(duty1, duty2, duty3, duty4)
        self.left_upper_wheel(duty1)
        self.left_lower_wheel(duty2)
        self.right_upper_wheel(duty3)
        self.right_lower_wheel(duty4)

    # Stop all wheels and close I2C bus.
    def close(self):
        self.set_motor_model(0, 0, 0, 0)
        self.pwm.close()