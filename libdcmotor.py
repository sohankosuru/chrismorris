from machine import Pin, PWM

class DCMotors:

    def __init__(self, pwm0, pwm1, pwm2, pwm3):
        self.pwm0 = pwm0
        self.pwm1 = pwm1
        self.pwm2 = pwm2
        self.pwm3 = pwm3

    def forward1(self):
        self.pwm0.duty(0)
        self.pwm1.duty(1023)

    def forward2(self):
        self.pwm3.duty(0)
        self.pwm2.duty(1023)
