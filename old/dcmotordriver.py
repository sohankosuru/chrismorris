from libdcmotor import DCMotors
from machine import Pin, PWM

# 10-bit PWM
pwm0 = PWM(Pin(12), freq=5000)
pwm1 = PWM(Pin(13), freq=5000)
pwm2 = PWM(Pin(25), freq=5000)
pwm3 = PWM(Pin(26), freq=5000)

dcmotor = DCMotors(pwm0, pwm1, pwm2, pwm3)

while True:
    dcmotor.forward1()
    dcmotor.forward2()
