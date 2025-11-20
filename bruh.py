from machine import PWM, Pin

pin12 = Pin(12)
pin13 = Pin(13)
pin25 = Pin(25)
pin26 = Pin(26)


# 10-bit PWM
pwm0 = PWM(pin12, freq=5000, duty_u16=32768)
pwm1 = PWM(pin13, freq=5000, duty_u16=32768)
pwm2 = PWM(pin25, freq=5000, duty_u16=32768)
pwm3 = PWM(pin26, freq=5000, duty_u16=32768)

while True:
    
    pwm0.duty(1023)
    pwm2.duty(0)
    pwm1.duty(0)
    pwm3.duty(1023)
def forward():
    pwm0.duty(1023)
    pwm2.duty(0)
    pwm1.duty(0)
    pwm3.duty(1023)
    
