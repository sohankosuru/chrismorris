# motor_auto_test.py
# Automatically test forward/backward/left/right with no keyboard input.

from machine import Pin, PWM
import time

FREQ = 5000
MAX_DUTY = 1023

# separate speeds
SPEED_A = 700
SPEED_B = 675

# Motor pins (your wiring)
pwm0 = PWM(Pin(26), freq=FREQ)  # Motor A input 1
pwm1 = PWM(Pin(27), freq=FREQ)  # Motor A input 2
pwm2 = PWM(Pin(33), freq=FREQ)  # Motor B input 1
pwm3 = PWM(Pin(32), freq=FREQ)  # Motor B input 2

def set_pwm(pwm, duty):
    duty = max(0, min(MAX_DUTY, duty))
    try:
        pwm.duty(duty)
    except AttributeError:
        scale = 65535 // MAX_DUTY
        pwm.duty_u16(int(duty * scale))

# --- motor primitives ---
def motor_a_forward():
    set_pwm(pwm0, SPEED_A)
    set_pwm(pwm1, 0)

def motor_a_reverse():
    set_pwm(pwm0, 0)
    set_pwm(pwm1, SPEED_A)

def motor_b_forward():
    set_pwm(pwm2, SPEED_B)
    set_pwm(pwm3, 0)

def motor_b_reverse():
    set_pwm(pwm2, 0)
    set_pwm(pwm3, SPEED_B)

def stop_all():
    set_pwm(pwm0, 0)
    set_pwm(pwm1, 0)
    set_pwm(pwm2, 0)
    set_pwm(pwm3, 0)

# --- high level commands ---
def go_forward():
    motor_a_reverse()
    motor_b_reverse()
    print("FORWARD")

def go_backward():
    motor_a_forward()
    motor_b_forward()
    print("BACKWARD")

def turn_left():
    motor_a_reverse()
    motor_b_forward()
    print("LEFT")

def turn_right():
    motor_a_forward()
    motor_b_reverse()
    print("RIGHT")

# --- Auto Test Sequence ---
def main():
    print("=== AUTO MOTOR TEST START ===")

    stop_all()
    time.sleep(1)

    go_forward()
    time.sleep(200)

    go_backward()
    time.sleep(2)

    turn_left()
    time.sleep(1.5)

    turn_right()
    time.sleep(1.5)

    stop_all()
    print("=== AUTO MOTOR TEST COMPLETE ===")

if __name__ == "__main__":
    main()
