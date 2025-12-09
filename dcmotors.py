# motor_auto_test.py
# Automatically test forward/backward/left/right with no keyboard input.

from machine import Pin, PWM
import time

FREQ = 5000
MAX_DUTY = 1023
SPEED = 700

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
def motor_a_forward(speed):
    set_pwm(pwm0, speed)
    set_pwm(pwm1, 0)

def motor_a_reverse(speed):
    set_pwm(pwm0, 0)
    set_pwm(pwm1, speed)

def motor_b_forward(speed):
    set_pwm(pwm2, speed)
    set_pwm(pwm3, 0)

def motor_b_reverse(speed):
    set_pwm(pwm2, 0)
    set_pwm(pwm3, speed)

def stop_all():
    set_pwm(pwm0, 0)
    set_pwm(pwm1, 0)
    set_pwm(pwm2, 0)
    set_pwm(pwm3, 0)

# --- high level (your corrected mapping) ---
# electrical reverse = robot forward
def go_forward():
    motor_a_reverse(SPEED)
    motor_b_reverse(SPEED)
    print("FORWARD")

def go_backward():
    motor_a_forward(SPEED)
    motor_b_forward(SPEED)
    print("BACKWARD")

def turn_left():
    motor_a_reverse(SPEED)
    motor_b_forward(SPEED)
    print("LEFT")

def turn_right():
    motor_a_forward(SPEED)
    motor_b_reverse(SPEED)
    print("RIGHT")

# --- Auto Test Sequence ---
def main():
    print("=== AUTO MOTOR TEST START ===")

    stop_all()
    time.sleep(1)

    # Forward test
    go_forward()
    time.sleep(2)

    # Backward test
    go_backward()
    time.sleep(2)

    # Left turn
    turn_left()
    time.sleep(1.5)

    # Right turn
    turn_right()
    time.sleep(1.5)

    # Done
    stop_all()
    print("=== AUTO MOTOR TEST COMPLETE ===")

if __name__ == "__main__":
    main()
