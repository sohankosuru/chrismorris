# main.py – ONLY mission zone logic, using your calibrated speeds
# Motor A (left) = 700, Motor B (right) = 675 for straight forward

from machine import Pin, PWM
from time import sleep
from math import pi
from enes100 import enes100

# ============================
#  MOTOR SETUP
# ============================

FREQ = 5000
MAX_DUTY = 1023

# Your tuned speeds
LEFT_FWD_SPEED  = 685   # motor A (left)
RIGHT_FWD_SPEED = 675   # motor B (right)

TURN_SPEED = 450        # for rotation in place

# Motor pins (same as your working scripts)
pwm0 = PWM(Pin(26), freq=FREQ)   # left motor IN1
pwm1 = PWM(Pin(27), freq=FREQ)   # left motor IN2
pwm2 = PWM(Pin(33), freq=FREQ)   # right motor IN1
pwm3 = PWM(Pin(32), freq=FREQ)   # right motor IN2

def set_pwm(p, d):
    if d < 0: d = 0
    if d > MAX_DUTY: d = MAX_DUTY
    try:
        p.duty(int(d))
    except:
        p.duty_u16(int(d * 64))

def stop():
    set_pwm(pwm0,0); set_pwm(pwm1,0)
    set_pwm(pwm2,0); set_pwm(pwm3,0)

# IMPORTANT: due to wiring, “forward” (front-first) is electrical reverse.
def drive_forward():
    # front of OTV moves forward
    set_pwm(pwm0,0);               # left IN1 low
    set_pwm(pwm1,LEFT_FWD_SPEED)   # left IN2 PWM
    set_pwm(pwm2,0);               # right IN1 low
    set_pwm(pwm3,RIGHT_FWD_SPEED)  # right IN2 PWM

def rotate_left(speed=TURN_SPEED):
    # left wheel backward, right wheel forward
    set_pwm(pwm0,0);      set_pwm(pwm1,speed)   # left backward
    set_pwm(pwm2,speed);  set_pwm(pwm3,0)       # right forward

def rotate_right(speed=TURN_SPEED):
    set_pwm(pwm0,speed);  set_pwm(pwm1,0)       # left forward
    set_pwm(pwm2,0);      set_pwm(pwm3,speed)   # right backward

# ============================
#  ANGLE + POSITION HELPERS
# ============================

def norm_ang(a):
    from math import pi
    while a > pi: a -= 2*pi
    while a < -pi: a += 2*pi
    return a

def turn_to(target):
    """Rotate until facing target angle (0 or pi)."""
    target = norm_ang(target)
    while True:
        th = enes100.theta
        if th == -1:
            stop()
            sleep(0.1)
            continue

        err = norm_ang(target - th)

        # about ~6 degrees tolerance
        if abs(err) < 0.10:
            stop()
            return

        if err > 0:
            rotate_left()
        else:
            rotate_right()

        sleep(0.03)

def forward_to_y(target_y):
    """
    Just drive straight forward until y is close to target_y.
    We trust your mechanical drift correction (700/675).
    """
    while True:
        if not enes100.is_visible:
            stop()
            sleep(0.1)
            continue

        y = enes100.y

        # within ~4 cm → good enough
        if abs(y - target_y) < 0.04:
            stop()
            return

        drive_forward()
        sleep(0.05)

# ============================
#  MISSION LOGIC (ONLY THIS)
# ============================

def run_mission_zone_only():
    enes100.print("MISSION ZONE ONLY MODE (700/675 speeds)")

    # wait until marker is visible
    while not enes100.is_visible:
        sleep(0.1)

    start_y = enes100.y
    enes100.print("Start y = {:.3f}".format(start_y))

    if start_y < 1.0:
        # We are at Mission B (bottom)
        TARGET_Y = 1.0
        TARGET_ANGLE = 0.0
        enes100.print("Assuming Mission B → target y=1.25, theta=0")
    else:
        # We are at Mission A (top)
        TARGET_Y = 0.5
        TARGET_ANGLE = pi
        enes100.print("Assuming Mission A → target y=0.75, theta=pi")

    # 1) Turn to face the correct direction along mission line
    turn_to(TARGET_ANGLE)

    # 2) Drive straight to target_y
    forward_to_y(TARGET_Y)

    enes100.print("MISSION ZONE COMPLETE (stopped at y≈{:.2f})".format(TARGET_Y))
    stop()

# ============================
#  MAIN
# ============================

def main():
    stop()
    enes100.begin("Team Chris Morris AWOG", "MATERIAL", 7, 1120)
    sleep(0.3)
    run_mission_zone_only()

if __name__ == "__main__":
    main()
