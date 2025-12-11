# main.py – mission zone (unchanged) + very simple obstacle zone
# Motor A (left) = 685, Motor B (right) = 675 for straight forward

from machine import Pin, PWM, ADC
from time import sleep
from math import pi
from enes100 import enes100
from hcsr04 import HCSR04

# ============================
#  MOTOR SETUP
# ============================

FREQ = 5000
MAX_DUTY = 1023

# Your tuned speeds
LEFT_FWD_SPEED  = 685   # motor A (left)
RIGHT_FWD_SPEED = 675   # motor B (right)

TURN_SPEED = 450        # for rotation in place

capsensor_treshold = -1

# Motor pins (same as your working scripts)
pwm0 = PWM(Pin(26), freq=FREQ)   # left motor IN1
pwm1 = PWM(Pin(27), freq=FREQ)   # left motor IN2
pwm2 = PWM(Pin(33), freq=FREQ)   # right motor IN1
pwm3 = PWM(Pin(32), freq=FREQ)   # right motor IN2

_capsensorpin = Pin(34)
capsensor = ADC(34)

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
    set_pwm(pwm0,0)                # left IN1 low
    set_pwm(pwm1,LEFT_FWD_SPEED)   # left IN2 PWM
    set_pwm(pwm2,0)                # right IN1 low
    set_pwm(pwm3,RIGHT_FWD_SPEED)  # right IN2 PWM
    
def drive_backward():
    # front of OTV moves forward
    set_pwm(pwm1,0)                # left IN1 low
    set_pwm(pwm0,LEFT_FWD_SPEED)   # left IN2 PWM
    set_pwm(pwm3,0)                # right IN1 low
    set_pwm(pwm2,RIGHT_FWD_SPEED)  # right IN2 PWM

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
    while a > pi:  a -= 2*pi
    while a < -pi: a += 2*pi
    return a

def turn_to(target):
    
    """Rotate until facing target angle (0, pi, or +/-pi/2)."""
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
# def _turn_to_angle(target):
#     """Basic in-place turn to target angle (no forward nudge)."""
#     target = norm_ang(target)
#     while True:
#         th = enes100.theta
#         if th == -1:
#             # marker not visible
#             stop()
#             sleep(0.1)
#             continue
# 
#         err = norm_ang(target - th)
# 
#         # about ~6 degrees tolerance
#         if abs(err) < 0.10:
#             stop()
#             return
# 
#         if err > 0:
#             rotate_left()
#         else:
#             rotate_right()
# 
#         sleep(0.03)

# def turn_to(target, nudge_time=0.60):
#     """
#     Turn in two stages to avoid the back getting stuck:
#     1) Turn to halfway between current angle and target.
#     2) Move forward a tiny bit.
#     3) Turn the rest of the way to target.
#     """
#     # Make sure we can see the tag first
#     while not enes100.is_visible:
#         stop()
#         sleep(0.1)
# 
#     # Current heading and total error
#     th0 = enes100.theta
#     if th0 == -1:
#         # fallback: just do a simple turn
#         _turn_to_angle(target)
#         return
# 
#     target = norm_ang(target)
#     err0 = norm_ang(target - th0)
# 
#     # Midpoint angle = halfway between now and target
#     mid_angle = norm_ang(th0 + err0 / 2.0)
# 
#     enes100.print("turn_to: stage 1 → mid_angle={:.2f}".format(mid_angle))
#     _turn_to_angle(mid_angle)
# 
#     # Small forward nudge to pull the back end away from obstacles
#     enes100.print("turn_to: nudge forward")
#     drive_forward()
#     sleep(nudge_time)
#     stop()
# 
#     # Final precise turn to the actual target
#     enes100.print("turn_to: stage 2 → target={:.2f}".format(target))
#     _turn_to_angle(target)

def calibrate_sensor():
    ranges = []
    for i in range(100):
        val = capsensor.read_uv()
        ranges.append(val)
        
    capsensor_treshold = min(ranges)
    
    enes100.print("capsensor calibrated to: soft ball")
        


def forward_to_y(target_y):
    """
    Just drive straight forward until y is close to target_y.
    We trust your mechanical drift correction (685/675).
    Assumes you ALREADY turned to 0 (up) or pi (down).
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
        
def backward_to_y(target_y):
    """
    Just drive straight forward until y is close to target_y.
    We trust your mechanical drift correction (685/675).
    Assumes you ALREADY turned to 0 (up) or pi (down).
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

        drive_backward()
        sleep(0.05)

# ------- Extra helpers for obstacle zone -------

# Calibrated cross-arena angles from your setup
ANGLE_POS_X = -1.54    # facing +x (towards obstacles / limbo)
ANGLE_NEG_X =  1.54    # facing -x (back toward mission zone)

def forward_to_y_auto(target_y):
    """Go to target_y, choosing 0 or pi based on current y."""
    while not enes100.is_visible:
        stop()
        sleep(0.1)
    y = enes100.y
    if y < target_y:
        turn_to(0.0)        # go up (+y)
    else:
        turn_to(pi)         # go down (-y)
    forward_to_y(target_y)

def forward_to_x(target_x):
    """Drive along x to target_x using +/- pi/2."""
    while not enes100.is_visible:
        stop()
        sleep(0.1)

    x = enes100.x
    # choose heading based on which side target is on
    if x < target_x:
        heading = ANGLE_POS_X     # face +x
    else:
        heading = ANGLE_NEG_X     # face -x

    turn_to(heading)

    while True:
        if not enes100.is_visible:
            stop()
            sleep(0.1)
            continue

        x = enes100.x
        if abs(x - target_x) < 0.04:   # ~4 cm
            stop()
            return

        drive_forward()
        sleep(0.05)

# ============================
#  RIGHT ULTRASONIC (SIDE)
# ============================

# Right sensor on side of OTV
RIGHT_TRIG_PIN = 0
RIGHT_ECHO_PIN = 25
right_sonar = HCSR04(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN)
SAFE_RIGHT_CM = 40

def get_right_cm():
    try:
        d = right_sonar.distance_cm()
        return d
    except OSError:
        return None
def right_close_stopped(threshold=SAFE_RIGHT_CM):
    """
    Stop, take several readings from right sonar, average them,
    and decide if 'right < threshold'.
    """
    stop()
    d = get_right_cm()
    # Keep the print short so it fits in WiFi console nicely
#     enes100.print("Right sonar reading")
    enes100.print("Right sonar reading = {}".format(d))
    return d < threshold
# def right_close_stopped(threshold=SAFE_RIGHT_CM):
#     d = get_right_cm()
#     if d is None:
#         return False
#     return d < threshold


def do_mission():
    enes100.print("Starting mission...")
    
    while not enes100.is_visible:
        sleep(0.1)
        enes100.print("in not visible loop")
    
  
        
    curr_y = enes100.y
    
    enes100.print("getting current y")
     
    if start_y < 1.0:
        # We are at Mission B (bottom)
        TARGET_Y = 1.8
        TARGET_ANGLE = pi
        enes100.print("Assuming Mission B → target y=1.0, theta=0")
    else:
        # We are at Mission A (top)
        TARGET_Y = 0.2
        TARGET_ANGLE = 0
        enes100.print("Assuming Mission A → target y=0.5, theta=pi")
        
    turn_to(TARGET_ANGLE)
    
    enes100.print("turning backwards")
    
    backward_to_y(TARGET_Y)
    
    # detect it rn
    if capsensor.read_uv() < capsensor_treshold:
        enes100.mission('MATERIAL_TYPE', 'PLASTIC')
        enes100.print("lastic ball")
    else:
        enes100.mission('MATERIAL_TYPE', 'FOAM')
        enes100.print("foam")
        
    enes100.print("MISSION COMPLETE (stopped at y≈{:.2f})".format(TARGET_Y))
    stop()
    
# ============================
#  MISSION LOGIC (UNCHANGED)
# ============================

def run_mission_zone_only():
    enes100.print("MISSION ZONE ONLY MODE (685/675 speeds)")

    # wait until marker is visible
    while not enes100.is_visible:
        sleep(0.1)

    start_y = enes100.y
    enes100.print("Start y = {:.3f}".format(start_y))

    if start_y < 1.0:
        # We are at Mission B (bottom)
        TARGET_Y = 1.0
        TARGET_ANGLE = 0.0
        enes100.print("Assuming Mission B → target y=1.0, theta=0")
    else:
        # We are at Mission A (top)
        TARGET_Y = 1.0
        TARGET_ANGLE = pi
        enes100.print("Assuming Mission A → target y=0.5, theta=pi")

    # 1) Turn to face the correct direction along mission line
    turn_to(TARGET_ANGLE)

    # 2) Drive straight to target_y
    forward_to_y(TARGET_Y)

    enes100.print("MISSION ZONE COMPLETE (stopped at y≈{:.2f})".format(TARGET_Y))
    stop()
    


# ============================
#  OBSTACLE ZONE (YOUR LOGIC)
# ============================

def obstacle_zone():
    enes100.print("Starting OBSTACLE ZONE...")

    # ---- Step 1: go to (x=0.6, y=0.7, theta=0) ----
    forward_to_x(0.6)
    forward_to_y_auto(0.7)
    turn_to(0.0) # face up (+y)

    enes100.print("At x≈0.6,y≈0.7")

    # ---- Column 1 logic at x≈0.7 ----
    if right_close_stopped():
        # Go to y = 1.0
        forward_to_y_auto(1.2)
        enes100.print("x≈0.6,y≈1.2")
        if right_close_stopped():
            # Go to y = 1.75
            forward_to_y_auto(1.75)
            enes100.print("Column near x≈0.6 cleared at y≈1.75")
        else:
            enes100.print("Column at x≈0.6 only at lower row, cleared.")
    else:
        # ---- No obstacle near first column → check second column ----
        enes100.print("No obstacle at x≈0.6; moving to x≈1.6 for second column")

        # go to (x=1.6, y=0.5, theta=0)
        forward_to_x(1.6)
        forward_to_y_auto(0.7)
        turn_to(0.0)

        enes100.print("At x≈1.5,y≈0.5")

        if right_close_stopped():
            # Go to y = 1.2
            forward_to_y_auto(1.2)
            enes100.print("x≈1.5,y≈1.2")
            if right_close_stopped():
                # Go to y = 1.75
                forward_to_y_auto(1.75)
                enes100.print("Column near x≈1.5 cleared at y≈1.75")
            else:
                enes100.print("Column at x≈1.5 only at lower row, cleared.")
        else:
            enes100.print("No obstacle at x≈1.5 either (lucky run).")
    # Turn to -pi/2
    turn_to(-pi/2)
    # Go To  X = 1.6
    forward_to_x(1.6)
    # Turn to Pi
    turn_to(pi)
    # Go To Y = 0.7
    forward_to_y_auto(0.7)
    # Turn to 0
    turn_to(0.0)
    # ---- Column 2 logic at x≈1.5 ----
    if right_close_stopped():
        # Go to y = 1.2
        forward_to_y_auto(1.2)
        enes100.print("x≈1.5,y≈1.2")
        if right_close_stopped():
            # Go to y = 1.75
            forward_to_y_auto(1.75)
            enes100.print("Column near x≈1.5 cleared at y≈1.75")
        else:
            enes100.print("Column at x≈1.5 only at lower row, cleared.")
    else:
        
        # ---- Final: go to open zone and through limbo row ----
        enes100.print("Going to open zone pose (2.8, 1.3, -pi/2)")

        forward_to_x(2.8)
        forward_to_y_auto(1.3)
        turn_to(-pi/2)  # roughly -1.54 rad, facing +x

        # Drive forward along x until x≈3.5
        while True:
            if not enes100.is_visible:
                stop()
                sleep(0.1)
                continue

            x = enes100.x
            if x >= 3.65:
                stop()
                enes100.print("Navigation successful: reached x≈3.65, y≈{:.2f}".format(enes100.y))
                break

            drive_forward()
            sleep(0.05)
#         # ---- No obstacle near first column → check second column ----
#         enes100.print("No obstacle at x≈0.7; moving to x≈1.6 for second column")
# 
#         # go to (x=1.6, y=0.5, theta=0)
#         forward_to_x(1.6)
#         forward_to_y_auto(0.7)
#         turn_to(0.0)
# 
#         d3 = get_right_cm()
#         enes100.print("At x≈1.6,y≈0.7, right={}".format(d3))
# 
#         if right_close_stopped():
#             # Go to y = 1.0
#             forward_to_y_auto(1.2)
#             d4 = get_right_cm()
#             enes100.print("x≈1.6,y≈1.2, right={}".format(d4))
#             if right_close_stopped():
#                 # Go to y = 1.5
#                 forward_to_y_auto(1.75)
#                 enes100.print("Column near x≈1.75 cleared at y≈1.5")
#             else:
#                 enes100.print("Column at x≈1.6 only at lower row, cleared.")
#         else:
#             enes100.print("No obstacle at x≈1.6 either (lucky run).")
# 
#     # ---- Final: go to open zone and through limbo row ----
#     enes100.print("Going to open zone pose (2.89, 1.5, -pi/2)")
# 
#     forward_to_x(2.89)
#     forward_to_y_auto(1.5)
#     turn_to(-pi/2)  # roughly -1.54 rad, facing +x
# 
#     # Drive forward along x until x≈3.5
#     while True:
#         if not enes100.is_visible:
#             stop()
#             sleep(0.1)
#             continue
# 
#         x = enes100.x
#         if x >= 3.5:
#             stop()
#             enes100.print("Navigation successful: reached x≈3.5, y≈{:.2f}".format(enes100.y))
#             break
# 
#         drive_forward()
#         sleep(0.05)

# ============================
#  MAIN
# ============================

def main():
    stop()
    enes100.begin("Chris Morris AWOG", "MATERIAL", 7, 1120)
    sleep(0.3)
    calibrate_sensor()

    # 1) Mission zone (A <-> B) – EXACTLY your old behavior
    run_mission_zone_only()
    
    do_mission()

    # 2) Obstacle zone using right ultrasonic + fixed waypoints
    obstacle_zone()

    enes100.print("FULL RUN COMPLETE")

if __name__ == "__main__":
    main()
