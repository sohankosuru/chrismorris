# main.py – mission zone + simple obstacle zone using 3 front sonars
#
# Mission zone:
#   - If start_y < 1.0 (Mission B): face theta=0 and drive until y≈1.0, then stop.
#   - If start_y > 1.0 (Mission A): face theta=pi and drive until y≈0.5, then stop.
#
# Obstacle zone:
#   - Turn to theta = -pi/2 (toward obstacle/goal side).
#   - Drive forward until x≈2.5 m.
#   - If ANY front sensor < 20 cm → back up, then:
#       * if y < 1.0: detour UP (+Y)
#       * if y > 1.0: detour DOWN (–Y)
#     then re-orient to -pi/2 and continue.
#
# After obstacle zone:
#   - Go to x = 2.8, y = 1.5, theta = -pi/2
#   - Drive forward to x ≈ 3.7 (goal) and stop.

from machine import Pin, PWM
from time import sleep
from math import pi
from enes100 import enes100
from hcsr04 import HCSR04

# ============================
# ANGLES (your calibration)
# ============================

ANGLE_POS_Y = 0.0      # front along +y (B -> A)
ANGLE_NEG_Y = pi       # front along -y (A -> B)
ANGLE_POS_X = -1.54    # front toward obstacle/goal side (+x)
ANGLE_NEG_X = 1.54     # front back toward mission side (-x)

# ============================
# TEAM / ENES CONFIG
# ============================

TEAM_NAME  = "Team Chris Morris AWOG"
TEAM_TYPE  = "MATERIAL"
ARUCO_ID   = 7
ROOM_NUM   = 1120

ENES_READY = False

def debug(msg: str):
    global ENES_READY
    if not ENES_READY:
        return
    try:
        enes100.print(msg[:120])
    except:
        pass

# ============================
# MOTOR SETUP (pins + speeds)
# ============================

FREQ = 5000
MAX_DUTY = 1023

# You said: motor A = 700, motor B = 675 for straight forward
LEFT_FWD_SPEED  = 1000   # motor A (left)
RIGHT_FWD_SPEED = 975   # motor B (right)

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
    debug("STOP")

# IMPORTANT: due to wiring, “forward” (front-first) is electrical reverse.
def drive_forward():
    """Front of OTV moves forward, using your tuned 700/675 speeds."""
    set_pwm(pwm0,0)                 # left IN1 low
    set_pwm(pwm1,LEFT_FWD_SPEED)    # left IN2 PWM
    set_pwm(pwm2,0)                 # right IN1 low
    set_pwm(pwm3,RIGHT_FWD_SPEED)   # right IN2 PWM
    debug("drive_forward")

def drive_backward():
    """Front of OTV moves backward (not used much, but handy for backup)."""
    set_pwm(pwm0,LEFT_FWD_SPEED)
    set_pwm(pwm1,0)
    set_pwm(pwm2,RIGHT_FWD_SPEED)
    set_pwm(pwm3,0)
    debug("drive_backward")

def rotate_left(speed=TURN_SPEED):
    # left wheel backward, right wheel forward
    set_pwm(pwm0,0);      set_pwm(pwm1,speed)   # left backward
    set_pwm(pwm2,speed);  set_pwm(pwm3,0)       # right forward
    debug("rotate_left")

def rotate_right(speed=TURN_SPEED):
    set_pwm(pwm0,speed);  set_pwm(pwm1,0)       # left forward
    set_pwm(pwm2,0);      set_pwm(pwm3,speed)   # right backward
    debug("rotate_right")

# ============================
# ANGLE + POSITION HELPERS
# ============================

def norm_ang(a):
    while a > pi:
        a -= 2*pi
    while a < -pi:
        a += 2*pi
    return a

def wait_for_visibility():
    while not enes100.is_visible:
        debug("Waiting for visibility...")
        sleep(0.1)
    debug("Visible: x={:.3f}, y={:.3f}, theta={:.3f}".format(
        enes100.x, enes100.y, enes100.theta
    ))

def turn_to(target):
    """Rotate until facing target angle."""
    target = norm_ang(target)
    wait_for_visibility()
    while True:
        th = enes100.theta
        if th == -1:
            stop()
            sleep(0.1)
            continue
        err = norm_ang(target - th)
        debug("turn_to: theta={:.3f}, target={:.3f}, err={:.3f}".format(th, target, err))

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
    Drive straight forward until y is close to target_y.
    Uses your drift-corrected speeds (700/675), no wiggle.
    """
    wait_for_visibility()
    while True:
        if not enes100.is_visible:
            stop()
            sleep(0.1)
            continue

        y = enes100.y
        debug("forward_to_y: y={:.3f}, target_y={:.3f}".format(y, target_y))

        # within ~4 cm → good enough
        if abs(y - target_y) < 0.04:
            stop()
            return

        drive_forward()
        sleep(0.05)
        stop()

# Helper to move in y from anywhere
def go_to_y(target_y):
    wait_for_visibility()
    y = enes100.y
    if y < target_y:
        # need to go "up" along +y
        turn_to(ANGLE_POS_Y)
    else:
        # need to go "down" along -y
        turn_to(ANGLE_NEG_Y)
    forward_to_y(target_y)

# ============================
# ULTRASONIC SENSORS (3 on front)
# ============================

FRONT_TRIG_PIN = 4
FRONT_ECHO_PIN = 23

LEFT_TRIG_PIN  = 2
LEFT_ECHO_PIN  = 18

RIGHT_TRIG_PIN = 0
RIGHT_ECHO_PIN = 25

front_center = HCSR04(FRONT_TRIG_PIN, FRONT_ECHO_PIN)
front_left   = HCSR04(LEFT_TRIG_PIN,  LEFT_ECHO_PIN)
front_right  = HCSR04(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN)

SAFE_FRONT_CM = 10.0   # obstacle threshold

def safe_distance(sensor):
    try:
        return sensor.distance_cm()
    except OSError:
        return None

def read_all_distances():
    df = safe_distance(front_center)
    dl = safe_distance(front_left)
    dr = safe_distance(front_right)
    if df is None: df = 999
    if dl is None: dl = 999
    if dr is None: dr = 999
    debug("Sonar FL:{:.1f} FC:{:.1f} FR:{:.1f} cm".format(dl, df, dr))
    return df, dl, dr

def is_front_blocked():
    df, dl, dr = read_all_distances()
    return (df < SAFE_FRONT_CM) or (dl < SAFE_FRONT_CM) or (dr < SAFE_FRONT_CM)

# ============================
# MISSION ZONE ONLY (WORKING PART)
# ============================

def run_mission_zone_only():
    debug("MISSION ZONE ONLY MODE (700/675 speeds)")

    # wait until marker is visible
    wait_for_visibility()
    start_y = enes100.y
    debug("Start y = {:.3f}".format(start_y))

    if start_y < 1.0:
        # We are at Mission B (bottom)
        TARGET_Y = 1.0      # this is what worked for you in practice
        TARGET_ANGLE = ANGLE_POS_Y   # 0.0
        debug("Assuming Mission B → target y≈1.0, theta=0")
    else:
        # We are at Mission A (top)
        TARGET_Y = 0.5
        TARGET_ANGLE = ANGLE_NEG_Y   # pi
        debug("Assuming Mission A → target y≈0.5, theta=pi")

    # 1) Turn to face the correct direction along mission line
    turn_to(TARGET_ANGLE)

    # 2) Drive straight to target_y
    forward_to_y(TARGET_Y)

    debug("MISSION ZONE COMPLETE (stopped at y≈{:.2f})".format(TARGET_Y))
    stop()

# ============================
# OBSTACLE ZONE (SIMPLE LOGIC)
# ============================

DY_DETOUR = 0.40   # how far in y to move around obstacle
Y_MIN      = 0.25
Y_MAX      = 1.75
X_END_OBS  = 2.5   # stop obstacle zone when x >= 2.5

def obstacle_zone_run():
    """
    From opposite mission site:
      - Turn to face obstacle side (theta = -pi/2).
      - Move forward until x≈2.5, detouring in y when blocked.
    Detour rule:
      - If y < 1.0 → detour UP (+Y)
      - If y > 1.0 → detour DOWN (–Y)
    """
    debug("Entering OBSTACLE ZONE RUN")
    wait_for_visibility()

    # Step 1: face obstacle zone (toward +x)
    turn_to(ANGLE_POS_X)

    while True:
        wait_for_visibility()
        x = enes100.x
        y = enes100.y
        debug("Obstacle loop: x={:.3f}, y={:.3f}".format(x, y))

        # Finished obstacle zone?
        if x >= X_END_OBS:
            debug("Obstacle zone complete (x>=2.5)")
            stop()
            return

        # Check for obstacle across front
        if is_front_blocked():
            debug("Obstacle detected in front — performing detour")

            # small backup
            drive_backward()
            sleep(0.3)
            stop()
            sleep(0.1)

            # choose detour direction based on y (not dl/dr)
            wait_for_visibility()
            y_now = enes100.y
            if y_now < 1.0:
                # bottom half → go UP (+Y)
                detour_sign = +1
                detour_angle = ANGLE_POS_Y
                debug("Detour UP (+Y), y={:.3f}".format(y_now))
            else:
                # top half → go DOWN (–Y)
                detour_sign = -1
                detour_angle = ANGLE_NEG_Y
                debug("Detour DOWN (-Y), y={:.3f}".format(y_now))

            # compute target y for detour and clamp
            target_y = y_now + detour_sign * DY_DETOUR
            if target_y < Y_MIN: target_y = Y_MIN
            if target_y > Y_MAX: target_y = Y_MAX

            # turn in y-direction and move
            turn_to(detour_angle)
            forward_to_y(target_y)

            # face obstacle side again
            turn_to(ANGLE_POS_X)
            # then continue main loop
            continue

        # No obstacle → drive forward a small step
        drive_forward()
        sleep(0.06)
        stop()

# ============================
# OPEN ZONE → LIMBO → GOAL
# ============================

X_LIMBO_ENTRY = 2.8
Y_LIMBO       = 1.30
X_GOAL        = 3.7

def limbo_and_goal_run():
    """
    After obstacle zone:
      - Move to x=2.8 facing +x.
      - Adjust y to 1.5.
      - Face +x and drive to x≈3.7 (goal).
    """
    debug("Entering LIMBO + GOAL RUN")
    wait_for_visibility()

    # 1) Make sure we face +x
    turn_to(ANGLE_POS_X)

    # 2) Move to x≈2.8
    while True:
        wait_for_visibility()
        x = enes100.x
        if x >= X_LIMBO_ENTRY:
            stop()
            break
        if is_front_blocked():
            debug("Unexpected obstacle before limbo entry; stopping")
            stop()
            break
        drive_forward()
        sleep(0.06)
        stop()

    # 3) Adjust to y=1.5
    go_to_y(Y_LIMBO)

    # 4) Face +x and go to x≈3.7
    turn_to(ANGLE_POS_X)
    while True:
        wait_for_visibility()
        x = enes100.x
        if x >= X_GOAL:
            stop()
            break
        if is_front_blocked():
            debug("Unexpected obstacle near goal; stopping")
            stop()
            break
        drive_forward()
        sleep(0.06)
        stop()

    debug("LIMBO + GOAL COMPLETE (x≈{:.2f}, y≈{:.2f})".format(enes100.x, enes100.y))
    stop()

# ============================
# WHOLE MISSION
# ============================

def run_mission():
    # 1) Mission zone
    run_mission_zone_only()

    # 2) Obstacle zone (straight across until x≈2.5, with simple detours)
    obstacle_zone_run()

    # 3) Limbo + final goal run
    limbo_and_goal_run()

    # You can add mission calls here if needed
    enes100.mission('WEIGHT', 'MEDIUM')
    enes100.mission('MATERIAL_TYPE', 'PLASTIC')
    debug("Mission calls sent: WEIGHT=MEDIUM, MATERIAL_TYPE=PLASTIC")

# ============================
# MAIN
# ============================

def main():
    global ENES_READY
    stop()
    enes100.begin(TEAM_NAME, TEAM_TYPE, ARUCO_ID, ROOM_NUM)
    ENES_READY = True
    enes100.print("Connected from main() – mission + obstacle zone")

    try:
        # quick sonar sanity check
        for _ in range(3):
            read_all_distances()
            sleep(0.3)

        run_mission()
        enes100.print("Mission run complete.")
    except Exception as e:
        msg = "ERROR in main: {}".format(repr(e))
        try:
            enes100.print(msg[:120])
        except:
            pass
    finally:
        stop()
        debug("main() exit, motors stopped.")

if __name__ == "__main__":
    main()
