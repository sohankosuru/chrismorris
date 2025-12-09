# main.py – ENES100 OTV navigation with calibrated angles
#
# Behavior:
#   1) From mission start (A or B), go to the opposite mission site (straight along mission line).
#   2) Move to TOP_LANE_Y (limbo row).
#   3) Navigate 2 obstacle columns with ultrasonics.
#   4) Fix Y to limbo row and drive to goal under limbo.
#
# All debug goes to WiFi Vision console via enes100.print.

from machine import Pin, PWM
from time import sleep
from math import pi
from enes100 import enes100
from hcsr04 import HCSR04

# ============================================================
# 0. CALIBRATED ANGLES (FROM YOUR MEASUREMENTS)
# ============================================================

# FRONT faces "up" along the mission line (B -> A):
ANGLE_POS_Y = 0.0        # theta ≈ 0 rad

# FRONT faces "down" along the mission line (A -> B):
ANGLE_NEG_Y = pi         # theta ≈ +π rad

# FRONT faces toward obstacle/goal side (across arena):
ANGLE_POS_X = -1.54      # theta ≈ -π/2

# FRONT faces back toward mission zone side:
ANGLE_NEG_X = 1.54       # theta ≈ +π/2

# ============================================================
# ENES100 CONFIG
# ============================================================

TEAM_NAME  = "Team Chris Morris AWOG"
TEAM_TYPE  = "MATERIAL"
ARUCO_ID   = 7
ROOM_NUM   = 1120

ENES_READY = False

def debug(msg: str):
    """Send debug text to WiFi vision console."""
    global ENES_READY
    if not ENES_READY:
        return
    try:
        enes100.print(msg[:120])
    except Exception:
        pass

# ============================================================
# MOTOR CONFIG – YOUR PINS
# ============================================================

FREQ = 5000
MAX_DUTY_10BIT   = 1023
DEFAULT_SPEED    = 700
MISSION_SPEED    = 600
OBSTACLE_SPEED   = 650
LIMBO_SPEED      = 600
ANGLE_TURN_SPEED = 500

# Your working motor pins (from differential_drive_keys.py)
pwm0 = PWM(Pin(26), freq=FREQ)  # motor A input 1
pwm1 = PWM(Pin(27), freq=FREQ)  # motor A input 2
pwm2 = PWM(Pin(33), freq=FREQ)  # motor B input 1
pwm3 = PWM(Pin(32), freq=FREQ)  # motor B input 2

def set_pwm_duty(pwm, duty_10bit):
    if duty_10bit < 0:
        duty_10bit = 0
    if duty_10bit > MAX_DUTY_10BIT:
        duty_10bit = MAX_DUTY_10BIT
    try:
        pwm.duty(int(duty_10bit))
    except AttributeError:
        scale = 65535 // MAX_DUTY_10BIT
        pwm.duty_u16(int(duty_10bit * scale))

def motor_a_forward(speed=DEFAULT_SPEED):
    set_pwm_duty(pwm0, speed)
    set_pwm_duty(pwm1, 0)

def motor_a_reverse(speed=DEFAULT_SPEED):
    set_pwm_duty(pwm0, 0)
    set_pwm_duty(pwm1, speed)

def motor_b_forward(speed=DEFAULT_SPEED):
    set_pwm_duty(pwm2, speed)
    set_pwm_duty(pwm3, 0)

def motor_b_reverse(speed=DEFAULT_SPEED):
    set_pwm_duty(pwm2, 0)
    set_pwm_duty(pwm3, speed)

def motor_a_stop():
    set_pwm_duty(pwm0, 0)
    set_pwm_duty(pwm1, 0)

def motor_b_stop():
    set_pwm_duty(pwm2, 0)
    set_pwm_duty(pwm3, 0)

def stop_all():
    motor_a_stop()
    motor_b_stop()

# IMPORTANT WIRING FACT:
#   - go_forward()  → robot moves BACKWARD physically
#   - go_reverse()  → robot moves FORWARD physically
def go_forward(speed=DEFAULT_SPEED):
    motor_a_forward(speed)
    motor_b_forward(speed)
    debug("go_forward (electrical)")

def go_reverse(speed=DEFAULT_SPEED):
    motor_a_reverse(speed)
    motor_b_reverse(speed)
    debug("go_reverse (electrical)")

# Turns are correct relative to FRONT
def turn_left(speed=DEFAULT_SPEED):
    motor_a_reverse(speed)  # left wheel backward
    motor_b_forward(speed)  # right wheel forward
    debug("TURN LEFT")

def turn_right(speed=DEFAULT_SPEED):
    motor_a_forward(speed)
    motor_b_reverse(speed)
    debug("TURN RIGHT")

def stop():
    stop_all()
    debug("STOP")

# Physical wrappers (relative to FRONT of OTV)
def drive_forward(speed=DEFAULT_SPEED):
    # FRONT-first is "reverse" electrically due to wiring
    go_reverse(speed)

def drive_backward(speed=DEFAULT_SPEED):
    go_forward(speed)

def rotate_left(speed=DEFAULT_SPEED):
    turn_left(speed)

def rotate_right(speed=DEFAULT_SPEED):
    turn_right(speed)

# ============================================================
# ULTRASONIC SENSORS
# ============================================================

FRONT_TRIG_PIN = 4
FRONT_ECHO_PIN = 23

LEFT_TRIG_PIN  = 2
LEFT_ECHO_PIN  = 18

RIGHT_TRIG_PIN = 0
RIGHT_ECHO_PIN = 25

front_sonar = HCSR04(FRONT_TRIG_PIN, FRONT_ECHO_PIN)
left_sonar  = HCSR04(LEFT_TRIG_PIN,  LEFT_ECHO_PIN)
right_sonar = HCSR04(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN)

SAFE_FRONT_CM = 20.0   # stop/avoid if closer than this

def safe_distance(sensor):
    try:
        return sensor.distance_cm()
    except OSError:
        return None

def read_all_distances():
    df = safe_distance(front_sonar)
    dl = safe_distance(left_sonar)
    dr = safe_distance(right_sonar)
    if df is None: df = 999
    if dl is None: dl = 999
    if dr is None: dr = 999
    debug("Sonar F:{:.1f} L:{:.1f} R:{:.1f} cm".format(df, dl, dr))
    return df, dl, dr

# ============================================================
# ARENA CONSTANTS
# ============================================================

ARENA_Y_MAX        = 2.0
TOP_LANE_Y         = 1.5      # limbo row
OPEN_ZONE_START_X  = 2.8
GOAL_CENTER_X      = 3.7

COL1_X = 1.5
COL2_X = 2.3
PASS_MARGIN = 0.12

COARSE_POS_TOLERANCE = 0.15   # 15 cm band for coarse targets
Y_DETOUR_TOL         = 0.03
DY_DETOUR            = 0.35   # y shift when going around obstacle

THETA_THRESHOLD = 0.10        # ~6°

# ============================================================
# HEADING & VISION HELPERS
# ============================================================

def normalize_angle(a):
    while a > pi:
        a -= 2 * pi
    while a < -pi:
        a += 2 * pi
    return a

def get_theta():
    th = enes100.theta
    if th == -1:
        return -1
    return normalize_angle(th)

def wait_for_visibility():
    while not enes100.is_visible:
        debug("Waiting for visibility...")
        sleep(0.1)
    debug("Visible: x={:.3f}, y={:.3f}, theta={:.3f}".format(
        enes100.x, enes100.y, get_theta()
    ))

def set_angle(target):
    """Rotate in place until heading ≈ target."""
    target = normalize_angle(target)
    debug("set_angle → target={:.3f}".format(target))
    wait_for_visibility()

    while True:
        th = get_theta()
        if th == -1:
            debug("theta=-1; waiting...")
            stop()
            sleep(0.1)
            continue

        err = normalize_angle(target - th)
        debug("theta={:.3f}, err={:.3f}".format(th, err))

        if abs(err) < THETA_THRESHOLD:
            debug("Angle aligned.")
            break

        if err > 0:
            rotate_left(ANGLE_TURN_SPEED)
        else:
            rotate_right(ANGLE_TURN_SPEED)

        sleep(0.02)

    stop()

# ============================================================
# STRAIGHT MOVES (NO CONTINUOUS WIGGLE)
# ============================================================

def straight_to_y_coarse(target_y, speed=MISSION_SPEED):
    """
    Move along mission line (between A and B) with:
      - ONE orientation (ANGLE_POS_Y or ANGLE_NEG_Y)
      - Straight segments, no constant turning correction.
    """
    wait_for_visibility()
    y = enes100.y
    debug("straight_to_y_coarse: start y={:.3f}, target_y={:.3f}".format(y, target_y))

    if y < target_y:
        # need to go "up" along mission line
        set_angle(ANGLE_POS_Y)
    else:
        # need to go "down"
        set_angle(ANGLE_NEG_Y)

    while True:
        if not enes100.is_visible:
            debug("Lost vis in straight_to_y; waiting")
            stop()
            wait_for_visibility()
            # re-orient once
            y = enes100.y
            if y < target_y:
                set_angle(ANGLE_POS_Y)
            else:
                set_angle(ANGLE_NEG_Y)

        y = enes100.y
        dist = abs(y - target_y)
        debug("straight_to_y: y={:.3f}, dist={:.3f}".format(y, dist))
        if dist <= COARSE_POS_TOLERANCE:
            debug("Reached coarse y")
            break

        df, _, _ = read_all_distances()
        if df < SAFE_FRONT_CM:
            debug("Front too close in straight_to_y; stopping")
            break

        drive_forward(speed)
        sleep(0.07)
        stop()

    stop()

def straight_to_x_coarse(target_x, speed=MISSION_SPEED):
    """
    Move across arena (toward/away from obstacles) with:
      - ONE orientation (ANGLE_POS_X or ANGLE_NEG_X)
      - Straight segments only.
    """
    wait_for_visibility()
    x = enes100.x
    debug("straight_to_x_coarse: start x={:.3f}, target_x={:.3f}".format(x, target_x))

    if x < target_x:
        # move toward obstacle/goal side
        set_angle(ANGLE_POS_X)
    else:
        # move back toward mission side
        set_angle(ANGLE_NEG_X)

    while True:
        if not enes100.is_visible:
            debug("Lost vis in straight_to_x; waiting")
            stop()
            wait_for_visibility()
            x = enes100.x
            if x < target_x:
                set_angle(ANGLE_POS_X)
            else:
                set_angle(ANGLE_NEG_X)

        x = enes100.x
        dist = abs(x - target_x)
        debug("straight_to_x: x={:.3f}, dist={:.3f}".format(x, dist))
        if dist <= COARSE_POS_TOLERANCE:
            debug("Reached coarse x")
            break

        df, _, _ = read_all_distances()
        if df < SAFE_FRONT_CM:
            debug("Front too close in straight_to_x; stopping")
            break

        drive_forward(speed)
        sleep(0.07)
        stop()

    stop()

# ============================================================
# COLUMN NAVIGATION (2 COLUMNS)
# ============================================================

def navigate_column(x_col):
    """
    Handle a single obstacle column at x_col:
      1) Approach to x_col - PASS_MARGIN along +X (ANGLE_POS_X).
      2) If front clear, pass straight.
      3) If blocked, detour in y based on side sonars, then pass.
    """
    debug("navigate_column: x_col={:.2f}".format(x_col))
    wait_for_visibility()

    # Approach just before column
    set_angle(ANGLE_POS_X)
    while True:
        if not enes100.is_visible:
            debug("Lost vis approaching column; waiting")
            stop()
            wait_for_visibility()
            set_angle(ANGLE_POS_X)

        x = enes100.x
        if x >= x_col - PASS_MARGIN:
            debug("Reached pre-column x={:.3f}".format(x))
            break

        df, _, _ = read_all_distances()
        if df < SAFE_FRONT_CM:
            debug("Saw obstacle early while approaching column")
            break

        drive_forward(OBSTACLE_SPEED)
        sleep(0.07)
        stop()

    stop()

    df, dl, dr = read_all_distances()

    # Case 1: front clear → straight pass
    if df > SAFE_FRONT_CM:
        debug("Column clear in this row → straight pass")
        set_angle(ANGLE_POS_X)
        while True:
            if not enes100.is_visible:
                debug("Lost vis during straight pass; waiting")
                stop()
                wait_for_visibility()
                set_angle(ANGLE_POS_X)

            x = enes100.x
            if x >= x_col + PASS_MARGIN:
                debug("Passed column x={:.3f}".format(x))
                break

            df2, _, _ = read_all_distances()
            if df2 < SAFE_FRONT_CM:
                debug("Unexpected obstacle while passing column; stopping")
                break

            drive_forward(OBSTACLE_SPEED)
            sleep(0.07)
            stop()

        stop()
        return

    # Case 2: obstacle ahead → detour
    debug("Obstacle at column {}, need detour".format(x_col))
    stop()
    sleep(0.1)

    # Small backup
    drive_backward(OBSTACLE_SPEED)
    sleep(0.3)
    stop()
    sleep(0.1)

    if dl is None: dl = 999
    if dr is None: dr = 999

    # Choose detour direction in y
    if dl > dr:
        detour_dir = +1
        detour_angle = ANGLE_POS_Y
        debug("Detour UP (+Y)")
    else:
        detour_dir = -1
        detour_angle = ANGLE_NEG_Y
        debug("Detour DOWN (-Y)")

    # Turn toward detour direction
    set_angle(detour_angle)
    wait_for_visibility()
    start_y = enes100.y
    target_y = start_y + detour_dir * DY_DETOUR
    if target_y < 0.2: target_y = 0.2
    if target_y > 1.8: target_y = 1.8

    debug("Detour y: from {:.3f} to {:.3f}".format(start_y, target_y))

    while True:
        if not enes100.is_visible:
            debug("Lost vis in detour y; waiting")
            stop()
            wait_for_visibility()
            set_angle(detour_angle)

        y = enes100.y
        disty = abs(y - target_y)
        debug("Detour y: y={:.3f}, dist={:.3f}".format(y, disty))
        if disty <= Y_DETOUR_TOL:
            debug("Finished detour y")
            break

        df2, _, _ = read_all_distances()
        if df2 < SAFE_FRONT_CM:
            debug("Front too close during detour y; stopping")
            break

        drive_forward(OBSTACLE_SPEED)
        sleep(0.07)
        stop()

    stop()

    # Turn back toward +X side and pass column
    set_angle(ANGLE_POS_X)
    debug("Passing column after detour")
    while True:
        if not enes100.is_visible:
            debug("Lost vis during column pass; waiting")
            stop()
            wait_for_visibility()
            set_angle(ANGLE_POS_X)

        x = enes100.x
        if x >= x_col + PASS_MARGIN:
            debug("Passed column x={:.3f} after detour".format(x))
            break

        df2, _, _ = read_all_distances()
        if df2 < SAFE_FRONT_CM:
            debug("Still blocked after detour; stopping")
            break

        drive_forward(OBSTACLE_SPEED)
        sleep(0.07)
        stop()

    stop()

# ============================================================
# LIMBO + GOAL
# ============================================================

def limbo_and_goal_run():
    debug("Fixing Y to TOP_LANE_Y={:.2f}".format(TOP_LANE_Y))
    straight_to_y_coarse(TOP_LANE_Y, speed=MISSION_SPEED)

    debug("Final run to goal x={:.2f}".format(GOAL_CENTER_X))
    straight_to_x_coarse(GOAL_CENTER_X, speed=LIMBO_SPEED)
    stop()
    debug("In goal zone near limbo.")

# ============================================================
# MISSION LOGIC
# ============================================================

def run_mission():
    """
    1) From mission site A/B, go to the opposite site (mirror across y=1.0).
    2) Move to TOP_LANE_Y.
    3) Navigate column at x=1.5.
    4) Navigate column at x=2.3.
    5) Align to limbo row and go to goal.
    """
    wait_for_visibility()
    start_x = enes100.x
    start_y = enes100.y
    debug("Start: x={:.3f}, y={:.3f}".format(start_x, start_y))

    mission_x = start_x
    mission_y = ARENA_Y_MAX - start_y
    debug("Other mission site: x={:.3f}, y={:.3f}".format(mission_x, mission_y))

    # Phase 1: straight along mission line A <-> B
    debug("Phase 1: straight Y to other mission site")
    straight_to_y_coarse(mission_y, speed=MISSION_SPEED)

    # Phase 2: go to limbo row
    debug("Phase 2: straight Y to TOP_LANE_Y")
    straight_to_y_coarse(TOP_LANE_Y, speed=MISSION_SPEED)

    # Phase 3: first column
    debug("Phase 3: navigate column x={:.2f}".format(COL1_X))
    navigate_column(COL1_X)

    # Phase 4: second column
    debug("Phase 4: navigate column x={:.2f}".format(COL2_X))
    navigate_column(COL2_X)

    # Phase 5: limbo + goal
    debug("Phase 5: limbo + goal")
    limbo_and_goal_run()

    # MATERIAL mission calls (placeholder values)
    enes100.mission('WEIGHT', 'MEDIUM')
    enes100.mission('MATERIAL_TYPE', 'PLASTIC')
    debug("Mission calls sent: WEIGHT=MEDIUM, MATERIAL_TYPE=PLASTIC")

# ============================================================
# MAIN
# ============================================================

def main():
    global ENES_READY
    stop_all()
    enes100.begin(TEAM_NAME, TEAM_TYPE, ARUCO_ID, ROOM_NUM)
    ENES_READY = True
    enes100.print("Connected from main() with calibrated angles")

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
        except Exception:
            pass
    finally:
        stop_all()
        debug("main() exit, motors stopped.")

if __name__ == "__main__":
    main()
