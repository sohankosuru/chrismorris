# main.py – ENES100 OTV navigation (MATERIAL mission)
# Behavior:
#   - Mission zone A <-> B: straight moves only (no wiggle)
#   - Obstacle zone: handle columns at x=1.5 and x=2.3, column-by-column
#   - Uses front + left + right ultrasonics
#   - After all columns, fix Y to limbo row, then straight to goal
#
# YOU MUST SET: HEADING_OFFSET (see comment below)

from machine import Pin, PWM
from time import sleep
from math import pi
from enes100 import enes100
from hcsr04 import HCSR04

# ============================================================
# ENES100 CONFIG
# ============================================================

TEAM_NAME  = "Team Chris Morris AWOG"
TEAM_TYPE  = "MATERIAL"
ARUCO_ID   = 7
ROOM_NUM   = 1116

ENES_READY = False  # becomes True after begin()

def debug(msg: str):
    """Send debug text to WiFi vision console (no Thonny print)."""
    global ENES_READY
    if ENES_READY:
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

def motor_a_stop():
    set_pwm_duty(pwm0, 0)
    set_pwm_duty(pwm1, 0)

def motor_b_forward(speed=DEFAULT_SPEED):
    set_pwm_duty(pwm2, speed)
    set_pwm_duty(pwm3, 0)

def motor_b_reverse(speed=DEFAULT_SPEED):
    set_pwm_duty(pwm2, 0)
    set_pwm_duty(pwm3, speed)

def motor_b_stop():
    set_pwm_duty(pwm2, 0)
    set_pwm_duty(pwm3, 0)

def stop_all():
    motor_a_stop()
    motor_b_stop()

def go_forward(speed=DEFAULT_SPEED):
    motor_a_forward(speed)
    motor_b_forward(speed)
    debug("FORWARD (speed={})".format(speed))

def go_reverse(speed=DEFAULT_SPEED):
    motor_a_reverse(speed)
    motor_b_reverse(speed)
    debug("REVERSE (speed={})".format(speed))

def turn_left(speed=DEFAULT_SPEED):
    motor_a_reverse(speed)
    motor_b_forward(speed)
    debug("TURN LEFT (speed={})".format(speed))

def turn_right(speed=DEFAULT_SPEED):
    motor_a_forward(speed)
    motor_b_reverse(speed)
    debug("TURN RIGHT (speed={})".format(speed))

def stop():
    stop_all()
    debug("STOP")

# ============================================================
# ULTRASONIC SENSORS – front, left, right
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

SAFE_FRONT_CM = 20.0   # stop / avoid if closer than this
# side sensors mostly for choosing detour direction
SAFE_SIDE_CM  = 5.0    

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
    debug("Distances  F:{:.1f}  L:{:.1f}  R:{:.1f} cm".format(df, dl, dr))
    return df, dl, dr

# ============================================================
# ARENA CONSTANTS (meters)
# ============================================================

ARENA_Y_MAX        = 2.0
TOP_LANE_Y         = 1.5      # limbo row
OPEN_ZONE_START_X  = 2.8
GOAL_CENTER_X      = 3.7

COL1_X = 1.5                 # first obstacle column
COL2_X = 2.3                 # second obstacle column
PASS_MARGIN = 0.12           # how far before/after each column we treat as "passed"

X_TOLERANCE          = 0.03
Y_TOLERANCE          = 0.03
COARSE_POS_TOLERANCE = 0.15  # 150 mm from target is OK

THETA_THRESHOLD = 0.10       # ~6 degrees

# ============================================================
# HEADING OFFSET
# ============================================================
# IMPORTANT:
#   Put here the value of theta from the vision system when the FRONT
#   of the OTV faces straight toward the obstacle zone (downfield).
#   Example: if the console says theta = -1.57 when facing obstacles,
#   then set HEADING_OFFSET = -1.57
HEADING_OFFSET = 0.0  # <<< CHANGE THIS AFTER YOUR TEST

def normalize_angle(a):
    while a > pi:
        a -= 2 * pi
    while a < -pi:
        a += 2 * pi
    return a

def get_robot_theta():
    """Corrected heading: 0 means facing downfield toward obstacles (+x)."""
    th = enes100.theta
    if th == -1:
        return -1
    return normalize_angle(th - HEADING_OFFSET)

def wait_for_visibility():
    while not enes100.is_visible:
        debug("Waiting for visibility...")
        sleep(0.1)
    debug("Visible: x={:.3f}, y={:.3f}, theta_corr={:.3f}".format(
        enes100.x, enes100.y, get_robot_theta()))

# ============================================================
# HEADING CONTROL
# ============================================================

def set_angle(target):
    """Spin in place until corrected theta ≈ target (rad)."""
    target = normalize_angle(target)
    debug("set_angle → target={:.3f}".format(target))
    wait_for_visibility()

    while True:
        theta = get_robot_theta()
        if theta == -1:
            debug("Theta not visible; waiting...")
            stop()
            sleep(0.1)
            continue

        error = normalize_angle(target - theta)
        debug("set_angle: theta={:.3f}, error={:.3f}".format(theta, error))

        if abs(error) < THETA_THRESHOLD:
            debug("Angle aligned.")
            break

        if error > 0:
            turn_left(ANGLE_TURN_SPEED)
        else:
            turn_right(ANGLE_TURN_SPEED)

        sleep(0.02)

    stop()

# ============================================================
# STRAIGHT MOVES (NO WIGGLE)
# ============================================================

def straight_to_y_coarse(target_y, speed=MISSION_SPEED):
    """
    Orient ONCE, then move along ±y until within 0.15 m of target_y.
    No continuous heading correction → no wiggle.
    """
    wait_for_visibility()
    y = enes100.y
    debug("straight_to_y_coarse: start y={:.3f}, target_y={:.3f}".format(y, target_y))

    if y < target_y:
        # need to go +Y (up)
        set_angle(pi/2)
    else:
        # need to go -Y (down)
        set_angle(-pi/2)

    while True:
        if not enes100.is_visible:
            debug("Lost visibility in straight_to_y; waiting")
            stop()
            wait_for_visibility()
        y = enes100.y
        dist = abs(y - target_y)
        debug("straight_to_y: y={:.3f}, dist={:.3f}".format(y, dist))

        if dist <= COARSE_POS_TOLERANCE:
            debug("Reached coarse y within {:.3f} m".format(COARSE_POS_TOLERANCE))
            break

        df, _, _ = read_all_distances()
        if df < SAFE_FRONT_CM:
            debug("Front too close during straight_to_y, stopping")
            break

        go_forward(speed)
        sleep(0.07)
        stop()

    stop()

def straight_to_x_coarse(target_x, speed=MISSION_SPEED):
    """
    Orient ONCE, then move along ±x until within 0.15 m of target_x.
    """
    wait_for_visibility()
    x = enes100.x
    debug("straight_to_x_coarse: start x={:.3f}, target_x={:.3f}".format(x, target_x))

    if x < target_x:
        # move toward goal (+x)
        set_angle(0.0)
    else:
        # move back toward start (-x)
        set_angle(pi)

    while True:
        if not enes100.is_visible:
            debug("Lost visibility in straight_to_x; waiting")
            stop()
            wait_for_visibility()
        x = enes100.x
        dist = abs(x - target_x)
        debug("straight_to_x: x={:.3f}, dist={:.3f}".format(x, dist))

        if dist <= COARSE_POS_TOLERANCE:
            debug("Reached coarse x within {:.3f} m".format(COARSE_POS_TOLERANCE))
            break

        df, _, _ = read_all_distances()
        if df < SAFE_FRONT_CM:
            debug("Front too close during straight_to_x, stopping")
            break

        go_forward(speed)
        sleep(0.07)
        stop()

    stop()

# ============================================================
# COLUMN-BASED OBSTACLE NAVIGATION
# ============================================================

DY_DETOUR = 0.35   # how far we shift in y when going around an obstacle
Y_DETOUR_TOL = 0.03

def navigate_column(x_col):
    """
    Handle one obstacle column at x_col:
      1) Approach x_col - PASS_MARGIN along +x.
      2) If front clear: pass straight to x_col + PASS_MARGIN.
      3) If front blocked: detour in y using left/right sonars, then pass.
    We do NOT care about staying exactly at TOP_LANE_Y here; we fix that later.
    """
    debug("navigate_column: x_col={:.2f}".format(x_col))
    wait_for_visibility()

    # --- Approach just before column ---
    set_angle(0.0)  # face +x
    while True:
        if not enes100.is_visible:
            debug("Lost visibility approaching column; waiting")
            stop()
            wait_for_visibility()
            set_angle(0.0)

        x = enes100.x
        if x >= x_col - PASS_MARGIN:
            debug("Reached pre-column x ≈ {:.3f}".format(x))
            break

        df, _, _ = read_all_distances()
        if df < SAFE_FRONT_CM:
            debug("Saw obstacle early while approaching column")
            break

        go_forward(OBSTACLE_SPEED)
        sleep(0.07)
        stop()

    stop()

    # Check front at column
    df, dl, dr = read_all_distances()

    # --- Case 1: front clear → just pass the column straight ---
    if df > SAFE_FRONT_CM:
        debug("Column clear in this row → straight pass")
        set_angle(0.0)
        while True:
            if not enes100.is_visible:
                debug("Lost visibility during pass; waiting")
                stop()
                wait_for_visibility()
                set_angle(0.0)

            x = enes100.x
            if x >= x_col + PASS_MARGIN:
                debug("Passed column x_col={:.2f}, now x={:.3f}".format(x_col, x))
                break

            df2, _, _ = read_all_distances()
            if df2 < SAFE_FRONT_CM:
                debug("Unexpected obstacle while passing column, stopping")
                break

            go_forward(OBSTACLE_SPEED)
            sleep(0.07)
            stop()

        stop()
        return

    # --- Case 2: front blocked → detour in y ---
    debug("Obstacle at column x={:.2f}, need detour".format(x_col))
    stop()
    sleep(0.1)

    # small backup
    go_reverse(OBSTACLE_SPEED)
    sleep(0.3)
    stop()
    sleep(0.1)

    # Choose detour direction using left/right distances
    if dl is None: dl = 999
    if dr is None: dr = 999

    if dl > dr:
        # more open to +y (assuming left sensor points +y)
        detour_dir = +1
        debug("Detour: UP (+y)")
        target_angle = pi/2
    else:
        detour_dir = -1
        debug("Detour: DOWN (-y)")
        target_angle = -pi/2

    # Turn toward detour direction
    set_angle(target_angle)

    # Move DY_DETOUR in y
    wait_for_visibility()
    start_y = enes100.y
    target_y = start_y + detour_dir * DY_DETOUR
    # clamp to arena (just in case)
    if target_y < 0.2:  target_y = 0.2
    if target_y > 1.8:  target_y = 1.8
    debug("Detour move in y: from {:.3f} to {:.3f}".format(start_y, target_y))

    while True:
        if not enes100.is_visible:
            debug("Lost visibility in detour move; waiting")
            stop()
            wait_for_visibility()
            set_angle(target_angle)

        y = enes100.y
        dist_y = abs(y - target_y)
        debug("Detour y: y={:.3f}, dist={:.3f}".format(y, dist_y))
        if dist_y <= Y_DETOUR_TOL:
            debug("Finished detour y shift.")
            break

        df2, _, _ = read_all_distances()
        if df2 < SAFE_FRONT_CM:
            debug("Front too close during detour, stopping")
            break

        go_forward(OBSTACLE_SPEED)
        sleep(0.07)
        stop()

    stop()

    # Turn back toward +x
    set_angle(0.0)

    # Pass the column now from this new row
    debug("Passing column from detoured row.")
    while True:
        if not enes100.is_visible:
            debug("Lost visibility during detour pass; waiting")
            stop()
            wait_for_visibility()
            set_angle(0.0)

        x = enes100.x
        if x >= x_col + PASS_MARGIN:
            debug("Passed column after detour. x={:.3f}".format(x))
            break

        df2, _, _ = read_all_distances()
        if df2 < SAFE_FRONT_CM:
            debug("Still blocked even after detour; stopping")
            break

        go_forward(OBSTACLE_SPEED)
        sleep(0.07)
        stop()

    stop()

# ============================================================
# LIMBO + GOAL
# ============================================================

def limbo_and_goal_run():
    """
    After all columns are cleared:
      1) Fix Y to TOP_LANE_Y.
      2) Move in X to GOAL_CENTER_X under limbo.
    """
    debug("Fixing Y back to TOP_LANE_Y={:.2f} in open zone".format(TOP_LANE_Y))
    straight_to_y_coarse(TOP_LANE_Y, speed=MISSION_SPEED)

    debug("Final straight run to GOAL x={:.2f}".format(GOAL_CENTER_X))
    straight_to_x_coarse(GOAL_CENTER_X, speed=LIMBO_SPEED)

    stop()
    debug("Stopped in goal zone near limbo.")

# ============================================================
# MISSION LOGIC
# ============================================================

def run_mission():
    """
    Mission sequence:
      1) Center on blue-line x in mission zone.
      2) Move straight in y to other mission site.
      3) Recenter on same x, then to TOP_LANE_Y.
      4) Navigate column at x=1.5.
      5) Navigate column at x=2.3.
      6) Fix Y in open zone, go to goal.
    """
    wait_for_visibility()
    start_x = enes100.x
    start_y = enes100.y
    debug("Start: x={:.3f}, y={:.3f}".format(start_x, start_y))

    # Mission site mirrored across y=1.0
    mission_x = start_x
    mission_y = ARENA_Y_MAX - start_y
    debug("Mission site: x={:.3f}, y={:.3f}".format(mission_x, mission_y))

    # Phase 1: center on mission column (blue line)
    debug("Phase 1: centering on blue-line x={:.3f}".format(start_x))
    straight_to_x_coarse(start_x, speed=MISSION_SPEED)

    # Phase 2: straight Y to other mission site (no wiggle)
    debug("Phase 2: straight Y from start to mission site (A <-> B)")
    straight_to_y_coarse(mission_y, speed=MISSION_SPEED)

    # Phase 3: re-center on same x, then go to TOP_LANE_Y
    debug("Phase 3: re-centering on x={:.3f}, then TOP_LANE_Y={:.2f}".format(mission_x, TOP_LANE_Y))
    straight_to_x_coarse(mission_x, speed=MISSION_SPEED)
    straight_to_y_coarse(TOP_LANE_Y, speed=MISSION_SPEED)

    # Phase 4: navigate first column
    debug("Phase 4: navigate first column at x={:.2f}".format(COL1_X))
    navigate_column(COL1_X)

    # Phase 5: navigate second column
    debug("Phase 5: navigate second column at x={:.2f}".format(COL2_X))
    navigate_column(COL2_X)

    # Phase 6: limbo + goal
    debug("Phase 6: limbo + goal run")
    limbo_and_goal_run()

    # MATERIAL mission calls (placeholder)
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
    enes100.print("Connected from main()")

    try:
        # quick sonar check
        for _ in range(3):
            read_all_distances()
            sleep(0.3)

        run_mission()
        enes100.print("Mission completed.")
    except Exception as e:
        msg = "ERROR in main: {}".format(repr(e))
        try:
            enes100.print(msg[:120])
        except Exception:
            pass
    finally:
        stop_all()
        debug("main() exiting, motors stopped.")

if __name__ == "__main__":
    main()
