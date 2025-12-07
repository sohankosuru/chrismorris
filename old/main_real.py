# main.py – ENES100 OTV navigation for MATERIAL mission
# Uses:
#   - Motors on pins: 26, 27, 33, 32  (your working config)
#   - Ultrasonics on pins: 4, 23, 2, 18, 0, 25 (your working config)
#   - ENES100 vision system
#   - Limbo/top-lane navigation

from machine import Pin, PWM
from time import sleep
from math import pi
from enes100 import enes100
from hcsr04 import HCSR04

# ============================================================
# ENES100 CONFIG
# ============================================================

TEAM_NAME  = "Team Chris Morris AWOG"
TEAM_TYPE  = "MATERIAL"   # mission type
ARUCO_ID   = 7
ROOM_NUM   = 1116         # 1116 or 1120

# Flag so debug() knows when it's safe to call enes100.print
ENES_READY = False

def debug(msg: str):
    """Send debug text to WiFi vision console only."""
    global ENES_READY
    if ENES_READY:
        try:
            # truncate so we don't spam huge lines
            enes100.print(msg[:120])
        except Exception:
            # if WiFi print fails, just ignore
            pass

# ============================================================
# MOTOR CONFIG – YOUR WORKING PINS
# ============================================================

FREQ = 5000
MAX_DUTY_10BIT = 1023
DEFAULT_SPEED = 700     # 0..1023
ANGLE_TURN_SPEED = 500  # a bit slower for turning
PRINT_FEEDBACK = True

# PWM objects – same pins as your working differential_drive_keys.py
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

# --- motor primitives ---
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

# --- high-level drive actions ---
def go_forward(speed=DEFAULT_SPEED):
    motor_a_forward(speed)
    motor_b_forward(speed)
    if PRINT_FEEDBACK:
        debug("FORWARD (speed={})".format(speed))

def go_reverse(speed=DEFAULT_SPEED):
    motor_a_reverse(speed)
    motor_b_reverse(speed)
    if PRINT_FEEDBACK:
        debug("REVERSE (speed={})".format(speed))

def turn_left(speed=DEFAULT_SPEED):
    # same behavior as your working script: A reverse, B forward
    motor_a_reverse(speed)
    motor_b_forward(speed)
    if PRINT_FEEDBACK:
        debug("TURN LEFT (speed={})".format(speed))

def turn_right(speed=DEFAULT_SPEED):
    motor_a_forward(speed)
    motor_b_reverse(speed)
    if PRINT_FEEDBACK:
        debug("TURN RIGHT (speed={})".format(speed))

def stop():
    stop_all()
    if PRINT_FEEDBACK:
        debug("STOP")

# ============================================================
# ULTRASONIC SENSORS – 3x HCSR04 (front, left, right)
# Using pins from your working test script
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

SAFE_FRONT_CM = 20.0   # stop/avoid if front < 20 cm
SAFE_SIDE_CM  = 15.0   # steer away if side < 15 cm

# ============================================================
# ARENA CONSTANTS (meters)
# ============================================================

ARENA_Y_MAX        = 2.0
OPEN_ZONE_START_X  = 2.8    # start of open zone
GOAL_CENTER_X      = 3.7    # approx center of goal zone
TOP_LANE_Y         = 1.5    # limbo row

# Nav tuning
X_TOLERANCE     = 0.03
Y_TOLERANCE     = 0.03
THETA_THRESHOLD = 0.05  # radians (~3°)
TURN_KP         = 1.0   # not heavily used now, but kept

# You said: facing obstacle zone gives theta ≈ -pi/2
# We want that to be "0" in our control frame → add +pi/2
HEADING_OFFSET = pi / 2

# ============================================================
# HELPER FUNCTIONS
# ============================================================

def constrain(val, mn, mx):
    return min(mx, max(mn, val))

def normalize_angle(angle):
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

def get_robot_theta():
    """Return theta with optional constant offset."""
    th = enes100.theta
    if th == -1:
        return -1
    return normalize_angle(th + HEADING_OFFSET)

def wait_for_visibility():
    """Block until marker visible."""
    while not enes100.is_visible:
        debug("Waiting for visibility...")
        sleep(0.1)
    debug("Marker visible. x={:.3f}, y={:.3f}, theta_eff={:.3f}".format(
        enes100.x, enes100.y, get_robot_theta()))

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
# OBSTACLE-AWARE FORWARD
# ============================================================

def forward_with_obstacle_check(speed=DEFAULT_SPEED):
    """
    Use go_forward / go_reverse / turn_left / turn_right / stop()
    with 3 ultrasonics to avoid obstacles.
    """
    df, dl, dr = read_all_distances()

    # Hard avoid if front is blocked
    if df < SAFE_FRONT_CM:
        debug("Front obstacle – backing up and turning.")
        stop()
        sleep(0.1)

        go_reverse(speed)
        sleep(0.4)
        stop()
        sleep(0.1)

        # Turn toward more open side
        if dl > dr + 5:
            debug("More space on LEFT → turn left.")
            turn_left(speed)
        elif dr > dl + 5:
            debug("More space on RIGHT → turn right.")
            turn_right(speed)
        else:
            debug("Sides similar → default turn right.")
            turn_right(speed)

        sleep(0.5)
        stop()
        sleep(0.1)
        return

    # Side “hugging” – steer away if one side is too close
    left_close  = dl < SAFE_SIDE_CM
    right_close = dr < SAFE_SIDE_CM

    if left_close and not right_close:
        debug("Wall LEFT → veer right.")
        motor_a_reverse(int(speed * 0.3))  # left wheel slow/reverse
        motor_b_forward(speed)             # right wheel fast
    elif right_close and not left_close:
        debug("Wall RIGHT → veer left.")
        motor_a_forward(speed)
        motor_b_reverse(int(speed * 0.3))
    else:
        go_forward(speed)

# ============================================================
# HEADING CONTROL: set_angle
# ============================================================

def set_angle(target):
    """
    Turn in place until robot theta ≈ target (radians).
    Uses sign of error to decide turn direction.
    """
    target = normalize_angle(target)
    debug("set_angle → target = {:.3f}".format(target))
    wait_for_visibility()

    while True:
        theta = get_robot_theta()
        if theta == -1:
            debug("Theta not visible; waiting...")
            stop()
            sleep(0.1)
            continue

        error = normalize_angle(target - theta)
        debug("set_angle(): theta={:.3f}, target={:.3f}, error={:.3f}".format(
            theta, target, error))

        if abs(error) < THETA_THRESHOLD:
            debug("Angle aligned: |error|={:.3f} < threshold".format(abs(error)))
            break

        if error > 0:
            debug("Cmd: CCW turn (LEFT)")
            turn_left(ANGLE_TURN_SPEED)
        else:
            debug("Cmd: CW turn (RIGHT)")
            turn_right(ANGLE_TURN_SPEED)

        sleep(0.02)

    stop()
    debug("Reached target angle.")

# ============================================================
# POSITION CONTROL: drive_to(x, y)
# ============================================================

def drive_to(target_x, target_y, speed=DEFAULT_SPEED):
    """
    Move in an 'L' shape:
      1) Adjust x to target_x (along +x or -x)
      2) Adjust y to target_y (along +y or -y)
    Uses forward_with_obstacle_check for straight segments.
    """
    debug("drive_to(): target_x={:.3f}, target_y={:.3f}".format(target_x, target_y))

    # --- Phase 1: adjust X ---
    wait_for_visibility()
    while True:
        x = enes100.x
        if x == -1:
            debug("X not visible; waiting...")
            stop()
            wait_for_visibility()
            continue

        debug("drive_to X: x={:.3f}, target_x={:.3f}".format(x, target_x))

        if abs(x - target_x) <= X_TOLERANCE:
            debug("X aligned.")
            stop()
            break

        if x < target_x:
            set_angle(0.0)    # face +x (obstacle zone direction)
        else:
            set_angle(pi)     # face -x (back toward start)

        forward_with_obstacle_check(speed)
        sleep(0.1)
        stop()
        sleep(0.05)

    # --- Phase 2: adjust Y ---
    wait_for_visibility()
    while True:
        y = enes100.y
        if y == -1:
            debug("Y not visible; waiting...")
            stop()
            wait_for_visibility()
            continue

        debug("drive_to Y: y={:.3f}, target_y={:.3f}".format(y, target_y))

        if abs(y - target_y) <= Y_TOLERANCE:
            debug("Y aligned.")
            stop()
            break

        if y < target_y:
            set_angle(pi/2)    # face +y
        else:
            set_angle(-pi/2)   # face -y

        forward_with_obstacle_check(speed)
        sleep(0.1)
        stop()
        sleep(0.05)

    stop()
    debug("drive_to: reached ({:.3f}, {:.3f})".format(target_x, target_y))

# ============================================================
# MISSION LOGIC – LIMBO-ONLY (TOP LANE)
# ============================================================

def run_mission():
    """
    1. Start at A or B.
    2. Go to mission site (opposite square).
    3. Move to TOP lane (limbo row).
    4. Drive through obstacle zone + open zone on TOP lane.
    5. End in goal zone on TOP lane.
    6. Send MATERIAL mission result.
    """
    wait_for_visibility()
    start_x = enes100.x
    start_y = enes100.y
    debug("Start pose: x={:.3f}, y={:.3f}".format(start_x, start_y))

    # Mission site is mirrored across y=1.0
    mission_x = start_x
    mission_y = ARENA_Y_MAX - start_y
    debug("Mission site: x={:.3f}, y={:.3f}".format(mission_x, mission_y))

    # Step 1: go to mission site
    drive_to(mission_x, mission_y)

    # Step 2: shift to TOP lane (limbo path)
    debug("Shifting to TOP lane at y={:.2f}".format(TOP_LANE_Y))
    drive_to(mission_x, TOP_LANE_Y)

    # Step 3: enter obstacle zone at x≈1.0
    entry_x = 1.0
    debug("Driving to obstacle entry at x={:.2f}".format(entry_x))
    drive_to(entry_x, TOP_LANE_Y)

    # Step 4: exit obstacle zone at x≈2.7
    exit_x = 2.7
    debug("Driving to obstacle exit at x={:.2f}".format(exit_x))
    drive_to(exit_x, TOP_LANE_Y)

    # Step 5: move into open zone (under limbo) and then goal zone
    debug("Driving into open zone at x={:.2f}".format(OPEN_ZONE_START_X))
    drive_to(OPEN_ZONE_START_X, TOP_LANE_Y)

    debug("Driving to goal center at x={:.2f}".format(GOAL_CENTER_X))
    drive_to(GOAL_CENTER_X, TOP_LANE_Y)

    stop()
    debug("Arrived in goal zone on TOP lane!")

    # Step 6: MATERIAL mission report (placeholder values)
    enes100.mission('WEIGHT', 'MEDIUM')          # 'HEAVY','MEDIUM','LIGHT'
    enes100.mission('MATERIAL_TYPE', 'PLASTIC')  # 'FOAM','PLASTIC'
    debug("Mission calls sent: WEIGHT=MEDIUM, MATERIAL_TYPE=PLASTIC")

# ============================================================
# MAIN
# ============================================================

def main():
    global ENES_READY
    stop_all()

    # Connect to vision system
    enes100.begin(TEAM_NAME, TEAM_TYPE, ARUCO_ID, ROOM_NUM)
    ENES_READY = True
    enes100.print("Connected from main()")

    try:
        # Quick ultrasonic sanity check
        for _ in range(3):
            read_all_distances()
            sleep(0.3)

        # For first test you *can* temporarily replace run_mission() with:
        #   wait_for_visibility()
        #   sx, sy = enes100.x, enes100.y
        #   drive_to(sx + 0.3, sy)
        # to see a small forward move.
        run_mission()
        debug("run_mission() completed.")
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
