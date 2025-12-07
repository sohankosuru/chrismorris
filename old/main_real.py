# main.py – ENES100 OTV navigation
# Uses your working motor PWM code + enes100 + 3x HCSR04 ultrasonics

from machine import Pin, PWM
from time import sleep
from math import pi
from enes100 import enes100
from hcsr04 import HCSR04

# ============================================================
# ENES100 CONFIG – FILL THESE IN
# ============================================================

TEAM_NAME  = "Team Chris Morris AWOG"
TEAM_TYPE  = "MATERIAL"   # mission type
ARUCO_ID   = 7            ### TODO: your ArUco marker ID (int)
ROOM_NUM   = 1120            ### TODO: 1116 or 1120

# ============================================================
# MOTOR CONFIG – YOUR WORKING CODE
# ============================================================

FREQ = 5000
MAX_DUTY_10BIT = 1023
DEFAULT_SPEED = 700  # 0..1023
PRINT_FEEDBACK = True

# PWM objects – exactly as in your script
pwm0 = PWM(Pin(26), freq=FREQ)  # motor A input 1
pwm1 = PWM(Pin(27), freq=FREQ)  # motor A input 2
pwm2 = PWM(Pin(14), freq=FREQ)  # motor B input 1
pwm3 = PWM(Pin(12), freq=FREQ)  # motor B input 2

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

# --- motor primitives (unchanged) ---
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
        print("FORWARD (speed={})".format(speed))

def go_reverse(speed=DEFAULT_SPEED):
    motor_a_reverse(speed)
    motor_b_reverse(speed)
    if PRINT_FEEDBACK:
        print("REVERSE (speed={})".format(speed))

def turn_left(speed=DEFAULT_SPEED):
    motor_a_reverse(speed)
    motor_b_forward(speed)
    if PRINT_FEEDBACK:
        print("TURN LEFT (speed={})".format(speed))

def turn_right(speed=DEFAULT_SPEED):
    motor_a_forward(speed)
    motor_b_reverse(speed)
    if PRINT_FEEDBACK:
        print("TURN RIGHT (speed={})".format(speed))

def stop():
    stop_all()
    if PRINT_FEEDBACK:
        print("STOP")

# ============================================================
# ULTRASONIC SENSORS – 3x HCSR04 (front, left, right)
# ============================================================

# TODO: fill in the actual GPIO numbers for these 6 pins
FRONT_TRIG_PIN = 33   # e.g. 5
FRONT_ECHO_PIN = 19   # e.g. 18

LEFT_TRIG_PIN  = 32   # e.g. 16
LEFT_ECHO_PIN  = 27   # e.g. 17

RIGHT_TRIG_PIN = 18   # e.g. 19
RIGHT_ECHO_PIN = 34  # e.g. 21

front_sonar = HCSR04(FRONT_TRIG_PIN, FRONT_ECHO_PIN)
left_sonar  = HCSR04(LEFT_TRIG_PIN,  LEFT_ECHO_PIN)
right_sonar = HCSR04(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN)

# Safety thresholds (tweak in lab)
SAFE_FRONT_CM = 20.0   # stop/avoid if front < 20 cm
SAFE_SIDE_CM  = 15.0   # steer away if side < 15 cm

# ============================================================
# ARENA CONSTANTS (meters)
# ============================================================

ARENA_Y_MAX        = 2.0
OPEN_ZONE_START_X  = 2.8    # start of open zone
GOAL_CENTER_X      = 3.7    # approx center of goal zone
TOP_LANE_Y         = 1.5    # limbo row
# BOTTOM_LANE_Y    = 0.5    # log row (we’re not using this)

# Nav tuning
X_TOLERANCE = 0.03
Y_TOLERANCE = 0.03
THETA_THRESHOLD = 0.05  # radians (~3°)

TURN_KP = 1.0   # P gain for set_angle

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

def wait_for_visibility():
    while not enes100.is_visible:
        print("Waiting for visibility...")
        sleep(0.1)
    print("Marker visible.")

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

    print("Distances  F:{:.1f}  L:{:.1f}  R:{:.1f} cm".format(df, dl, dr))
    return df, dl, dr

# ============================================================
# OBSTACLE-AWARE FORWARD USING YOUR MOTOR FUNCTIONS
# ============================================================

def forward_with_obstacle_check(speed=DEFAULT_SPEED):
    """
    Use go_forward / go_reverse / turn_left / turn_right / stop()
    with 3 ultrasonics to avoid obstacles.
    """
    df, dl, dr = read_all_distances()

    # Hard avoid if front is blocked
    if df < SAFE_FRONT_CM:
        print("Front obstacle – backing up and turning.")
        stop()
        sleep(0.1)

        go_reverse(speed)
        sleep(0.4)
        stop()
        sleep(0.1)

        # Turn toward more open side
        if dl > dr + 5:
            print("More space on LEFT → turn left.")
            turn_left(speed)
        elif dr > dl + 5:
            print("More space on RIGHT → turn right.")
            turn_right(speed)
        else:
            print("Sides similar → default turn right.")
            turn_right(speed)

        sleep(0.5)
        stop()
        sleep(0.1)
        return

    # Side “hugging” – steer away if one side is too close
    left_close  = dl < SAFE_SIDE_CM
    right_close = dr < SAFE_SIDE_CM

    if left_close and not right_close:
        print("Wall LEFT → veer right.")
        # left wheel slower: right motor forward, left motor slight reverse/stop
        motor_a_reverse(int(speed * 0.3))  # left
        motor_b_forward(speed)             # right
    elif right_close and not left_close:
        print("Wall RIGHT → veer left.")
        motor_a_forward(speed)             # left
        motor_b_reverse(int(speed * 0.3))  # right
    else:
        go_forward(speed)

# ============================================================
# HEADING CONTROL: set_angle
# ============================================================

def set_angle(target):
    """
    Turn in place until enes100.theta ≈ target (radians).
    Uses P control but with your full-speed turn functions.
    """
    target = normalize_angle(target)
    print("set_angle → target =", target)
    wait_for_visibility()

    while True:
        theta = enes100.theta
        if theta == -1:
            print("Theta not visible; waiting...")
            stop()
            sleep(0.1)
            continue

        error = normalize_angle(target - theta)
        print("theta={:.3f}, error={:.3f}".format(theta, error))

        if abs(error) < THETA_THRESHOLD:
            break

        turn_cmd = TURN_KP * error
        turn_cmd = constrain(turn_cmd, -1.0, 1.0)

        if turn_cmd > 0:
            # positive error → need to rotate CCW (right motor backward, left forward)
            turn_right(DEFAULT_SPEED)
        else:
            # negative error → rotate CW
            turn_left(DEFAULT_SPEED)

        sleep(0.02)

    stop()
    print("Reached target angle.")

# ============================================================
# POSITION CONTROL: drive_to(x, y)
# ============================================================

def drive_to(target_x, target_y, speed=DEFAULT_SPEED):
    """
    Move in an 'L' shape:
      1) Adjust x to target_x (along +x or -x)
      2) Adjust y to target_y (along +y or -y)
    """
    # --- Phase 1: adjust X ---
    wait_for_visibility()
    x = enes100.x
    print("drive_to X: now x={:.3f}, target_x={:.3f}".format(x, target_x))

    if x < target_x:
        set_angle(0.0)  # +x
        while True:
            if not enes100.is_visible:
                print("Lost visibility (X+); waiting...")
                stop()
                wait_for_visibility()
            x = enes100.x
            print("Moving +X → x={:.3f}".format(x))
            if x >= target_x - X_TOLERANCE:
                break
            forward_with_obstacle_check(speed)
            sleep(0.05)
    else:
        set_angle(pi)   # -x
        while True:
            if not enes100.is_visible:
                print("Lost visibility (X-); waiting...")
                stop()
                wait_for_visibility()
            x = enes100.x
            print("Moving -X → x={:.3f}".format(x))
            if x <= target_x + X_TOLERANCE:
                break
            forward_with_obstacle_check(speed)
            sleep(0.05)

    stop()
    print("Reached target X.")

    # --- Phase 2: adjust Y ---
    wait_for_visibility()
    y = enes100.y
    print("drive_to Y: now y={:.3f}, target_y={:.3f}".format(y, target_y))

    if y < target_y:
        set_angle(pi / 2)  # +y
        while True:
            if not enes100.is_visible:
                print("Lost visibility (Y+); waiting...")
                stop()
                wait_for_visibility()
            y = enes100.y
            print("Moving +Y → y={:.3f}".format(y))
            if y >= target_y - Y_TOLERANCE:
                break
            forward_with_obstacle_check(speed)
            sleep(0.05)
    else:
        set_angle(-pi / 2) # -y
        while True:
            if not enes100.is_visible:
                print("Lost visibility (Y-); waiting...")
                stop()
                wait_for_visibility()
            y = enes100.y
            print("Moving -Y → y={:.3f}".format(y))
            if y <= target_y + Y_TOLERANCE:
                break
            forward_with_obstacle_check(speed)
            sleep(0.05)

    stop()
    print("Reached target Y.")

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
    print("Start pose: x={:.3f}, y={:.3f}".format(start_x, start_y))

    # Mission site is mirrored across y=1.0
    mission_x = start_x
    mission_y = ARENA_Y_MAX - start_y
    print("Mission site: x={:.3f}, y={:.3f}".format(mission_x, mission_y))

    # Step 1: go to mission site
    drive_to(mission_x, mission_y)

    # Step 2: shift to TOP lane (limbo path)
    print("Shifting to TOP lane at y={:.2f}".format(TOP_LANE_Y))
    drive_to(mission_x, TOP_LANE_Y)

    # Step 3: enter obstacle zone at x≈1.0
    entry_x = 1.0
    drive_to(entry_x, TOP_LANE_Y)

    # Step 4: exit obstacle zone at x≈2.7
    exit_x = 2.7
    drive_to(exit_x, TOP_LANE_Y)

    # Step 5: move into open zone (under limbo) and then goal zone
    drive_to(OPEN_ZONE_START_X, TOP_LANE_Y)
    drive_to(GOAL_CENTER_X, TOP_LANE_Y)

    stop()
    print("Arrived in goal zone on TOP lane!")

    # Step 6: MATERIAL mission report (replace with your real values)
    enes100.mission('WEIGHT', 'MEDIUM')          # 'HEAVY','MEDIUM','LIGHT'
    enes100.mission('MATERIAL_TYPE', 'PLASTIC')  # 'FOAM','PLASTIC'

# ============================================================
# MAIN
# ============================================================

def main():
    print("Starting ENES100 OTV navigation...")
    stop_all()
    enes100.begin(TEAM_NAME, TEAM_TYPE, ARUCO_ID, ROOM_NUM)
    enes100.print("Connected from main()")

    # Quick ultrasonic sanity check
    for _ in range(3):
        read_all_distances()
        sleep(0.3)

    run_mission()

if __name__ == "__main__":
    main()
