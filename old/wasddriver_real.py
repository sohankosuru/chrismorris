# differential_drive_keys.py
# MicroPython script for ESP32 — control two motors via L298-like H-bridge
# Pins: motor A = PWM(Pin(12)), PWM(Pin(13))
#       motor B = PWM(Pin(25)), PWM(Pin(26))
# Controls: w/a/s/d to drive, x or space to stop, q to quit

from machine import Pin, PWM
import sys
import time

# --- Configuration ---
FREQ = 5000
# 10-bit PWM expected (0..1023). Adjust MAX_DUTY if your build uses other range.
MAX_DUTY_10BIT = 1023
DEFAULT_SPEED = 700  # 0..MAX_DUTY_10BIT (tweak to suit motors / battery)
PRINT_FEEDBACK = True

# --- PWM objects (from your starter code) ---
pwm0 = PWM(Pin(26), freq=FREQ)  # motor A input 1
pwm1 = PWM(Pin(27), freq=FREQ)  # motor A input 2
pwm2 = PWM(Pin(33), freq=FREQ)  # motor B input 1
pwm3 = PWM(Pin(32), freq=FREQ)  # motor B input 2

# --- helper to set duty robustly (handles duty() or duty_u16() variants) ---
def set_pwm_duty(pwm, duty_10bit):
    # clamp
    if duty_10bit < 0:
        duty_10bit = 0
    if duty_10bit > MAX_DUTY_10BIT:
        duty_10bit = MAX_DUTY_10BIT

    # try standard 10-bit duty() first; otherwise scale to 16-bit duty_u16
    try:
        pwm.duty(int(duty_10bit))
    except AttributeError:
        # duty_u16 expects 0..65535; scale from 0..1023
        scale = 65535 // MAX_DUTY_10BIT
        pwm.duty_u16(int(duty_10bit * scale))

# --- motor primitives ---
def motor_a_forward(speed=DEFAULT_SPEED):
    set_pwm_duty(pwm0, speed)  # IN1 = PWM
    set_pwm_duty(pwm1, 0)      # IN2 = 0

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

# --- high-level actions ---
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
    # left motor reverse, right motor forward -> spin left in place
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

# --- input loop ---
def read_char():
    """
    Read a single character from stdin. Returns '' on EOF.
    On some ports sys.stdin.read(1) returns bytes, on others str; we normalize to str.
    """
    try:
        ch = sys.stdin.read(1)  # blocking until key pressed
    except Exception:
        # some environments may raise; handle gracefully
        return ''
    if not ch:
        return ''
    # If bytes, decode; if already str, that's fine.
    if isinstance(ch, bytes):
        try:
            ch = ch.decode('utf-8')
        except Exception:
            ch = chr(ch[0])
    return ch

def main():
    print("Differential drive keyboard control")
    print("w=forward, s=reverse, a=turn left, d=turn right, x/space=stop, q=quit")
    print("Run this with the board's serial terminal focused so keypresses are sent to the device.")
    stop_all()
    try:
        while True:
            ch = read_char()
            if not ch:
                # no input, small sleep to yield CPU
                time.sleep_ms(20)
                continue
            ch = ch.lower()
            if ch == 'w':
                go_forward()
            elif ch == 's':
                go_reverse()
            elif ch == 'a':
                turn_left()
            elif ch == 'd':
                turn_right()
            elif ch == 'x' or ch == ' ':
                stop()
            elif ch == 'q':
                print("Quitting — stopping motors.")
                break
            else:
                # ignore other keys but provide small feedback
                if PRINT_FEEDBACK:
                    print("Ignored key: {!r}".format(ch))
    except KeyboardInterrupt:
        # Ctrl-C from REPL/terminal
        print("KeyboardInterrupt — stopping motors.")
    finally:
        stop_all()
        # optionally deinit PWM channels if you prefer
        try:
            pwm0.deinit()
            pwm1.deinit()
            pwm2.deinit()
            pwm3.deinit()
        except Exception:
            pass
        print("Clean exit.")

if __name__ == "__main__":
    main()
