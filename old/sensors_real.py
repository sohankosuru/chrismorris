# Complete project details at https://RandomNerdTutorials.com/micropython-hc-sr04-ultrasonic-esp32-esp8266/
from hcsr04 import HCSR04
from time import sleep

FRONT_TRIG_PIN = 4   # e.g. 5
FRONT_ECHO_PIN = 18   # e.g. 18

LEFT_TRIG_PIN  = 2   # e.g. 16
LEFT_ECHO_PIN  = 18   # e.g. 17

RIGHT_TRIG_PIN = 0   # e.g. 19
RIGHT_ECHO_PIN = 18   # e.g. 21
# ESP32
front_sonar = HCSR04(FRONT_TRIG_PIN, FRONT_ECHO_PIN)
left_sonar  = HCSR04(LEFT_TRIG_PIN,  LEFT_ECHO_PIN)
right_sonar = HCSR04(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN)

# ESP8266
#sensor = HCSR04(trigger_pin=12, echo_pin=14, echo_timeout_us=10000)

while True:
    distance1 = front_sonar.distance_cm()
    distance2 = left_sonar.distance_cm()
    distance3 = right_sonar.distance_cm()
    print('Distance1:', distance1, 'cm')
    print('Distance2:', distance2, 'cm')
    print('Distance3:', distance3, 'cm')
    sleep(1)
