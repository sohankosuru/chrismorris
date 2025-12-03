from machine import ADC, Pin
from time import sleep

pin = Pin(26)
adc = ADC(pin)

# TOUCH THE BALL SUCH THAT THE CAPACITVE SENSOR IS JUST TOUCHING THE BALL BUT NOT
# PRESSING IT OR APPLYING ANY PRESSURE. THE SENSOR WILL DETECT THE HARD BALL
# AND THE SENSOR'S LED WILL TURN ON, AND WILL NOT "DETECT" THE SOFT BALL, MEANING
# WE CAN DIFFERENTIATE THE BALLS

while True:
    arr = []
    for i in range(10):
        val = adc.read_uv()  # units in uV
        arr.append(val)
        sleep(0.1)
    if (min(arr) > 300000):
        print("hard ball")
    elif (max(arr) < 200000):
        print("soft ball")
    else:
        print("scan was indeterminate")
