import board
from adafruit_apds9960.apds9960 import APDS9960
import digitalio
import busio
import adafruit_apds9960.apds9960
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_apds9960.apds9960.APDS9960(i2c)

class Gesture():
    def __init__(self):
        i2c = board.I2C()
        apds = APDS9960(i2c)
        apds.enable_gesture = True
        sensor.enable_gesture = True
        gesture = sensor.gesture()
        while gesture == 0:
            gesture = sensor.gesture()
        print('Saw gesture: {0}'.format(gesture))

        while True:
            self.gesture_controls(apds)

    def gesture_controls(self, apds):
        gesture = apds.gesture()
        if gesture == 1:
             print("up")
        elif gesture == 2:
            print("down")
        elif gesture == 3:
            print("left")
        elif gesture == 4:
            print("right")

if __name__ == "__main__": 
    gesture = Gesture()  