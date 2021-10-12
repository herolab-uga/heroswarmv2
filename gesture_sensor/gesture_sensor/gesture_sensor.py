import board
from adafruit_apds9960.apds9960 import APDS9960
import digitalio


class Gesture():
    def __init__(self):
        i2c = board.I2C()
        apds = APDS9960(i2c)
        apds.enable_gesture = True
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