import time
import board
import busio

import adafruit_lsm303
import adafruit_dotstar as dotstar

dot = dotstar.DotStar(board.APA102_SCK, board.APA102_MOSI, 1, brightness=1.0, auto_write=False)

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm303.LSM303(i2c)

old = (None, None, None)

ON = True

while True:
    x, y, z = sensor.raw_acceleration

    if [o for o in old if o is not None]:
        diff = (abs(old[0] - x), abs(old[1] - y), abs(old[2] - z))

        dot.brightness = max(diff) / 32768.

        if ON:
            dot[0] = (255, 0, 0)
        else:
            dot[0] = (0, 0, 0)
        dot.show()

        ON = not ON

        old = (x, y, z)
        time.sleep(0.1)
    else:
        # print('Debug: (%s, %s, %s)' % (x, y, z))
        old = (x, y, z)
