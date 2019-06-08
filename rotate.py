#!/usr/bin/env python3

"""Persistence of Vision rotation script.


"""

import argparse
import time
from contextlib import contextmanager

import RPi.GPIO as GPIO
import adafruit_dotstar as dotstar
import board

DOTS = 72

freqs = (
    100,
    150,
    200,
    400,
    600,
    800,
    1000,
    1200,
    1400,
    1600,
    2000,
    2500,
    3000,
    3500,
    4000,
    4250,
) + tuple(range(4300, 30000, 75))


COLORS = {
    'rainbow': (
        (209, 0, 0),
        (255, 102, 34),
        (255, 218, 33),
        (51, 221, 0),
        (17, 51, 204),
        (34, 0, 102),
        (51, 0, 68),
    ),
    'america': (
        (255, 0, 0),
        (255, 255, 255),
        (0, 0, 255),
    ),
    'bruised': (
        (0, 0, 115),
        (0, 0, 0),
    )
}


@contextmanager
def PWM(pin):
    """Set up GPIO/PWM and return reference to PWM pin.

    Ensures that both are properly shut down afterwards.
    """

    try:
        # GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin, GPIO.OUT)
        pwm = GPIO.PWM(pin, 1)
        pwm.start(30)
        yield pwm
    finally:
        try:
            pwm.stop()
        except UnboundLocalError:
            pass
        GPIO.cleanup()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--colors', choices=list(COLORS.keys()), default='rainbow')
    args = parser.parse_args()

    colors = COLORS[args.colors]

    start = time.time()

    with PWM(pin=12) as pwm, dotstar.DotStar(board.SCK, board.MOSI, DOTS, brightness=1.0, auto_write=False) as dots:
        old_step = 0
        color_idx = 0

        while True:
            time.sleep(0.01)
            dt = time.time() - start
            steps = int(dt * 4)

            for doti in range(len(dots)):
                dots[doti] = colors[color_idx]
            dots.show()

            if old_step != steps:
                old_step = steps
                freq_idx = min(len(freqs) - 1, steps)
                freq = freqs[freq_idx]
                pwm.ChangeFrequency(freq)

            color_idx = (color_idx + 1) % len(colors)


if __name__ == '__main__':
    main()
