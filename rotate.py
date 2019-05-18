#!/usr/bin/env python3

"""Persistence of Vision rotation script.


"""

import RPi.GPIO as GPIO
import time
import math
from contextlib import contextmanager
import argparse
from functools import partial
import board
import adafruit_dotstar as dotstar
from random import randint

DOTS = 72


@contextmanager
def PWM(pin):
    """Set up GPIO/PWM and return reference to PWM pin.

    Ensures that both are properly shut down afterwards.
    """

    try:
        # GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin, GPIO.OUT)
        pwm = GPIO.PWM(pin, 1)
        pwm.start(50)
        yield pwm
    finally:
        try:
            pwm.stop()
        except UnboundLocalError:
            pass
        GPIO.cleanup()


def freq(max_freq, growth, dt):
    """Calculate spin-up frequency according to S-curve based on time."""
    # return max(max_freq, 0.1 * math.pow(now - start, 2))
    return max_freq / (1 + max_freq / 5 * math.pow(math.e, -growth * dt))


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT)
    # for i in range(3200):
    timeout = 0.005
    with dotstar.DotStar(board.SCK, board.MOSI, DOTS, brightness=1.0, auto_write=False) as dots:
        dots[0] = (255, 0, 0)
        dots.show()
        while True:
            timeout = max(0.00025, 0.999 * timeout)
            GPIO.output(18, 1)
            time.sleep(timeout)
            GPIO.output(18, 0)
            time.sleep(timeout)
    # parser = argparse.ArgumentParser()
    # parser.add_argument('-m', '--max-frequency', type=int, default=1000)
    # parser.add_argument('-g', '--growth', type=float, default=2.0)
    # args = parser.parse_args()
    #
    # start = time.time()
    #
    # f = partial(freq, max_freq=args.max_frequency, growth=args.growth)
    #
    # with PWM(pin=18) as pwm, dotstar.DotStar(board.SCK, board.MOSI, DOTS, brightness=1.0, auto_write=False) as dots:
    #     dots[0] = (255, 0, 0)
    #     dots.show()
    #     while True:
    #         time.sleep(0.1)
    #         dt = time.time() - start
    #         frequency = f(dt=dt)
    #         pwm.ChangeFrequency(frequency)
    #
    #         # if dt % 0.1 < 0.005:
    #         #     dots[0] = (255, 0, 0)
    #         # else:
    #         #     dots[0] = (0, 0, 0)
    #         # dots.show()


if __name__ == '__main__':
    main()
