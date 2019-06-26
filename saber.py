"""Pi Saber

Wiring:
2 - Red
4 - Red
6 - Black
19 - White
23 - Yellow
25 - Black
"""
from itertools import count
from random import randint

import adafruit_dotstar as dotstar
import board

DOTS = 144


def counter(dots):
    for i in count():
        dots.fill((0, 0, 0))
        chosen = -abs(i % (2 * DOTS) - DOTS) + DOTS
        dots[chosen] = (255, 0, 0)
        dots.show()


def colors(dots):
    while True:
        for i in range(DOTS):
            dots[i] = (randint(0, 255), randint(0, 255), randint(0, 255))
        dots.show()


def main():
    with dotstar.DotStar(board.SCK, board.MOSI, DOTS, brightness=1.0, auto_write=False) as dots:
        # counter(dots)
        colors(dots)


if __name__ == '__main__':
    main()
