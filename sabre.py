import board
import adafruit_dotstar as dotstar
from bmp_reader import BMPReader

image = BMPReader('mario.bmp')
grid = image.get_pixels()

dot = dotstar.DotStar(board.SCK, board.MOSI, 144, brightness=1.0, auto_write=False)

column = 0
while True:
    # Make image 4x bigger
    for row in range(image.height):
        for i in range(4):
            dot[row*4 + i] = grid[row][column]
            dot[143 - (row*4 + i)] = grid[row][column]
    dot.show()

    column += 1
    column %= image.width
