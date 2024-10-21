import time
from pyfirmata import Arduino, util

# Set up the port and board
arduino_board = Arduino('COM3')  # Change this to the corresponding port

# LED pins in a 3x3 matrix
led_pins = [
    [2, 5, 8],
    [3, 6, 9],
    [4, 7, 10]
]

# Set pins as output
for i in range(3):
    for j in range(3):
        arduino_board.digital[led_pins[i][j]].mode = 1  # OUTPUT

# Function to turn off all LEDs
def turn_off_leds():
    for i in range(3):
        for j in range(3):
            arduino_board.digital[led_pins[i][j]].write(0)

# Main loop
try:
    while True:
        for i in range(10):
            turn_off_leds()
            if i < 9:
                row = i // 3
                column = i % 3
                arduino_board.digital[led_pins[row][column]].write(1)
            time.sleep(1)

except KeyboardInterrupt:
    turn_off_leds()
    arduino_board.exit()
