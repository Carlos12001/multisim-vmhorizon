import pyfirmata
import time
import numpy as np
import matplotlib.pyplot as plt
import threading

# Define constants
SAMPLES = 128
SAMPLING_FREQUENCY = 8000
THRESHOLD = 50
TOLERANCE = 20
low_freqs = [697, 770, 852]
high_freqs = [1209, 1336, 1477]

# Set up Arduino board using pyfirmata
board = pyfirmata.Arduino('COM3')  # Replace 'COM3' with your actual COM port
it = pyfirmata.util.Iterator(board)
it.start()

# Set up the analog input pin and digital output pins
analog_input = board.get_pin('a:0:i')
led_pins = [
    [board.get_pin('d:2:o'), board.get_pin('d:3:o'), board.get_pin('d:4:o')],  # LEDs for the 697 Hz row
    [board.get_pin('d:5:o'), board.get_pin('d:6:o'), board.get_pin('d:7:o')],  # LEDs for the 770 Hz row
    [board.get_pin('d:8:o'), board.get_pin('d:9:o'), board.get_pin('d:10:o')]  # LEDs for the 852 Hz row
]

# Flag to enable/disable plotting
ENABLE_PLOT = True

# Function to acquire data from Arduino
def acquire_data():
    raw_data = []
    sampling_period = 1.0 / SAMPLING_FREQUENCY
    for _ in range(SAMPLES):
        value = analog_input.read()
        if value is not None:
            raw_data.append(value * 5.0)  # Assuming the value is normalized (0-1) and scaling to 0-5V
        else:
            raw_data.append(0.0)  # Append 0 if no value is read
        time.sleep(sampling_period)
    return np.array(raw_data)

# Function to process the acquired signal
def process_signal(signal):
    # Apply Hamming window
    windowed_signal = signal * np.hamming(SAMPLES)
    # Compute FFT
    fft_result = np.fft.fft(windowed_signal)
    magnitude = np.abs(fft_result[:SAMPLES // 2])
    frequencies = np.linspace(0, SAMPLING_FREQUENCY / 2, SAMPLES // 2)
    return magnitude, frequencies

# Function to detect frequencies
def detect_frequencies(magnitude, frequencies):
    low_index = -1
    high_index = -1
    # Search for low frequencies
    for i, freq in enumerate(low_freqs):
        low_range = (freq - TOLERANCE, freq + TOLERANCE)
        for j, f in enumerate(frequencies):
            if low_range[0] <= f <= low_range[1] and magnitude[j] > THRESHOLD:
                low_index = i
                break
        if low_index != -1:
            break
    # Search for high frequencies
    for i, freq in enumerate(high_freqs):
        high_range = (freq - TOLERANCE, freq + TOLERANCE)
        for j, f in enumerate(frequencies):
            if high_range[0] <= f <= high_range[1] and magnitude[j] > THRESHOLD:
                high_index = i
                break
        if high_index != -1:
            break
    return low_index, high_index

# Function to control LEDs based on detected frequencies
def control_leds(low_index, high_index):
    # Turn off all LEDs before updating
    for row in led_pins:
        for led in row:
            led.write(0)

    # Turn on the corresponding LED if both frequencies are detected
    if low_index != -1 and high_index != -1:
        led_pins[low_index][high_index].write(1)
        print(f"LED at row {low_index}, column {high_index} turned on.")
    else:
        print("No valid frequency detected.")

# Function to plot frequencies in real-time
def plot_frequencies():
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'r')
    ax.set_xlim(0, SAMPLING_FREQUENCY / 2)
    ax.set_ylim(0, 100)
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('Magnitude')
    ax.set_title('Real-time Frequency Spectrum')

    while ENABLE_PLOT:
        signal = acquire_data()
        magnitude, frequencies = process_signal(signal)
        line.set_xdata(frequencies)
        line.set_ydata(magnitude)
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.1)

# Main loop to acquire data, process it, and control LEDs
def main_loop():
    try:
        while True:
            signal = acquire_data()
            magnitude, frequencies = process_signal(signal)
            low_index, high_index = detect_frequencies(magnitude, frequencies)
            control_leds(low_index, high_index)
            time.sleep(0.1)
    finally:
        board.exit()

# Run the main program
if __name__ == "__main__":
    if ENABLE_PLOT:
        plot_thread = threading.Thread(target=plot_frequencies)
        plot_thread.daemon = True
        plot_thread.start()

    main_loop()
