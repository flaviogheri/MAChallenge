import math
import matplotlib.pyplot as plt

def generate_sine_wave(amplitude, frequency, duration, sampling_rate):
    # Calculate the number of samples
    num_samples = int(duration * sampling_rate)

    # Generate the time array
    time_array = [i/sampling_rate for i in range(num_samples)]

    # Generate the sine wave
    sine_wave = [amplitude * math.sin(2 * math.pi * frequency * t) for t in time_array]

    # Return the sine wave
    return sine_wave

# Generate a sine wave with amplitude 1, frequency 1 Hz, duration 10 seconds, and sampling rate 100 Hz
sine_wave = generate_sine_wave(1, 1, 10, 100)

# Create an empty plot with axis labels
fig, ax = plt.subplots()
ax.set_xlabel('Time (s)')
ax.set_ylabel('Amplitude')

# Plot the sine wave on the empty plot
ax.plot(sine_wave)

# Show the plot
plt.show()






