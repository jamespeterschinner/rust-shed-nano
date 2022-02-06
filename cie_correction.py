from resource import prlimit
from time import time_ns
import numpy as np

"""
The human eye doesn't pervieve brightness in a linear manner, but we do
controll the average lumens in a linear manner (with PWM) this little
script it to try and compensate for this non linearity while not reducing
the resolution of the PWM signal. Inorder to achieve this we'll approximate
time intervals to wait for between each update of the PWM controller.

In a summay, we're more sensitive to changes in actual lumens the darker it is
this means we need to increase the light on time more slowly when we're outputting
lower light levels
"""
# spi_clock_freq =  16000000 / 4

# spi_write_time = 1/spi_clock_freq * 288 # There are 288 bits

# timer_interval = 1 / (16000000 / 64)

# Generate a linear space of the desired percieved brightness 
# with the resolution of the PWM signal (12 bits)
percieved_brightness = np.linspace(0, 116, 4095)

"""
This calculation is to compensate for the nonlinearity of the human eye

More information can be found here: 
https://stackoverflow.com/questions/596216/formula-to-determine-perceived-brightness-of-rgb-color
"""
cutoff =  0.008856

lower_values = (percieved_brightness / 903.3)

lower_values = lower_values[lower_values <= cutoff]

upper_values = (0.000800411 * pow((percieved_brightness), 3/2))
upper_values = upper_values[upper_values > cutoff]

# Linear power in that this would be the power required to generate a linear percieved brightness
linear_power = np.concatenate((lower_values, upper_values))

# Because we can't actually control the light output to the precision
# Required by 'linear_power' we'll instead calculate 'time units' between each value
time_units = 1 / np.diff(linear_power)

# scale the 'time units' such that we end up at 0
time_units = (time_units - time_units[-1]) 

# Here we reduce the accuracy to integers so we have values that can be used 
# the atmega328p
time_units = np.array(sorted(set(np.round(time_units))))

# Because this is an approximation, eventually the 'time units' become completely linear
# when the light output is past a certain threshold. And we're only interested in the part
# which is mostly nonlinear
linear_portion = np.argmax(np.array(np.diff(time_units)) > 2)

"""
There is actually an inverse relationship between the PWM setting and light output
the above calculation is based upon a propotional relationship.

We'll adjust for that now
"""

time_units = time_units[linear_portion:] + 50

time_units = list(map(int, time_units))


print(f"""
use avr_progmem::progmem;

pub static CIE_THRESHOLD: u16 = {0xFFF - len(time_units)};

progmem! {{
    pub static progmem CIE_DELAY: [u16; {len(time_units)}] = {time_units};
}}
""")

