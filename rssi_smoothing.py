import numpy as np
from scipy import signal
L=3 #L-point filter
b = (np.ones(L))/L #numerator co-effs of filter transfer function
a = np.ones(1)  #denominator co-effs of filter transfer function
# x = np.random.randn(10) #10 random samples for x
x = [1, 2, 3, 7, 9]
y = signal.convolve(x,b) #filter output using convolution
y = signal.lfilter(b,a,x) #filter output using lfilter function
# print(type(y))
print(np.array(y).tolist())

# numbers = [1, 2, 3, 7, 9]
# window_size = 3

# i = 0
# moving_averages = []
# while i < len(numbers) - window_size + 1:
#     this_window = numbers[i : i + window_size]           # get current window
#     window_average = sum(this_window) / window_size
#     moving_averages.append(window_average)
#     i += 1

# print(moving_averages)