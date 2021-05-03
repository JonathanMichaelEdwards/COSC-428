#!python3.8

import matplotlib.pyplot as plt
import FileIO
import time, csv
import numpy as np


(x_data, y_data) = FileIO.read("src/data/yaw_plotting_data")

plt.plot(x_data, y_data)
plt.axis([min(x_data), max(x_data), min(y_data), max(y_data)])
plt.xticks(np.arange(-1, 20, 2))
plt.xlim(-1, 20)
plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (∠)')
plt.title('Yaw Control')
# plt.savefig('figure1.png')
plt.show()

