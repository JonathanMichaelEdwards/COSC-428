import matplotlib.pyplot as plt
import FileIO
import time, csv
import numpy as np


(x_data, y_data) = FileIO.read("src/data/yaw_plotting_data")
# _x_data = list(range(len(x_data)))
# plt.xticks(_x_data, x_data)
# print(int(float(x_data[-2].strip())))
# x_data = np.arange(int(float(x_data[0].strip())), int(float(x_data[-2].strip())), 0.5)

plt.plot(x_data, y_data)
plt.xticks(range(1, 30, 2))
plt.xlim(0, 21)
plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (∠)')
plt.title('Yaw Control')
# plt.savefig('figure1.png')
plt.show()


