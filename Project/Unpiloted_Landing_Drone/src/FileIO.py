import time, csv
    


def write_X_Y_Z_YAW(x_data, x, y, z, yaw, fileName):
    """
        - Writes (x, y, z) data to file. 
    """
    i = 0 

    with open(f"{fileName}.csv",'w') as file:
        write = csv.writer(file, delimiter=',', lineterminator='\n')
        while i < len(x_data):  # Seperate data
            try:
                write.writerow([x_data[i], x[i], y[i], z[i], yaw[i]])
            except IndexError:
                pass
            i += 1
    file.close()


def writeAngle(x_data, yaw_data, cp_data, fileName):
    """
        - Writes the Angle data to file. 
    """
    i = 0 

    with open(f"{fileName}.csv",'w') as file:
        write = csv.writer(file, delimiter=',', lineterminator='\n')
        while i < len(x_data):  # Seperate data
            try:
                write.writerow([x_data[i], yaw_data[i], cp_data[i]])
            except IndexError:
                pass
            i += 1
    file.close()

