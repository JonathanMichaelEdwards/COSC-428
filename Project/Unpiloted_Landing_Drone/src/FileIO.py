import time, csv
    


def write(x_data, y_data, fileName):
    """
        - Writes data to file. 
    """
    i = 0 

    with open(f"{fileName}.csv",'w') as file:
        write = csv.writer(file, delimiter=',', lineterminator='\n')
        while i < len(x_data):  # Seperate data
            write.writerow([x_data[i], y_data[i]])
            i += 1
    file.close()


def read(file):
    """
        - Reads data file.
        
        Returns -> buffered (x, y) data from file.
    """
    i = 0 
    buff_x = []
    buff_y = []

    file = open(f"{file}.csv",'r')
    new_value = file.readlines()
    while i < len(new_value):  # Seperate data
        (x_data, y_data) = new_value[i].split(',')
        buff_x.append(x_data)
        buff_y.append(y_data.strip())
        i += 1
    file.close()
    
    return (buff_x, buff_y)


# x = [1.11, 2.22, 3.55]
# y = [4, 5, 6]
# write(x, y, "src/data/yaw_plotting_data")
# (x_data, y_data) = read("src/data/yaw_plotting_data")
# if (
# print(x_data, x_data)