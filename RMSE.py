import os
import glob
import matplotlib.pyplot as plt
from utilities import FileReader
from math import sqrt

# Root Mean Squared Error
def euclidean_distance(x_odom, y_odom, x_EKF, y_EKF):
    return sqrt( (x_odom - x_EKF)**2 + (y_odom - y_EKF)**2 )

def calculate_RMSE(values, headers):
    square_summation = 0
    for lin in values:
        list_len = len(headers)
        dist = euclidean_distance(
            lin[list_len - 3], #x_odom
            lin[list_len - 2], #y_odom
            lin[list_len - 5], #x_EKF
            lin[list_len - 4]  #y_EKF
        )
        square_summation += dist**2

    RMSE = sqrt( square_summation / len(values) )

    print(f"RMSE: {RMSE}")
    return RMSE


# Import All csv's
relative_folder_path = "./data"
csv_files = glob.glob(os.path.join(relative_folder_path, "*.csv"))

RMSEs = []
names = []
for csv_file in csv_files:
    if "point" in csv_file:
        print(f"File: {csv_file.replace('./data/point_EKF_', '')}")
        headers, values = FileReader(csv_file).read_file()
        RMSEs.append(calculate_RMSE(values, headers))
        names.append(csv_file)

min_RMSE = min(RMSEs)
min_name = names[RMSEs.index(min_RMSE)]
print(f"min | RMSE: {min_RMSE} | name: {min_name}")