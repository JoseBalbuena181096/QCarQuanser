'''This example demonstrates how to read and display data from the QCar Lidar
'''
import time
import matplotlib.pyplot as plt
from pal.products.qcar import QCarLidar, IS_PHYSICAL_QCAR

if not IS_PHYSICAL_QCAR:
    import qlabs_setup
    qlabs_setup.setup()

# polar plot object for displaying LIDAR data later on
ax = plt.subplot(111, projection='polar')
plt.show(block=False)

runTime = 10.0 # seconds
with QCarLidar() as myLidar:
    t0 = time.time()
    while time.time() - t0  < runTime:
        plt.cla()

        # Capture LIDAR data
        myLidar.read()

        ax.scatter(myLidar.angles, myLidar.distances, marker='.')
        ax.set_theta_zero_location("W")
        ax.set_theta_direction(-1)

        plt.pause(0.1)