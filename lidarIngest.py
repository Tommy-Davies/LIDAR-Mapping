import sys
import numpy as np
from rplidar import RPLidar
import matplotlib.pyplot as plt
import bufferedCurve
import scipy as sp

BUFFER_SIZE = 10
ALLOWED_ERROR = 4
curve_lidar = bufferedCurve.Curve(
            BUFFER_SIZE, ALLOWED_ERROR, errorType=bufferedCurve.Distance.PERPENDICULAR, runOnLogic=True)


def plotPoints(x, y, reduced_points, fig, ax):
    if(x.all() is None):
        plt.show()
    plt.clf()
    newX, newY = zip(*reduced_points)
    plt.scatter(0, 0, c='g')
    plt.scatter(x, y, c='b')
    # plt.scatter(newX, newY, c='r')
    plt.xlim([-6000, 6000])
    plt.ylim([-6000, 6000])
    fig.canvas.draw()
    plt.pause(0.01)

def runScan(lidar):
    # fig = plt.figure(figsize=(15,10))
    # ax = fig.add_subplot(111)
    fig, ax = plt.subplots()
    for scan in lidar.iter_scans():
        r = np.array(scan)
        theta = (r[:,1])
        h = r[:,2]
        x = h * (np.sin(np.deg2rad(theta)))
        y = h * (np.cos(np.deg2rad(theta)))

        for i in range(len(x)):
            curve_lidar.add_point(x[i], y[i])
        curve_lidar.reduce_current_buffer(True)
        reduced_points = curve_lidar.get_reduced_points(True)
        # print("Number of points kept by algorithm:", len(
        # reduced_points), "out of", len(x))
        # print(reduced_points)
        plotPoints(x, y, reduced_points, fig, ax)

def run():
    '''Main function'''
    lidar = RPLidar('/dev/cu.usbserial-0001')
    
    xData = []
    yData = []
    
    try:
        print('Recording measurments... Press Crl+C to stop.')

        runScan(lidar)

        
            
    except KeyboardInterrupt:
        print('Stopping.')
 
    lidar.stop()
    lidar.disconnect()


if __name__ == '__main__':
    run()