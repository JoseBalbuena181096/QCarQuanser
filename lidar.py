import time
import numpy as np
import cv2
import math
from pal.utilities.lidar import Lidar

class LidarProcessor:
    def __init__(self):
        self.pixelsPerMeter = 50
        self.sideLengthScale = 8 * self.pixelsPerMeter
        self.decay = 0.9
        self.maxDistance = 1.5
        self.map = np.zeros((self.sideLengthScale, self.sideLengthScale), dtype=np.float32)

        # Lidar settings
        self.numMeasurements = 360
        self.lidarMeasurementMode = 2
        self.lidarInterpolationMode = 0

        self.myLidar = Lidar(
            type='RPLidar',
            numMeasurements=self.numMeasurements,
            rangingDistanceMode=self.lidarMeasurementMode,
            interpolationMode=self.lidarInterpolationMode
        )

    def lidar_measure(self):
        self.map = self.decay * self.map
        self.myLidar.read()
        anglesInBodyFrame = self.myLidar.angles * -1 + np.pi/2
        idx = [i for i, v in enumerate(self.myLidar.distances) if v < self.maxDistance]
        x = self.myLidar.distances[idx] * np.cos(anglesInBodyFrame[idx])
        y = self.myLidar.distances[idx] * np.sin(anglesInBodyFrame[idx])

        pX = (self.sideLengthScale/2 - x*self.pixelsPerMeter).astype(np.uint16)
        pY = (self.sideLengthScale/2 - y*self.pixelsPerMeter).astype(np.uint16)
        coordenadas_filtradas = [(px, py) for px, py in zip(pX, pY) if 190 <= py <= 210 and px <= 190]
        
        if coordenadas_filtradas:
            pX_filtrado, pY_filtrado = zip(*coordenadas_filtradas)
            self.map[pX_filtrado, pY_filtrado] = 1
            return self.map, pY_filtrado, pX_filtrado
        return self.map, [], []

    def detect_object(self):
        map_, pX_, pY_ = self.lidar_measure()
        Y_ = [y for y in pY_ if y > 165]
        return len(Y_) > 10

    def end_lidar(self):
        self.myLidar.terminate()

# Usage example:
# lidar_processor = LidarProcessor()
# if lidar_processor.detect_object():
#     print("Object detected!")
# lidar_processor.end_lidar()