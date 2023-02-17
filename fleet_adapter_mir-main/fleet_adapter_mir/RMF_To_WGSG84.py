#! /usr/bin/python

import math

# The RMF x and y is not always positive. Do remember to make everything positive. 
# If your world coordinate is outside of the max and min, that is the issue.


class WGS84:

    # Edit this based on the real world max and min
    # lat and lon of the building floorplan
    MIN_LON = 103.741333
    MIN_LAT = 1.330722
    MAX_LON = 103.742278
    MAX_LAT = 1.331612

    # Dimensions of RMF map
    width = 191.9904
    height = 135.5994

    angle = -0.523599  # 30 degrees
    # angle = -2.61799

    # Coordinates of the robot in RMF map
    x = None
    y = None

    # Final lat and lon coordinates
    lat = None
    lon = None

    def __init__(self, width, height, X, Y):
        self.MAP_WIDTH = width
        self.MAP_HEIGHT = height
        self.X = X
        self.Y = Y

    def rotate_point(self, rmfX, rmfY):

        s = math.sin(WGS84.angle)
        c = math.cos(WGS84.angle)

        xnew = lat * c - lon * s
        ynew = lat * s + lon * c

        return xnew, ynew
        
    def RMFConversion(self):

        diffLat = WGS84.MAX_LAT - WGS84.MIN_LAT
        diffLon = WGS84.MAX_LON - WGS84.MIN_LON

        widthProp = diffLat/WGS84.width
        heightProp = diffLon/WGS84.height

        finalLat = (self.X * widthProp) + WGS84.MIN_LAT
        finalLon = (self.Y * heightProp) + WGS84.MIN_LON

        # print("Lat: ", finalLat, "Lon: ", finalLon )
        # newlat, newlon = self.rotate_point(finalLat,finalLon)

        return finalLat, finalLon
        

        
    # Might need to implement function to obtain orientation

