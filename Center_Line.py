import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import cv2
from dronekit import *
from pymavlink import mavutil
import time
import math
import numpy as np

master = mavutil.mavlink_connection('/dev/ttyACM0')
vehicle = connect('/dev/ttyACM0', wait_ready=True)

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("Ждем моторы...")
    time.sleep(1)

def move_forward(duration):
    vehicle.channels.overrides['3'] = 1225
    vehicle.channels.overrides['1'] = 1500
    
def move_left_fast(duration):
    vehicle.channels.overrides['1'] = 1325
    vehicle.channels.overrides['3'] = 1200
    
def move_right_fast(duration):
    vehicle.channels.overrides['1'] = 1725
    vehicle.channels.overrides['3'] = 1200
    
def move_left_slow(duration):
    vehicle.channels.overrides['1'] = 1350
    vehicle.channels.overrides['3'] = 1200
    
def move_right_slow(duration):
    vehicle.channels.overrides['1'] = 1650
    vehicle.channels.overrides['3'] = 1200

cap = cv2.VideoCapture(0)

while True:
    ret, image = cap.read()
    image = cv2.resize(image, (640, 480))
    height, width = image.shape[:2]
    top_cutoff = int(height * 0.01)
    bottom_cutoff = int(height * 0.6)
    image = image[top_cutoff:bottom_cutoff, :]
    minLineLength = 15
    maxLineGap = 30
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    ret, threshold = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(threshold, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(edges,1,np.pi/180,10,minLineLength,maxLineGap)

    if lines is not None:

        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            center_x = (x1 + x2) // 2

            if center_x is not None:
                    
                if center_x >= 500:
                    print("Резкий поворот вправо")
                    move_right_fast(0.25)

                elif center_x > 380 and center_x < 500:
                    print("Легкий поворот вправо")
                    move_right_slow(0.25)

                elif center_x >= 320 and center_x <= 380:
                    print("Продолжение движения")
                    move_forward(0.25)

                elif center_x > 150 and center_x < 320:
                    print("Легкий поворот влево")
                    move_left_slow(0.25)

                elif center_x >= 0 and center_x <= 150:
                    print("Резкий поворот влево")
                    move_left_fast(0.25)
                    
    else:
        print("Линия не найдена")
        
    cv2.imshow("Image", edges)

    if cv2.waitKey(1) & 0xff == 27:
        break

cap.release()
cv2.destroyAllWindows()
