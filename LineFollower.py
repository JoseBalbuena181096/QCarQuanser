## task_lane_following.py
# This example combines both the left csi and motor commands to 
# allow the QCar to follow a yellow lane. The car will stop when
# an obstacle is detected in front of it.

from pal.products.qcar import QCar
from pal.utilities.math import *

import numpy as np 
import cv2
import time

from lines import LaneDetect
from keys import KeyListener
from lidar import LidarProcessor
from camera import CameraProcessor
from control import ControlSystem

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Initialize car components
myCar = QCar()

lanes = LaneDetect()
lidar = LidarProcessor()
camera = CameraProcessor()
control = ControlSystem(kp=0.00225)  # Using improved gain value
key_listener = KeyListener()
key_listener.start()

# Initialize control variables
throttle_axis = 0 
steering_axis = 0
driving_active = False
obstacle_detected = False
resume_counter = 0
RESUME_TIME = 5  # Frames to wait before resuming after obstacle is no longer detected

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

## Main Loop
print("Starting line follower. Press 'a' to begin, 's' to stop.")

try:
    while not key_listener.should_exit:
        # Take frame and process lines
        frame = camera.take_frame()
        lines_frame = lanes.find_lines(frame)
        
        # Process key presses
        if key_listener.last_key_pressed == 'a' and not driving_active:
            driving_active = True
            print("Line following activated!")
        
        if key_listener.last_key_pressed == 's' and driving_active:
            driving_active = False
            throttle_axis = 0
            steering_axis = 0
            print("Line following stopped!")
        
        # Check for obstacle detection from LIDAR
        current_obstacle = lidar.detect_object()
        
        # If obstacle state changed
        if current_obstacle != obstacle_detected:
            obstacle_detected = current_obstacle
            if obstacle_detected:
                print("Obstacle detected! Stopping.")
                resume_counter = RESUME_TIME  # Set resume countdown
            else:
                print("Obstacle no longer detected.")
        
        # Handle obstacle avoidance countdown
        if not obstacle_detected and resume_counter > 0:
            resume_counter -= 1
            
        # Handle control based on state
        if driving_active:
            # Calculate steering based on line position with PID control
            steering_axis = control.control_pid(lanes.error)
            
            # Set throttle based on obstacle detection
            if obstacle_detected or resume_counter > 0:
                throttle_axis = 0  # Stop when obstacle detected
            else:
                throttle_axis = 0.08  # Normal speed
        else:
            # Not active - no movement
            throttle_axis = 0
            steering_axis = 0
        
        # Apply steering limits
        steering_axis = control.saturate(steering_axis, 0.6, -0.6)
        
        # Set LED indicators
        if driving_active:
            if obstacle_detected:
                # Obstacle detected - warning lights
                LEDs = np.array([1, 0, 1, 0, 1, 0, 1, 0])
            else:
                # Normal driving
                LEDs = np.array([0, 0, 0, 0, 1, 1, 0, 0])
        else:
            # Not active
            LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        
        # Send commands to the car
        myCar.read_write_std(throttle_axis, steering_axis, LEDs)
        
        # Add status information to display
        status_y = 30
        cv2.putText(lines_frame, f"Active: {driving_active}", (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        status_y += 30
        cv2.putText(lines_frame, f"Error: {lanes.error:.2f}", (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        status_y += 30
        cv2.putText(lines_frame, f"Throttle: {throttle_axis:.2f}", (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        status_y += 30
        cv2.putText(lines_frame, f"Steering: {steering_axis:.2f}", (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        status_y += 30
        cv2.putText(lines_frame, f"Obstacle: {obstacle_detected}", (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        if resume_counter > 0:
            status_y += 30
            cv2.putText(lines_frame, f"Resuming in: {resume_counter}", (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Display the frame
        try:        		     
            cv2.imshow('Camera', lines_frame)
        except:
            pass
        
        # Process GUI events
        cv2.waitKey(1)
    
except KeyboardInterrupt:
    print("User interrupted!")

finally:
    # Terminate camera and QCar
    myCar.terminate()
    lidar.end_lidar()
    camera.end_camera()
    print("All systems terminated.")
# -- -- -- -- -- -- -- -- -- -- -- -- -- --