from flask import Flask, render_template, request, jsonify
import math
import rospy
import os
import time
from datetime import datetime
import cv2
from clover import srv
from std_srvs.srv import Trigger
import threading
import requests

speed = 0.3

# ROS service proxies
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land_srv = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=0, yaw=math.nan, speed=speed, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res

# Function to get coordinates from the PID results endpoint
def get_pid_results():
    try:
        response = requests.get('http://127.0.0.1:5000/get_pid_results')
        response.raise_for_status()
        data = response.json()
        return data['x'], data['y']
    except requests.exceptions.RequestException as e:
        print(f"Error fetching PID results: {e}")
        return None, None

def main():
    target = get_pid_results()
    if target is not None and target[0] is not None and target[1] is not None:
        print(f"Target coordinates: x={-target[1]}, y={-target[0]} at {datetime.now().isoformat()}")
        navigate_wait(x=-target[1], y=-target[0], z=0, frame_id='body', auto_arm=True)
    else:
        print(f"Target coordinates are None, skipping navigation at {datetime.now().isoformat()}")

if __name__ == '__main__':
    rospy.init_node('flight_node')
    # navigate_wait(x=0, y=0, z=1.4, frame_id='body', auto_arm=True)
    
    while not rospy.is_shutdown():
        main()
        
    land_srv()

