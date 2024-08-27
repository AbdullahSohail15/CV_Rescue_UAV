print("Loading Libraries...")
from pymavlink import mavutil
import time
import numpy as np
import math
import argparse
import torch
import cv2
import time
import numpy as np
from algorithms import *
from copter_commands import *
from yolov5_engine import YoLov5TRT
from video_capture import VideoDisplay

print("Libraries Loaded Successfully...")
def parse_2d_list(args):
    return [args[i:i + 2] for i in range(0, len(args), 2)]
ap = argparse.ArgumentParser()
ap.add_argument("-alt", "--altitude", required=True,
   help="Altitude of the drone")
ap.add_argument("-alpha", "--alpha", required=True,
   help="Horizontal FOV")
ap.add_argument("-beta","--beta",required=True,help="Vertical FOV")
ap.add_argument("-coord_list","--coord_list", required=True,type=float, nargs='+', help='List of GPS coordinates for reaching the searching area')
ap.add_argument("-sa_list","--sa_list", required=True, type=float, nargs='+', help='List of 4 GPS coordinates of the Searching Area')
args = vars(ap.parse_args())

def main(altitude,alpha,beta,coord_list,sa_list):
    engine_file_path = "build/best.engine"
    YOLO=YoLov5TRT(engine_file_path)
    CAP=cv2.VideoCapture(0)
    frame=preprocessor(CAP)
    STREAM=VideoDisplay(name="Live Stream - Team Burraq", frame=frame)
    STREAM.start()
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    OUT=cv2.VideoWriter(f"recording_{time.time()}.mp4",fourcc,6,(640,480))
    STREAM_ARRAY=[YOLO,CAP,STREAM,OUT]
    DRONE=initialize_drone()
    DRONE.mav.command_long_send(DRONE.target_system, 110, mavutil.mavlink.MAV_CMD_DO_SET_RELAY, 0, 0, 1, 0, 0 ,0, 0, 0, 0)
    a=input("Enter a key to start: ")
    COUNT=[0]
    wait4start(DRONE)
    arm(DRONE)
    takeoff(DRONE,altitude,STREAM_ARRAY)
    start_mission(DRONE,STREAM_ARRAY,COUNT,coord_list,sa_list,altitude,alpha,beta,land=0)


if __name__ == '__main__':
    try:
        print("=====================================")
        print(f"Parsed Arguments: {int(args['altitude']),float(args['alpha']),float(args['beta'])}")
        coord_list=parse_2d_list(args['coord_list'])
        sa_list=parse_2d_list(args['sa_list'])
        print(f"The coordinates are: {coord_list}")
        print(f"The SA coordinates are: {sa_list}")
        print("=====================================")
        main(int(args['altitude']),float(args['alpha']),float(args['beta']),coord_list,sa_list)
    except KeyboardInterrupt:

        exit()
