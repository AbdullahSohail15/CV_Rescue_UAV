from pymavlink import mavutil
import numpy as np
import cv2
import time
import math
from calculations import *
from copter_commands import *


def IBVS_algo(drone, w, pError, pError_y, pError_z, y, boxes, stream_array):
  """
  Function to implement the IBVS algorithm for aligning the drone with the drowning person

  Args:
    drone: obj: The connection object of the drone
    w: int: The width of the frame
    pError: float: The previous error in x
    pError_y: float: The previous error in y
    pError_z: float: The previous error in z
    y: int: The height of the frame
    boxes: list: The list of bounding boxes
    stream_array: list: The list of the stream objects

  Returns:
    pError: float: The previous error in x
    pError_y: float: The previous error in y
    pError_z: float: The previous error in z
    speed_y: float: The speed in y direction
    speed: float: The speed in x direction
  """
  pid=[0.003,0.0005]
  xmin=boxes[0][0]
  ymin=boxes[0][1]
  xmax=boxes[0][2]
  ymax=boxes[0][3]

  cx=xmin+(xmax-xmin)//2
  cy=ymin+(ymax-ymin)//2
  area=(xmax-xmin)*(ymax-ymin)
  print("The center in x is {}".format(cx))
  print("The center in y is {}".format(cy))
  error_z=area-10000
  error_y=cy-y
  error = cx-w
  print(f"Error in X: {error} | Error in Y: {error_y} | Error in Z: {error_z}")
  if ((cx!=0) and (abs(error)>80 or abs(error_y)>80)):

    speed_y=pid[0]*error_y + pid[1]*(error_y-pError_y)
    speed_y=np.clip(speed_y,-5,5)

    speed=0.0008*error + 0.00025*(error-pError)
    speed=np.clip(speed,-5,5)

    speed_z=0.00001*error_z+0.00005*(error_z-pError_z)
    speed_z=np.clip(speed_z, -0.5, 0.5)

    print(f"Speed in x: {speed_y} | Speed in Z: {speed_z} | Yaw Rate: {speed}")

    drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,drone.target_system,drone.target_component,
                  mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,int(0b010111000111),0,0,0,-speed_y,0,0,0,0,0,0,speed))
  else:
    print("Navigation Completed")
    speed=0
    speed_y=0
    drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,drone.target_system,drone.target_component,
                  mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,int(0b110111000111),0,0,0,0,0,0,0,0,0,0,0))
    payload_delivery(drone,stream_array)

  return pError, pError_y, pError_z, speed_y, speed

def payload_delivery(drone,stream_array):
  stream_(stream_array)
  drone.mav.command_long_send(drone.target_system,110,mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,0,0,0,0,0,0,0,0,) #After DO_SET, 3rd Value
  print(f"Actuated Dropping Mechanism... Waiting 5 seconds...")
  count=0
  while count<=50:
    stream_(stream_array)
    count+=1
  print(f"5 Seconds Completed... Returning to Launch")
  stream_(stream_array)
  change_to_RTL(drone, stream_array)

def wait_wp(drone,stream_array,detection=True):
  """
  Function to wait for the drone to reach the waypoint

  Args:
    drone: obj: The connection object of the drone
    stream_array: list: The list of the stream objects
    detection: bool: The flag to check if detection is enabled
  """
  message = request_message(drone)
  print(message)
  print("Waiting to reach Waypoint!!")
  if detection:
    while(message.wp_dist>=0.5):
      message = request_message(drone)
      start_time=time.time()
      boxes=stream_(stream_array,return_=True)
      end_time=time.time()
      print(f"FPS:{1/(end_time-start_time)}")
      
      if(len(boxes)!=0):
        print("AlERT!!!! Drowning Person Detected. Aligning......")
        pError=0
        pError_y=0
        pError_z=0
        while True:
          if (len(boxes)!=0):
            pError, pError_y, pError_z, speed_y, speed=IBVS_algo(drone, 320, pError, pError_y, pError_z, 320,boxes,stream_array)
          else:
            drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,drone.target_system,drone.target_component,
              mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,int(0b010111000111),0,0,0,-speed_y,0,0,0,0,0,0,speed))

          boxes=stream_(stream_array,return_=True)
  else:
    while(message.wp_dist>=0.5):
      message = request_message(drone)
      stream_(stream_array)
  print("Waypoint reached!!")   

def movement(drone,stream_array,count,x=0,y=0,gps=[0,0],type="gps",alt=None):
  """
  Function to move the drone to a specific location

  Args:
    drone: obj: The connection object of the drone
    stream_array: list: The list of the stream objects
    count: int: The count of the waypoints
    x: float: The distance to move in x direction
    y: float: The distance to move in y direction
    gps: list: The GPS coordinates
    type: str: The type of movement
    alt: float: The altitude 
  """
  lat=gps[0]
  lon=gps[1]
  if type=='gps':
    drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, drone.target_system,
                    drone.target_component, 6, int(0b010111111000), int(lat* 10 ** 7), int(lon * 10 ** 7), alt, 0, 0, 0, 0, 0, 0, 0, 0))
    wait_wp(drone,stream_array)

  else:
    drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,drone.target_system,drone.target_component,
                            9,int(0b010111111000),x,y,0,0,0,0,0,0,0,0,0))

    wait_wp(drone,stream_array,count)
  drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,drone.target_system,drone.target_component,
                              9,int(0b010111111000),0,0,0,0,0,0,0,0,0,0,0))

def start_mission(drone,stream_array,count,coord_list,sa_list,altitude,alpha,beta,land=0):
  """
  Function to start the mission

  Args:
    drone: obj: The connection object of the drone
    stream_array: list: The list of the stream objects
    count: int: The count of the waypoints
    coord_list: list: The list of GPS coordinates
    sa_list: list: The list of GPS coordinates of the searching area
    altitude: float: The altitude of the drone
    alpha: float: The horizontal FOV
    beta: float: The vertical FOV
    land: bool: The flag to check if the drone should land

  """
  for i in range(len(coord_list)):
    print(f"Going to Waypoint: {coord_list[i]}")
    movement(drone,stream_array,count,gps=coord_list[i],alt=altitude)
  center_gps=gps_midpoint(sa_list) 
  print(f"The center GPS is: {center_gps}")
  radius=haversine_distance(center_gps, sa_list[0])
  radius=np.clip(radius,0,4)
  print(f"The radius is: {radius} m")
  print(f"Moving Towards Center GPS: {center_gps}, Descending to Height = {altitude} m") 
  print("Changing speed to 30 cm/s")
  drone.mav.param_set_send(drone.target_system, drone.target_component,
                       b'WPNAV_SPEED', 30,mavutil.mavlink.MAV_PARAM_TYPE_REAL32) 
  spiral_algo(drone,stream_array,count,altitude,alpha,beta,center_gps,radius,land=0)

  return

def spiral_algo(drone,stream_array,count,altitude,alpha,beta,center_gps,radius,land=0):

  """
  Function to run the spiral algorithm

  Args:
    drone: obj: The connection object of the drone
    stream_array: list: The list of the stream objects
    count: int: The count of the waypoints
    altitude: float: The altitude of the drone
    alpha: float: The horizontal FOV
    beta: float: The vertical FOV
    center_gps: list: The GPS coordinates of the center
    radius: float: The radius of the spiral
    land: bool: The flag to check if the drone should land
  """

  diameter = 2*radius

  cell_w=2*altitude*math.tan(alpha/2.0)
  cell_l=2*altitude*math.tan(beta/2.0)
  print(f'Width: {cell_w}, Length = {cell_l}')

  no_of_rows = math.ceil(diameter / cell_w)
  no_of_columns = math.ceil(diameter / cell_l)

  # making grid rows and columns equal
  if (no_of_rows > no_of_columns):
    no_of_columns = no_of_rows
  else:
    no_of_rows = no_of_columns

  # making rows and columns odd
  if (no_of_columns % 2 == 0):
    no_of_rows += 1
    no_of_columns += 1

  print(f"Columns = {no_of_columns}")
  movement(drone,stream_array,count,gps=center_gps,alt=altitude)

  for i in range(no_of_columns):
    if not land:
      if i%2==0:
        movement(drone,stream_array,count,(i+1)*cell_w,0,type='movement')
        movement(drone,stream_array,count,0,(i+1)*cell_l,type='movement')
      else:
        movement(drone,stream_array,count,-(i+1)*cell_w,0,type='movement')
        movement(drone,stream_array,count,0,-(i+1)*cell_l,type='movement')
    else:
        break
  return

