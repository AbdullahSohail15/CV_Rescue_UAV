import time
from pymavlink import mavutil
import cv2

def change_to_RTL(drone,stream_array):
  mode_id = drone.mode_mapping()['RTL']
  drone.mav.set_mode_send(
  drone.target_system,
  mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
  mode_id)
  print("RTL Mode Set, Returning to Home...")
  stream_(stream_array)
  msg=request_message(drone,type="GLOBAL_POSITION_INT")
  count=0
  while msg.relative_alt>=1000:
    stream_(stream_array)
    msg=request_message(drone,type="GLOBAL_POSITION_INT")
  stream_(stream_array,end=True)

def request_message(vehicle,type='NAV_CONTROLLER_OUTPUT'):
    """
    This function requests a message from the drone

    Args:
    vehicle: obj: The connection object of the drone
    type: str: The type of message to request

    Returns:
    msg: obj: The message object
    """
    if type=='NAV_CONTROLLER_OUTPUT':
        vehicle.mav.command_long_send(vehicle.target_system,vehicle.target_component,mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,62,0,0,0,0,0,0)
        msg=vehicle.recv_match(type=type, blocking=True)
    elif type=='LOCAL_POSITION_NED':
        vehicle.mav.command_long_send(vehicle.target_system,vehicle.target_component,mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,32,0,0,0,0,0,0)
        msg=vehicle.recv_match(type=type, blocking=True)
    elif type=='COMMAND_ACK':
        vehicle.mav.command_long_send(vehicle.target_system,vehicle.target_component,mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,77,0,0,0,0,0,0)
        msg=vehicle.recv_match(type=type, blocking=True)
    elif type=='HEARTBEAT':
        vehicle.mav.command_long_send(vehicle.target_system,vehicle.target_component,mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,0,0,0,0,0,0,0)
        msg=vehicle.recv_match(type=type, blocking=True)
    elif type=='ALTITUDE':
        vehicle.mav.command_long_send(vehicle.target_system,vehicle.target_component,mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,141,0,0,0,0,0,0)
        msg=vehicle.recv_match(type=type,blocking=True)
    elif type=='GLOBAL_POSITION_INT':
        vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,33,0,0,0,0,0,0)
        msg=vehicle.recv_match(type=type, blocking=True)
    else:
        print("Invalid Type")
        msg=None
    return msg

def arm(drone):
  """
  This function arms the drone so that it can takeoff

  Args:
  drone: obj: The connection object of the drone
  """
  drone.mav.command_long_send(drone.target_system,drone.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0) 
  msg = request_message(drone,type='COMMAND_ACK')
  print(f"ARM ACK : {msg}")


def takeoff(drone,altitude,stream_array):
  """
  This function takes off the drone to a specified altitude

  Args:
  drone: obj: The connection object of the drone
  altitude: int: The altitude to which the drone should takeoff
  """
  drone.mav.command_long_send(drone.target_system,drone.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,altitude)
  msg = request_message(drone,type="GLOBAL_POSITION_INT")
  print(f"Drone Altitude: {msg.relative_alt/1000}")
  stream_(stream_array)
  while(msg.relative_alt<=(altitude*1000-100)):
    stream_(stream_array)
    msg = request_message(drone,type="GLOBAL_POSITION_INT")
    stream_(stream_array)
    print(f"Drone Altitude: {msg.relative_alt/1000}")
  print(f"Reached Height of {altitude} m")
  stream_(stream_array)



def initialize_drone(connection_string="/dev/ttyACM0"):
  """
  This function initializes the drone and returns the connection object

  Args:
  connection_string: str: The connection string of the drone

  Returns:
  the_connection: obj: The connection object of the drone
  """
  the_connection=mavutil.mavlink_connection(connection_string)
  the_connection.wait_heartbeat()
  print("Heartbeat from system (system %u component %u)" %
    (the_connection.target_system, the_connection.target_component))
  return the_connection

def wait4start(vehicle):
  """
  This function waits for the drone to activate guided mode
  
  Args:
    vehicle: obj: The connection object of the drone
  """
  # Wait a heartbeat before sending commands
  print("Waiting for heartbeat")
  vehicle.wait_heartbeat()
  print("Heartbeat Checked")
  while True:
    msg =request_message(vehicle,type='HEARTBEAT')
    print("The message is {}".format(msg))
    if msg:
      mode = mavutil.mode_string_v10(msg)
      print("Waiting for Guided")
      time.sleep(1)
      if mode=='GUIDED':
        print("Guided Mode")
        break
  return

def stream_(stream_array,return_=False,end=False):
  yolo,cap,stream,out=stream_array
  frame=preprocessor(cap)
  frame,boxes=yolo.get_bbox(frame,0.3,["Drowning"])
  stream.frame=frame
  out.write(frame)
  if end:
    out.release()
    cv2.destroyAllWindows()

  if return_:
    return boxes
  

def preprocessor(cap):
  _,frame=cap.read()
  resized_frame = cv2.resize(frame,(640,640))
  return resized_frame