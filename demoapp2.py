from pymavlink import mavutil
# import mavlink
import time
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
import math
print(master.wait_heartbeat())



def give_command(mode):
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)


def move_command(side_length):
    print("Reaching vertice one")
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0,master.target_system, master.target_component,1,0b0000111111111000,0,side_length,-10,0,0,0,0,0,0,0,0))
    time.sleep(10)
    print("Reaching vertice two")
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0,master.target_system, master.target_component,1,0b0000111111111000,0.866*side_length,side_length/2,-10,0,0,0,0,0,0,0,0))
    time.sleep(10)
    print("Reaching vertice three")
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0,master.target_system, master.target_component,1,0b0000111111111000,0,0,-10,0,0,0,0,0,0,0,0))
    time.sleep(10)

print("Mode guided")
give_command("GUIDED")

# while True:
#   msg = master.recv_match()
#   if msg!=None:
#       if msg.get_type()=="GLOBAL_POSITION_INT":
#           lat = msg.lat
#           lon = msg.lon
#           break
time.sleep(5)
print("arming")
master.mav.command_long_send(master.target_system,master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM ,0,1,0,0,0,0,0,0)
time.sleep(5)

master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,10)
print("Taking off")
time.sleep(10)
print("takeoff done")
move_command(10)
# time.sleep(10)
print("Landing")
master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_NAV_LAND,0,0,0,0,0,0,0,0)
master.close()