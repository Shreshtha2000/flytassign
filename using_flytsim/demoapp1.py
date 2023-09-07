import time
from flyt_python import api

drone = api.navigation(timeout=120000)

time.sleep(3)

print("taking off")

drone.takeoff(5.0)

print("takeoff done")

print("Following square trajectory")

drone.position_set(6.5,0,-0.1,relative=True)
time.sleep(3)
drone.position_set(0,6.5,-0.1,relative=True)
time.sleep(3)
drone.position_set(-6.5,0,-0.1,relative=True)
time.sleep(3)
drone.position_set(0,-6.5,0,relative=True)

print("square done")

print("landing")
time.sleep(5)

drone.land(async=False)

drone.disconnect()
