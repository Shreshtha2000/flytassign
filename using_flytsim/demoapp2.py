import time
from flyt_python import api

drone = api.navigation(timeout=120000)

time.sleep(3)
length = input("enter triangle side")

print("taking off")

drone.takeoff(5.0)

print("takeoff done")

print("Following triangle trajectory")

drone.position_set(length*0.866025,length*0.5,0,relative=True)
time.sleep(3)
drone.position_set(-1*length*0.866025,length*0.5,0,relative=True)
time.sleep(3)
drone.position_set(0,-1*length,0,relative=True)


print("triangle done")

print("landing")
time.sleep(5)

drone.land(async=False)

drone.disconnect()
