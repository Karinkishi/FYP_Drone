from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# Connect to the Vehicle (in this case a UDP endpoint)
vehicle = connect('/dev/serial0', wait_ready=True, baud=57600)

# vehicle is an instance of the Vehicle class
print ("Autopilot Firmware version: %s" % vehicle.version)
print ("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
print ("Global Location: %s" % vehicle.location.global_frame)
print ("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print ("Local Location: %s" % vehicle.location.local_frame)    #NED
print ("Attitude: %s" % vehicle.attitude)
print ("Velocity: %s" % vehicle.velocity)
print ("GPS: %s" % vehicle.gps_0)


print ("Heading: %s" % vehicle.heading)
print ("Is Armable?: %s" % vehicle.is_armable)
print ("System status: %s" % vehicle.system_status.state)
print ("Mode: %s" % vehicle.mode.name)    # settable
print ("Armed: %s" % vehicle.armed)    # settable

'''
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.mode.name=='GUIDED' and not vehicle.armed and not api.exit:
    print (" Getting ready to take off ...")
    time.sleep(1)

'''

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)
    """
    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")# vehicle.mode = VehicleMode("ALT_HOLD")
    vehicle.flush()
    vehicle.armed   = True
    
    time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            '''
            vehicle.mode = VehicleMode("LOITER")# vehicle.mode = VehicleMode("ALT_HOLD")
            vehicle.flush()
            '''
            break
        time.sleep(1)

arm_and_takeoff(1) #meter
time.sleep(5)
# Send the land command
print ("Copter will now land..")
#time.sleep(3)
# Slowly descend to the ground
vehicle.mode = VehicleMode("LOITER") # vehicle.mode = VehicleMode("ALT_HOLD")
vehicle.flush()
vehicle.channels.overrides['3'] = 450
time.sleep(3)
'''
descent_rate = 0.1  # meters per second
while vehicle.location.global_relative_frame.alt > 0.05:
    vehicle.simple_goto(vehicle.location.global_frame, groundspeed=descent_rate)
    time.sleep(1)
'''
vehicle.mode = VehicleMode("LAND")
vehicle.flush()

time.sleep(5)
# Disarm the vehicle
vehicle.armed = False
vehicle.flush()

# Close the vehicle connection
vehicle.close()
