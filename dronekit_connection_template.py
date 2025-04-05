from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import argparse

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    
    connection_string = args.connect
    
    # if we did not launch simulated drone manually
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()
    
    vehicle = connect(connection_string, wait_ready=True)
    
    return vehicle

def arm_and_takeoff(targetHeight):
    
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)
    
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    
    print("Taking off!")
    vehicle.simple_takeoff(targetHeight)
    
    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= targetHeight * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_velocity(vehicle, vx, vy, vz, duration):
    # send velocity commands to vehicle
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # Bitmask
        0, 0, 0,              # Position
        vx, vy, vz,          # Velocity
        0, 0, 0,             # Acceleration
        0, 0
    )
    for _ in range(duration):
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(1)

def send_global_velocity(vehicle, vx, vy, vz, duration):
   
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        0b0000111111000111,  # Bitmask
        0, 0, 0,              # Position
        vx, vy, vz,          # Velocity
        0, 0, 0,             # Acceleration
        0, 0
    )
    for _ in range(duration):
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(1)

def move_up_down(vehicle, velocity, duration, method='local'):
    
    # make the drone move
    
    print("Moving up")
    if method == 'local':
        send_velocity(vehicle, 0, 0, -velocity, duration)
    else:
        send_global_velocity(vehicle, 0, 0, -velocity, duration)
    
    print("Moving down")
    if method == 'local':
        send_velocity(vehicle, 0, 0, velocity, duration)
    else:
        send_global_velocity(vehicle, 0, 0, velocity, duration)

if __name__ == "__main__":
    vehicle = connectMyCopter()
    
    arm_and_takeoff(5)
    
    move_up_down(vehicle, velocity=0.5, duration=3, method='local')
    
    vehicle.close()


## another posible commands
# vehicle.airspeed = 7

#vehicle.simple_goto(wpl)
