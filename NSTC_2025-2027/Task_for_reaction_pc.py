import time
import logging
import cflib.crtp
import csv

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander


# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E703'

take_off_vel = 0.8      # take off velocity; unit: m/s
task_vel = 0.8
offset = 0.18         # drone height offset; unit: m
tar_h = 1.30            # target height
ball_length = 0.1     # hanging part lenght from the LH deck; unit: m
de_h = tar_h - offset + ball_length # default height; unit: m

start_x = float(0.0)  # initial pos_X of the drone; unit: m
start_y = float(0.0)  # initial pos_y of the drone; unit: m
start_z = float(0.0)  # initial pos_z of the drone; unit: m

position_estimate_1 = [0, 0, 0]  # Drone's pos

# CSV file setup (s1_uw_1.csv)
# filename = "s38_fw_1.csv" 
filename = "test.csv"
fields = ['timestamp', 'pos_x', 'pos_y', 'pos_z']

# Write header if the file does not exist
try:
    with open(filename, 'x', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(fields)
except FileExistsError:
    pass


# # Positioning Callback Section
def log_pos_callback_1(uri, timestamp, data, logconf_1):
    global position_estimate_1
    position_estimate_1[0] = data['kalman.stateX']
    position_estimate_1[1] = data['kalman.stateY']
    position_estimate_1[2] = data['kalman.stateZ']
    # print("{}: {} is at pos: ({}, {}, {})".format(timestamp, uri, position_estimate_1[0], position_estimate_1[1], position_estimate_1[2]))

    # Append to CSV file if both estimates are available
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([timestamp, position_estimate_1[0], position_estimate_1[1], position_estimate_1[2]])
     

def accelerate_test(scf):
    with PositionHlCommander(
        scf,
        x=start_x, y=start_y, z=start_z,
        default_velocity=take_off_vel,
        default_height=de_h,
        controller=PositionHlCommander.CONTROLLER_PID) as pc:

        time.sleep(3)

 # # Uncomment the drone's fly-out direction of the current trial
        # pc.forward(0.8, velocity=task_vel)   
        pc.right(0.8, velocity=task_vel)
        # pc.left(0.8, velocity=task_vel)
        # pc.up(0.4, velocity=task_vel)

        time.sleep(1)

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':

    # # initializing Crazyflie 
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        logconf_1 = LogConfig(name='Position', period_in_ms=10)
        logconf_1.add_variable('kalman.stateX', 'float')
        logconf_1.add_variable('kalman.stateY', 'float')
        logconf_1.add_variable('kalman.stateZ', 'float')        
        scf.cf.log.add_config(logconf_1)
        logconf_1.data_received_cb.add_callback( lambda timestamp, data, logconf_1: log_pos_callback_1(uri, timestamp, data, logconf_1) )

        logconf_1.start()

        time.sleep(3)

        # # Perform the movement
        accelerate_test(scf)
        
        time.sleep(3)

        logconf_1.stop()



