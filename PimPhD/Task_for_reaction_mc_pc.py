import time
import logging
import cflib.crtp
import math
import csv

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
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

'''
f = open("conditions.txt", "rt")
data = f.read()

data_split = data.split()
h0 = float(data_split[0]) # Eyes level height (unit: meter)
T_fly_out = float(data_split[1]) # Fly out time (unit: sec)
V_fly_out = float(data_split[2]) # Fly out velocity (unit: m/s)
fly_out_x = float(data_split[3]) # Fly out position in x-axis (unit: m) 
fly_out_y = float(data_split[4]) # Fly out position in y-axis (unit: m)
fly_out_z = float(data_split[5]) # Fly out position in z-axis (unit: m)

dist = math.sqrt(fly_out_x*fly_out_x + fly_out_y*fly_out_y + fly_out_z*fly_out_z)
'''

position_estimate_1 = [0, 0, 0]  # Drone's pos

# CSV file setup (s1_u1.csv)
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
     

# # # Crazyflie Motion (using MotionCommander)
# drone stand's height: 0.15 m; ball and stick: 0.08 m (count from drone's legs)

'''
def drone_move_mc(scf): # default take-off height = 0.3 m
    
    t_init = time.time()

    with MotionCommander(scf) as mc:

        t_take_off = time.time()
        print("start taking off at ", t_take_off-t_init)   
    
        ## Go up: h0 meter (at the eyes level)
        mc.up(h0 - 0.37, velocity=take_off_vel) # eye_level (h0)-init_takeoff(0.3)-drone_standing(0.15)-ball&stick(0.08)

        t1 = time.time()
        print("t take off: ", t1-t_take_off)

        ## hover before flying out      
        mc.stop()

        t0 = time.time()

        ## hovering time
        time.sleep(T_fly_out)

        t1 = time.time() - t0
        print("t wait: ", t1)

        # Timestamp before flying out
        t_fo = time.time() - t_take_off
        print("t fly out: ", t_fo)

        ## Fly out
        print("Start flying out")
        mc.move_distance(fly_out_x, fly_out_y, fly_out_z, velocity=V_fly_out) 
        # mc.up(fly_out_z, velocity=V_fly_out)

        t_f1 = time.time() - t_take_off
        print("t after fly out: ", t_f1)
       
        mc.stop()
        
        ## Delay 1 sec before landing
        time.sleep(1)


# # # Crazyflie Motion (using MotionCommander)

def drone_move_pc(scf): # default take-off height = 0.3 m
    
    t_init = time.time()

    with PositionHlCommander(
        scf,
        x=start_x, y=start_y, z=start_z,
        default_velocity=take_off_vel,
        default_height=0.3,
        controller=PositionHlCommander.CONTROLLER_PID) as pc:

        t_take_off = time.time()
        print("start taking off at ", t_take_off-t_init)   
    
        ## Go up: h0 meter (at the eyes level)
        pc.up(h0 - 0.37, velocity=take_off_vel)
        # time.sleep((h0 - 0.53)/take_off_vel)
        print(pc.get_position())
        
        t_prep = time.time()
        print("t take off: ", t_prep-t_take_off)

        ## Delay before flying out (*** maybe make it random for training purpose)
        time.sleep(T_fly_out)

        t_wait = time.time() - t_prep
        print("t wait: ", t_wait)

        # Timestamp before flying out
        t_fo = time.time() - t_take_off
        print("t fly out: ", t_fo)

        ## Fly out
        print("Start flying out")
        pc.move_distance(fly_out_x, fly_out_y, fly_out_z, velocity=V_fly_out) 
        # time.sleep(dist/V_fly_out)

        t_f1 = time.time() - t_take_off
        print("t after fly out: ", t_f1)
        
        ## Delay 1 sec before landing
        time.sleep(1)
'''


def accelerate_test(scf):
    with PositionHlCommander(
        scf,
        x=start_x, y=start_y, z=start_z,
        default_velocity=take_off_vel,
        default_height=de_h,
        controller=PositionHlCommander.CONTROLLER_PID) as pc:

        time.sleep(3)

        # pc.forward(0.8, velocity=task_vel)
        # time.sleep((0.6)/take_off_vel)
        # print(pc.get_position())

        pc.right(0.8, velocity=task_vel)
        # # time.sleep((0.7)/take_off_vel)
        # print(pc.get_position())  

        # pc.up(0.4, velocity=task_vel)
        # # time.sleep((0.4)/take_off_vel)
        # print(pc.get_position())

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
        # drone_move_mc(scf)
        # drone_move_pc(scf)
        accelerate_test(scf)
        
        time.sleep(3)

        logconf_1.stop()



