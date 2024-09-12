import time
import cflib.crtp
import winsound
import cv2
import csv

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig
from threading import Lock



# URI to the Crazyflie to connect to
uri_1 = 'radio://0/80/2M/E7E7E7E701' # Drone's uri
uri_2 = 'radio://0/80/2M/E7E7E7E7E9' # Tag's uri

start_x = float(1.0)  # initial pos_X of the drone; unit: m
start_y = float(0.0)  # initial pos_y of the drone; unit: m   #Left: 0.0; Right: -0.4
init_Vel = 0.3  # initial velocity

# f = open("conditions.txt", "rt")
# data = f.read()

# Read configuration from file
with open("conditions.txt", "rt") as f:
    data = f.read()
 
data_split = data.split()
h0 = float(data_split[0]) # Eyes level height (unit: meter)
T_fly_out = float(data_split[1]) # Fly out time (unit: sec)
V_fly_out = float(data_split[2]) # Fly out velocity (unit: m/s)
fly_out_x = float(data_split[3]) # Fly out position in x-axis (unit: m) 
fly_out_y = float(data_split[4]) # Fly out position in y-axis (unit: m)
fly_out_z = float(data_split[5]) # Fly out position in z-axis (unit: m)

offset = float(0.37)  # offset for the initial take off height; unit: m
init_H = h0 - offset  # Initial drone's height; unit: m


position_estimate_1 = [0, 0, 0]  # Drone's pos
position_estimate_2 = [0, 0, 0]  # LS's pos


# CSV file setup
filename = "record.csv"
fields = ['timestamp', 'pos1_x', 'pos1_y', 'pos1_z', 'pos2_x', 'pos2_y', 'pos2_z']
lock = Lock()

# Write header if the file does not exist
try:
    with open(filename, 'x', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(fields)
except FileExistsError:
    pass


# # Positioning Callback Section
def log_pos_callback_1(uri_1, timestamp, data, logconf_1):
    global position_estimate_1
    position_estimate_1[0] = data['kalman.stateX']
    position_estimate_1[1] = data['kalman.stateY']
    position_estimate_1[2] = data['kalman.stateZ']
    print("{}: {} is at pos: ({}, {}, {})".format(timestamp, uri_1, position_estimate_1[0], position_estimate_1[1], position_estimate_1[2]))

    # Append to CSV file if both estimates are available
    with lock, open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([timestamp, position_estimate_1[0], position_estimate_1[1], position_estimate_1[2], position_estimate_2[0], position_estimate_2[1], position_estimate_2[2]])
                                            

def log_pos_callback_2(uri_2, timestamp, data, logconf_2):
    global position_estimate_2
    position_estimate_2[0] = data['kalman.stateX']
    position_estimate_2[1] = data['kalman.stateY']
    position_estimate_2[2] = data['kalman.stateZ']
    print("{}: {} is at pos: ({}, {}, {})".format(timestamp, uri_2, position_estimate_2[0], position_estimate_2[1], position_estimate_2[2]))

    # Append to CSV file if both estimates are available
    with lock, open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([timestamp, position_estimate_1[0], position_estimate_1[1], position_estimate_1[2], position_estimate_2[0], position_estimate_2[1], position_estimate_2[2]])
                 

# # # Crazyflie Motion (using MotionCommander)

def drone_move_pc(scf1): # default take-off height = 0.3 m
    
    t_init = time.time()

    # BEEP before start
    print("Task Begin!")
    frequency = 1000  # Set Frequency To 2500 Hertz
    duration = 250  # Set Duration To 250 ms == 0.25 second
    winsound.Beep(frequency, duration)

    with PositionHlCommander(
            scf1,
            x=start_x, y=start_y, z=0.0,
            default_velocity=init_Vel,
            default_height=0.3,
            controller=PositionHlCommander.CONTROLLER_PID) as pc:

        t_take_off = time.time() - t_init
        print("start taking off at ", t_take_off)   
    
        ## Go up: h0 meter (at the eyes level)
        pc.up(init_H)
        time.sleep(init_H/init_Vel)
        print(pc.get_position())
              

        ## Delay before flying out
        time.sleep(T_fly_out)

        ## BEEP before flying out
        # print("Beep!!")
        # frequency = 1000  # Set Frequency To 2500 Hertz
        # duration = 250  # Set Duration To 250 ms == 0.25 second
        # winsound.Beep(frequency, duration)

        # time.sleep(0.1)

        # Timestamp before flying out
        t_fo = time.time() - t_init
        print("t fly out: ", t_fo)

        ## Fly out
        print("Start flying out")
        pc.move_distance(fly_out_x, fly_out_y, fly_out_z, velocity=V_fly_out) 
        # mc.up(fly_out_z, velocity=V_fly_out)

        t_f1 = time.time() - t_init
        print("t after fly out: ", t_f1)
       
        
        ## Delay 0.5 sec before landing
        time.sleep(0.5)

    

if __name__ == '__main__':

    # # initializing Crazyflie 
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri_2, cf=Crazyflie(rw_cache='./cache')) as scf_2:
        logconf_2 = LogConfig(name='Position', period_in_ms=500)
        logconf_2.add_variable('kalman.stateX', 'float')
        logconf_2.add_variable('kalman.stateY', 'float')
        logconf_2.add_variable('kalman.stateZ', 'float')            
        scf_2.cf.log.add_config(logconf_2)
        logconf_2.data_received_cb.add_callback( lambda timestamp, data, logconf_2: log_pos_callback_2(uri_2, timestamp, data, logconf_2) )

        with SyncCrazyflie(uri_1, cf=Crazyflie(rw_cache='./cache')) as scf_1:
            logconf_1 = LogConfig(name='Position', period_in_ms=500)
            logconf_1.add_variable('kalman.stateX', 'float')
            logconf_1.add_variable('kalman.stateY', 'float')
            logconf_1.add_variable('kalman.stateZ', 'float')        
            scf_1.cf.log.add_config(logconf_1)
            logconf_1.data_received_cb.add_callback( lambda timestamp, data, logconf_1: log_pos_callback_1(uri_1, timestamp, data, logconf_1) )

            logconf_1.start()
            logconf_2.start()

            time.sleep(3)

            # Perform the catching task
            drone_move_pc(scf_1)

            time.sleep(3)

            logconf_1.stop()
            logconf_2.stop()




