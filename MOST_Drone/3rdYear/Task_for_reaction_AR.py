import time
import cflib.crtp
import winsound
import threading
import csv
import math
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig
from threading import Lock



# URI to the Crazyflie to connect to
uri_1 = 'radio://0/80/2M/E7E7E7E701' # Drone's uri
uri_2 = 'radio://0/80/2M/E7E7E7E7E8' # Tag's uri


start_x = float(0.0)  # initial pos_X of the drone; unit: m
start_y = float(0.0)  # initial pos_y of the drone; unit: m
start_z = float(0.0)  # initial pos_z of the drone; unit: m

init_Vel = float(0.5)   # initial velocity
h0 = 1.4                # Eyes level height (unit: meter)
T_fly_out = float(3)    # Fly out time (unit: sec)
V_fly_out = float(1)    # Fly out velocity (unit: m/s)


fw = float(0.8)  # Fly out position in x-axis (unit: m); 0.8
rw = float(-0.8) # Fly out position in y-axis (unit: m); -0.8
uw = float(0.4)  # Fly out position in z-axis (unit: m); 0.4

# fly_out_1 = [fw, 0, 0]
# fly_out_2 = [0, rw, 0]
# fly_out_3 = [0, 0, uw]

# fly_out_1 = [0, rw, 0]
# fly_out_2 = [fw, 0, 0]
# fly_out_3 = [0, 0, uw]

fly_out_1 = [0, 0, uw]
fly_out_2 = [0, rw, 0]
fly_out_3 = [fw, 0, 0]

# fly_out_1 = [0, rw, 0]
# fly_out_2 = [0, 0, uw]
# fly_out_3 = [fw, 0, 0]

# fly_out_1 = [fw, 0, 0]
# fly_out_2 = [0, 0, uw]
# fly_out_3 = [0, rw, 0]

# fly_out_1 = [0, 0, uw]
# fly_out_2 = [fw, 0, 0]
# fly_out_3 = [0, rw, 0]


position_estimate_1 = [0, 0, 0]  # Drone's pos
position_estimate_2 = [0, 0, 0]  # LS's pos

d_th = float(0.3)   # threshold

# # 1: FW; 2: RW; 3: UW
# direct = ['0', '0', '0', '0', '0']
# dir1 = ['1', '2','1', '3', '2']
# dir2 = [3,1,1,2,3]
# dir3 = [2,3,1,3,2]

# direct = dir1

# CSV file setup
filename = "S1_t3.csv"
fields = ['timestamp', 'pos1_x', 'pos1_y', 'pos1_z', 'pos2_x', 'pos2_y', 'pos2_z', 'distance']

proximity_filename = "proximity_log_s1_t3.csv"
proximity_fields = ['timestamp', 'distance']

lock = Lock()

# Write header if the file does not exist
try:
    with open(filename, 'x', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(fields)
except FileExistsError:
    pass

try:
    with open(proximity_filename, 'x', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(proximity_fields)
except FileExistsError:
    pass

# # Positioning Callback Section
def log_pos_callback_1(uri_1, timestamp, data, logconf_1):
    global position_estimate_1, position_estimate_2
    position_estimate_1[0] = data['kalman.stateX']
    position_estimate_1[1] = data['kalman.stateY']
    position_estimate_1[2] = data['kalman.stateZ']
    # print("{}: {} is at pos: ({}, {}, {})".format(timestamp, uri_1, position_estimate_1[0], position_estimate_1[1], position_estimate_1[2]))

    distance = math.sqrt(
        (position_estimate_1[0] - position_estimate_2[0]) ** 2 +
        (position_estimate_1[1] - position_estimate_2[1]) ** 2 +
        (position_estimate_1[2] - position_estimate_2[2]) ** 2)
    
    # print("{} is at distance: {}".format(timestamp, distance))

    # Append to CSV file if both estimates are available
    with lock, open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([timestamp, position_estimate_1[0], position_estimate_1[1], position_estimate_1[2], position_estimate_2[0], position_estimate_2[1], position_estimate_2[2], distance])

    check_proximity(timestamp)                                        


def log_pos_callback_2(uri_2, timestamp, data, logconf_2):
    global position_estimate_2
    position_estimate_2[0] = data['kalman.stateX']
    position_estimate_2[1] = data['kalman.stateY']
    position_estimate_2[2] = data['kalman.stateZ']
    # print("{}: {} is at pos: ({}, {}, {})".format(timestamp, uri_2, position_estimate_2[0], position_estimate_2[1], position_estimate_2[2]))

    # # Append to CSV file if both estimates are available
    # with lock, open(filename, 'a', newline='') as csvfile:
    #     writer = csv.writer(csvfile)
    #     writer.writerow([timestamp, position_estimate_1[0], position_estimate_1[1], position_estimate_1[2], position_estimate_2[0], position_estimate_2[1], position_estimate_2[2]])
    

def check_proximity(timestamp):
    global position_estimate_1, position_estimate_2
    # Calculate the Euclidean distance
    distance = math.sqrt(
        (position_estimate_1[0] - position_estimate_2[0]) ** 2 +
        (position_estimate_1[1] - position_estimate_2[1]) ** 2 +
        (position_estimate_1[2] - position_estimate_2[2]) ** 2)

    # # If within the threshold, log it to the proximity CSV
    # if distance < d_th:
    #     # winsound.PlaySound('Success.wav', winsound.SND_FILENAME)
    #     # print("good job")   
    with lock, open(proximity_filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([timestamp, distance])

def drone_move_pc(scf1, event2): # default take-off height = 0.3 m

    # BEEP before start
    print("start!!!")
    winsound.PlaySound('game-start-6104.wav', winsound.SND_FILENAME)
    

    with PositionHlCommander(
            scf1,
            x=start_x, y=start_y, z=start_z,
            default_velocity=init_Vel,
            default_height=h0,
            controller=PositionHlCommander.CONTROLLER_PID) as pc:

        time.sleep(h0/init_Vel)
        print(pc.get_position())

        # # # 1st round

        ## Delay before flying out
        time.sleep(T_fly_out)
        
        print("1st Trial")
        # frequency = 1000  # Set Frequency To 2500 Hertz
        # duration = 250  # Set Duration To 250 ms == 0.25 second
        # winsound.Beep(frequency, duration) 

        pc.move_distance(fly_out_1[0], fly_out_1[1], fly_out_1[2], velocity=V_fly_out)
        print(pc.get_position())
        time.sleep(1)

        # return to starting position
        pc.go_to(start_x, start_y, h0, velocity=init_Vel)
        print(pc.get_position())
        time.sleep(1)


        # # # 2nd round

        ## Delay before flying out
        time.sleep(T_fly_out)
        
        print("2nd Trial")
        # frequency = 1000  # Set Frequency To 2500 Hertz
        # duration = 250  # Set Duration To 250 ms == 0.25 second
        # winsound.Beep(frequency, duration) 

        pc.move_distance(fly_out_2[0], fly_out_2[1], fly_out_2[2], velocity=V_fly_out)
        print(pc.get_position())
        time.sleep(1)

        # return to starting position
        pc.go_to(start_x, start_y, h0, velocity=init_Vel)
        print(pc.get_position())
        time.sleep(1)

        
        # # # 3rd round

        ## Delay before flying out
        time.sleep(T_fly_out)
        
        print("3rd Trial")
        # frequency = 1000  # Set Frequency To 2500 Hertz
        # duration = 250  # Set Duration To 250 ms == 0.25 second
        # winsound.Beep(frequency, duration) 

        pc.move_distance(fly_out_3[0], fly_out_3[1], fly_out_3[2], velocity=V_fly_out)
        print(pc.get_position())
        time.sleep(1)

        # return to starting position
        pc.go_to(start_x, start_y, h0, velocity=init_Vel)
        print(pc.get_position())
        time.sleep(1)

        ## Delay 1 sec before landing
        time.sleep(1)

        ## set the event for turning off the sound feedback process
        event2.set()


def position_state_change(event1, event2):
    print("position thread start")
    global position_estimate_1, position_estimate_2
    while not event2.is_set():
        # if abs((position_estimate_2[2]+0.2)-position_estimate_1[2]) > d_th or abs((position_estimate_3[2]+0.2)-position_estimate_1[2]) > d_th:
        #     # print("---Wrist Sensor is outbounded---")
        #     event1.set()
        distance = math.sqrt(
        (position_estimate_1[0] - position_estimate_2[0]) ** 2 +
        (position_estimate_1[1] - position_estimate_2[1]) ** 2 +
        (position_estimate_1[2] - position_estimate_2[2]) ** 2)
        
        print(distance)

        # If within the threshold, log it to the proximity CSV
        if distance < d_th:
            event1.set()
            # winsound.PlaySound('Success.wav', winsound.SND_FILENAME)
            print("Right!")
            
        else:
            event1.clear()
            # print("It's Pam!!!!!")
            # event1.clear()

def sound_feedback(event1, event2):
    print("sound thread started")
    while not event2.is_set():
        if event1.is_set()==True:
            print("Great!")
            frequency = 2500  # Set Frequency To 2500 Hertz
            duration = 500  # Set Duration To 250 ms == 0.25 second
            winsound.Beep(frequency, duration)
        else:
            # print("Nothing")
            pass
            
        time.sleep(0.1)


if __name__ == '__main__':

    # # initializing the queue and event object
    e1 = threading.Event()
    e2 = threading.Event()

    # # initializing Crazyflie 
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri_2, cf=Crazyflie(rw_cache='./cache')) as scf_2:
        logconf_2 = LogConfig(name='Position', period_in_ms=10) # sampling at 100Hz
        logconf_2.add_variable('kalman.stateX', 'float')
        logconf_2.add_variable('kalman.stateY', 'float')
        logconf_2.add_variable('kalman.stateZ', 'float')            
        scf_2.cf.log.add_config(logconf_2)
        logconf_2.data_received_cb.add_callback( lambda timestamp, data, logconf_2: log_pos_callback_2(uri_2, timestamp, data, logconf_2) )

        with SyncCrazyflie(uri_1, cf=Crazyflie(rw_cache='./cache')) as scf_1:
            logconf_1 = LogConfig(name='Position', period_in_ms=10) # sampling at 100Hz
            logconf_1.add_variable('kalman.stateX', 'float')
            logconf_1.add_variable('kalman.stateY', 'float')
            logconf_1.add_variable('kalman.stateZ', 'float')        
            scf_1.cf.log.add_config(logconf_1)
            logconf_1.data_received_cb.add_callback( lambda timestamp, data, logconf_1: log_pos_callback_1(uri_1, timestamp, data, logconf_1) )

            logconf_1.start()
            logconf_2.start()

            time.sleep(3)

            # Declaring feedback threads for movement no.3
            pos_state_thread = threading.Thread(name='Position-State-Change-Thread', target=position_state_change, args=(e1, e2))
            sound_thread = threading.Thread(name='Sound-Feedback-Thread', target=sound_feedback, args=(e1, e2))
        
            # Starting threads for movement no.3
            sound_thread.start()
            pos_state_thread.start()

            # Perform the catching task
            drone_move_pc(scf_1, e2)

            # Threads join  
            sound_thread.join()
            pos_state_thread.join()

            time.sleep(3)

            logconf_1.stop()
            logconf_2.stop()




