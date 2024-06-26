import time
import cflib.crtp
import queue
import threading
import winsound

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig


# URI to the Crazyflie to connect to
uri_1 = 'radio://0/80/2M/E7E7E7E704' # Drone's uri
uri_2 = 'radio://0/80/2M/E7E7E7E7E7' # Leg sensor's uri

init_H = float(0.2)  # Initial drone's height; unit: m
# final_H = float(0.7)  # Final drone's height; unit: m

## Define the max ROM according to the movement
max_hip_exten = float(0.68)         # for movement (a) Hip exten; unit: m
max_hip_abd = float(0.58)           # for movement (b) Hip abd/add; unit: m
max_knee_flex = float(0.5)          # for movement (c) Knee flex; unit: m
max_tiptoe = float(0.35)            # for movement (d) Tiptoe; unit: m
max_hip_knee_flex = float(0.53)     # for movement (e) Hip & knee flex; unit: m
max_heel_to_heel = float(0.2)       # for movement (f) Heel to heel; unit: m
max_step_forward = float(0.5)       # for movement (g) Step forward; unit: m

max_ROM = max_knee_flex    # change this variable according to the selected movement


init_Vel = 0.5  # Initial velocity
task_Vel = 0.3  # on-task velocity


position_estimate_1 = [0, 0, 0]  # Drone's pos
position_estimate_2 = [0, 0, 0]  # LS's pos


# # Positioning Callback Section

def log_pos_callback_1(uri_1, timestamp, data, logconf_1):
    global position_estimate_1
    position_estimate_1[0] = data['kalman.stateX']
    position_estimate_1[1] = data['kalman.stateY']
    position_estimate_1[2] = data['kalman.stateZ']
    print("{}: {} is at pos: ({}, {}, {})".format(timestamp, uri_1, position_estimate_1[0], position_estimate_1[1], position_estimate_1[2]))
                                            

def log_pos_callback_2(uri_2, timestamp, data, logconf_2):
    global position_estimate_2
    position_estimate_2[0] = data['kalman.stateX']
    position_estimate_2[1] = data['kalman.stateY']
    position_estimate_2[2] = data['kalman.stateZ']
    print("{}: {} is at pos: ({}, {}, {})".format(timestamp, uri_2, position_estimate_2[0], position_estimate_2[1], position_estimate_2[2]))
                                              


# # Crazyflie Motion Section

def drone_guide_mc(scf, event1, event2): # default take-off height = 0.3 m

    with MotionCommander(scf) as mc:
        mc.up(init_H, velocity=init_Vel)
        time.sleep(2)

        for i in range(1,5):
            print("Round: ", i)

            ## Movement (a) Hip exten & (c) Knee flex
            # mc.move_distance(-0.5, 0, 0.5, velocity=task_Vel)  # moving up-front (refers to the drone)
            mc.up(0.5, velocity=task_Vel)  # moving up (refers to the drone)
            time.sleep(0.8)

            while event1.is_set()==False:
                print("Fighting!")
                mc.stop()
                time.sleep(0.5)

            print("Target was reached!")
            # mc.move_distance(0.5, 0, -0.5, velocity=task_Vel)  # moving back
            mc.down(0.5, velocity=task_Vel)  # moving back
            time.sleep(0.5)


            # ## Movement (b) Hip abd/add
            # mc.move_distance(0, 0.4, 0.4, velocity=task_Vel)  # moving up-left (refers to the drone)
            # time.sleep(0.8)

            # while event1.isSet()==False:
            #     mc.stop()
            #     time.sleep(1.5)

            # print("Target was reached!")
            # mc.move_distance(0, -0.4, -0.4, velocity=task_Vel)  # moving back
            # time.sleep(1.5)


            # ## Movement (d) Tip-toe
            # mc.up(0.3, velocity=task_Vel)  # moving up
            # time.sleep(0.8)

            # while event1.isSet()==False:
            #     mc.stop()
            #     time.sleep(1.5)

            # print("Target was reached!")
            # mc.down(0.3, velocity=task_Vel)  # moving back
            # time.sleep(1.5)


            # ## Movement (e) Hip & Knee flex
            # mc.up(0.4, velocity=task_Vel)  # moving up
            # time.sleep(0.8)

            # while event1.isSet()==False:
            #     mc.stop()
            #     time.sleep(1.5)

            # print("Target was reached!")
            # mc.down(0.4, velocity=task_Vel)  # moving back
            # time.sleep(1.5)


            # ## Movement (f) Heel to heel & (g) knee raising, then step forward *** Be careful the step amount!! (can't be out of the LH range)
            # mc.forward(0.2, velocity=task_Vel)  # moving forward (refers to the drone)
            # time.sleep(1.5)

            # while event1.isSet()==False:    # go to the "position_state_change" --> change it to comparing the max_ROM in x or y axis
            #     mc.stop()
            #     time.sleep(1.5)

            # print("Target was reached!")


        # set the event for turning off the sound feedback process
        event2.set()


# # Feedback Section

def position_state_change(event1, event2):
    print("position thread start")
    while not event2.is_set():  # the drone hasn't finished the guiding yet
        # if position_estimate_2[0] < max_ROM:   # If the current leg sensor's position doesn't reach the max ROM in x-axis
        if position_estimate_2[2] < max_ROM:   # If the current leg sensor's position doesn't reach the max ROM in z-axis
            event1.clear()
        
        else:
            event1.set() # subject reaches the target point


def sound_feedback(event1, event2):
    print("sound thread started")
    while not event2.is_set():  # the drone hasn't finished the guiding yet
        if event1.is_set()==True:  # subject reaches the target point
            print("Good Job!")
            frequency = 2500  # Set Frequency To 2500 Hertz
            duration = 500  # Set Duration To 250 ms == 0.25 second
            winsound.Beep(frequency, duration)
        else:
            pass
            
        time.sleep(0.1)

# using other sounds, see in:  https://www.geeksforgeeks.org/python-winsound-module/


if __name__ == '__main__':

    # # initializing the queue and event object
    q = queue.Queue(maxsize=0)
    e1 = threading.Event()  # Checking whether the drone completes its task?
    e2 = threading.Event()  # Checking whether the Subject reaches the target height?

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


        # # Drone Motion (MotionCommander)
            # Declaring threads for feedback providing
            pos_state_thread = threading.Thread(name='Position-State-Change-Thread', target=position_state_change, args=(e1, e2))
            sound_thread = threading.Thread(name='Sound-Feedback-Thread', target=sound_feedback, args=(e1, e2))

            # Starting threads for drone motion
            pos_state_thread.start()
            sound_thread.start()

            # Perform the drone guiding task
            drone_guide_mc(scf_1, e1, e2)
            
            # Threads join
            pos_state_thread.join()
            sound_thread.join()
    
            
            time.sleep(3)

            logconf_1.stop()
            logconf_2.stop()

