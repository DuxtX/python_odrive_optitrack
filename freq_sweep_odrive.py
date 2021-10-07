# -*- coding: utf-8 -*-
"""
Created on Sun Oct  3 12:30:59 2021

@author: yuhao
"""
import odrive
import math
import numpy as np
import idealab_tools.data_exchange.csv as csv
import sys
import time
from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame( new_id, position, rotation ):
    global pos
    pos = position
    pass
    # print( "Received frame for rigid body", new_id," ",position," ",rotation )

optionsDict = {}
optionsDict["clientAddress"] = "192.168.0.163"
optionsDict["serverAddress"] = "192.168.0.161"
optionsDict["use_multicast"] = True

streaming_client = NatNetClient()
streaming_client.set_client_address(optionsDict["clientAddress"])
streaming_client.set_server_address(optionsDict["serverAddress"])
streaming_client.set_use_multicast(optionsDict["use_multicast"])

# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streaming_client.rigid_body_listener = receive_rigid_body_frame

# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
is_running = streaming_client.run()
if not is_running:
    print("ERROR: Could not start streaming client.")
    try:
        sys.exit(1)
    except SystemExit:
        print("...")
    finally:
        print("exiting")

time.sleep(1)
if streaming_client.connected() is False:
    print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
    try:
        sys.exit(2)
    except SystemExit:
        print("...")
    finally:
        print("exiting")


#Initialize Odrive
print("finding an odrive...")
my_drive = odrive.find_any()
axis = my_drive.axis0
mo = axis.motor
enc = axis.encoder
ctrl = axis.controller
mo.config.requested_current_range = 90
mo.config.current_lim = 80
axis.requested_state = 3
time.sleep(30)
my_drive.config.dc_max_negative_current = -30
print('Motor found{}'.format(mo))
print('Setting control mode')
ctrl.config.control_mode = 2

'''
CTRL_MODE_VOLTAGE_CONTROL = 0,
CTRL_MODE_CURRENT_CONTROL = 1,
CTRL_MODE_VELOCITY_CONTROL = 2,
CTRL_MODE_POSITION_CONTROL = 3,
CTRL_MODE_TRAJECTORY_CONTROL = 4
'''

vel_max = 80

t_initial = time.time()
ctrl.input_vel = 0
axis.requested_state = 8
for vel in range(63, vel_max, 1):
    optitrack_data = []
    tic = time.time()
    toc = time.time()
    ctrl.input_vel = vel
    time.sleep(2)
    while toc - tic <= 5:
        en_vel = axis.encoder.vel_estimate
        #print(en_vel)
        optitrack_data.append([en_vel,pos[0],pos[1],pos[2]])
        toc = time.time()        
    print('Step velocity={} finished'.format(vel))
    Name = 'Odrive_freq_sweep_vel{}'.format(vel)
    Name += '.csv'
    csv.write(Name, optitrack_data)
    
t_final = time.time()
t_total = t_final - t_initial
print('Test finished, time elipsed: {}'.format(t_total))
    
'''
AXIS_STATE_UNDEFINED = 0,           //<! will fall through to idle
AXIS_STATE_IDLE = 1,                //<! disable PWM and do nothing
AXIS_STATE_STARTUP_SEQUENCE = 2, 	//<! the actual sequence is defined by the config.startup_... flags
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
AXIS_STATE_MOTOR_CALIBRATION = 4,   	//<! run motor calibration
AXIS_STATE_SENSORLESS_CONTROL = 5,  	//<! run sensorless control
AXIS_STATE_ENCODER_INDEX_SEARCH = 6, 	//<! run encoder index search
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
AXIS_STATE_CLOSED_LOOP_CONTROL = 8,  	//<! run closed loop control
AXIS_STATE_LOCKIN_SPIN = 9,       		//<! run lockin spin
AXIS_STATE_ENCODER_DIR_FIND = 10,
'''
    