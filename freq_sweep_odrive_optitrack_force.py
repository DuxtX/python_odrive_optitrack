# -*- coding: utf-8 -*-
"""
Created on Sun Oct  3 12:30:59 2021

@author: yuhao
"""
import odrive
import math
import numpy
import numpy as np
import idealab_tools.data_exchange.csv as csv
import sys
import time
from NatNetClient import NatNetClient
import nidaqmx
from nidaqmx.constants import Edge
from nidaqmx.utils import flatten_channel_string
from nidaqmx.stream_readers import AnalogMultiChannelReader
from nidaqmx.stream_writers import AnalogMultiChannelWriter
from nidaqmx import constants
from threading import Thread
import logging


daq_read_task = nidaqmx.Task()
daq_calibration_task = nidaqmx.Task()
daq_sample_rate = 1000
chans_in = 4
buffer_in_size = 10
bufsize_callback = buffer_in_size
buffer_in_size_cfg = round(buffer_in_size * 1)  # clock configuration
# Initialize data placeholders
buffer_in = np.zeros((chans_in, buffer_in_size))
daq_data = np.zeros((chans_in, 1))  # will contain a first column with zeros but that's fine

def configure_daq():
    sample_rate = daq_sample_rate
    daq_read_task.ai_channels.add_ai_voltage_chan("Dev1/ai0", max_val=10, min_val=-10)
    daq_read_task.ai_channels.add_ai_voltage_chan("Dev1/ai1", max_val=10, min_val=-10)
    daq_read_task.ai_channels.add_ai_voltage_chan("Dev1/ai2", max_val=10, min_val=-10)
    daq_read_task.ai_channels.add_ai_voltage_chan("Dev1/ai3", max_val=10, min_val=-10)
    daq_read_task.timing.cfg_samp_clk_timing(rate=sample_rate, sample_mode = nidaqmx.constants.AcquisitionType.CONTINUOUS, samps_per_chan=buffer_in_size_cfg)

def reading_task_callback(task_idx, event_type, num_samples, callback_data):  # bufsize_callback is passed to num_samples
    global daq_data
    global buffer_in

    if running:
        # It may be wiser to read slightly more than num_samples here, to make sure one does not miss any sample,
        # see: https://documentation.help/NI-DAQmx-Key-Concepts/contCAcqGen.html
        buffer_in = np.zeros((chans_in, num_samples))  # double definition ???
        reader.read_many_sample(buffer_in, num_samples, timeout=constants.WAIT_INFINITELY)
        daq_data = buffer_in  # appends buffered data to total variable data

    return 0  # Absolutely needed for this callback to be well defined (see nidaqmx doc).
    
def daq_acquisition():
    global running
    global reader
    global daq_data
    running = True
    reader = AnalogMultiChannelReader(daq_read_task.in_stream)
    daq_read_task.register_every_n_samples_acquired_into_buffer_event(bufsize_callback, reading_task_callback)
    daq_read_task.start()

def daq_calibration():
    daq_calib_data = daq_calibration_task.read()
    return daq_calib_data

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame( new_id, position, rotation ):
    global pos
    global rot
    pos = position
    rot = rotation
    pass
    # print( "Received frame for rigid body", new_id," ",position," ",rotation )

optionsDict = {}
optionsDict["clientAddress"] = "192.168.1.166"
optionsDict["serverAddress"] = "192.168.1.166"
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


#Calibrate load cell

print("Start load cell calibration, please lift up the object")
time.sleep(5)
print("Calibration start")
configure_daq()
tic = time.time()
toc = time.time()
daq_force = []
daq_calibration_task.ai_channels.add_ai_voltage_chan("Dev1/ai0", max_val=10, min_val=-10)
daq_calibration_task.ai_channels.add_ai_voltage_chan("Dev1/ai1", max_val=10, min_val=-10)
daq_calibration_task.ai_channels.add_ai_voltage_chan("Dev1/ai2", max_val=10, min_val=-10)
daq_calibration_task.ai_channels.add_ai_voltage_chan("Dev1/ai3", max_val=10, min_val=-10)
while toc - tic <= 5:
    daq_force.append(daq_calibration())
    toc = time.time()
daq_force = np.array(daq_force)
daq_calibration = np.mean(daq_force,axis=1)
daq_calibration_task.close()
print("Calibration finished")


#Initialize Odrive
print("finding an odrive...")
my_drive = odrive.find_any()
axis = my_drive.axis1
mo = axis.motor
enc = axis.encoder
ctrl = axis.controller
mo.config.current_lim = 30
mo.config.requested_current_range = 90
mo.config.calibration_current = 20
ctrl.config.vel_limit = 8000
mo.config.pole_pairs = 7
mo.config.motor_type = 0
'''
MOTOR_TYPE_HIGH_CURRENT = 0,
MOTOR_TYPE_GIMBAL = 2
'''
axis.encoder.config.cpr = 8192
axis.requested_state = 3
time.sleep(20)
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

vel_max = 45

t_initial = time.time()
ctrl.input_vel = 0
axis.requested_state = 8

# thread_daq = Thread(target = daq_acquisition)
# thread_daq.start()

daq_acquisition()

for vel in range(1, vel_max, 1):
    optitrack_data = []
    tic = time.time()
    toc = time.time()
    ctrl.input_vel = vel
    time.sleep(2)
    contact_force = []
    while toc - tic <= 5:
        en_vel = axis.encoder.vel_estimate 
        #print(en_vel)
        optitrack_data.append([en_vel, pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]])
        daq_force = daq_data[-1]
        contact_force.append([daq_force[0] - daq_calibration[0], daq_force[1] - daq_calibration[1], -(daq_force[2] - daq_calibration[2]), daq_force[3] - daq_calibration[3]])
        toc = time.time()        
    print('Step velocity={} finished'.format(vel))
    
    data = np.concatenate((np.array(optitrack_data), np.array(contact_force)), axis = 1)
    Name = 'UL_clockwise_force_vel{}'.format(vel)
    Name += '.csv'
    csv.write(Name, data)

daq_read_task.close()
axis.requested_state = 1
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
    