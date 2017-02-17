
#coding: utf-8

import sys, os, struct, math, time, socket
import fnmatch, errno, threading

from pyrr import *
from pymavlink import *

        
def get_usec():
    '''time since 1970 in microseconds'''
    return int(time.time() * 1.0e6)


def periodic_tasks():
    '''run periodic checks'''
    if self.status.setup_mode:
        return

    if self.settings.heartbeat != 0:
        heartbeat_period.frequency = self.settings.heartbeat

    if heartbeat_period.trigger() and self.settings.heartbeat != 0:
        self.status.counters['MasterOut'] += 1
        for master in self.mav_master:
            send_heartbeat(master)

    if heartbeat_check_period.trigger():
        check_link_status()

    set_stream_rates()

    if battery_period.trigger():
        battery_report()

#----------------------------------------------------------#
class MavConnection():
    def __init__(self, port,  baud, observer):
        self.port  = port
        self.baud = baud
        self.observer = observer
        
        self.conn = None
        self.running = False
        
    def start_request_data_stream(self,  master): 
        master.mav.request_data_stream_send(self.status.target_system, self.status.target_component, 0, 20, 1)
        
        #master.mav.request_data_stream_send(self.status.target_system, self.status.target_component, 1, 1, 1)
        #master.mav.request_data_stream_send(self.status.target_system, self.status.target_component, 2, 1, 1)
        #master.mav.request_data_stream_send(self.status.target_system, self.status.target_component, 3, 1, 1)
        #master.mav.request_data_stream_send(self.status.target_system, self.status.target_component, 4, 1, 1)
        #master.mav.request_data_stream_send(self.status.target_system, self.status.target_component, 6, 1, 1)
        
        """
        # MAV_DATA_STREAM
        MAV_DATA_STREAM_ALL = 0 # Enable all data streams
        MAV_DATA_STREAM_RAW_SENSORS = 1 # Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
        MAV_DATA_STREAM_EXTENDED_STATUS = 2 # Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
        MAV_DATA_STREAM_RC_CHANNELS = 3 # Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
        MAV_DATA_STREAM_RAW_CONTROLLER = 4 # Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT,
                # NAV_CONTROLLER_OUTPUT.
        MAV_DATA_STREAM_POSITION = 6 # Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.

        target_system             : The target requested to send the message stream. (uint8_t)
        target_component          : The target requested to send the message stream. (uint8_t)
        req_stream_id             : The ID of the requested data stream (uint8_t)
        req_message_rate          : The requested interval between two messages of this type (uint16_t)
        start_stop                : 1 to start sending, 0 to stop sending. (uint8_t
        """ 
    
    def on_msg_heartbeat(self, m, master):
        
        if (self.status.target_system != m.get_srcSystem() or self.status.target_component != m.get_srcComponent()):
            self.status.target_system = m.get_srcSystem()
            self.status.target_component = m.get_srcComponent()
            self.start_request_data_stream(master)
               
        self.status.last_heartbeat = time.time()
        #master.last_heartbeat =self.status.last_heartbeat
            
    def on_msg_sensor_offsets(self, m, master):
        self.status.accel_offsets = Vector3([m.accel_cal_x, m.accel_cal_y, m.accel_cal_z])
        self.status.gyro_offsets = Vector3([m.gyro_cal_x, m.gyro_cal_y, m.gyro_cal_z])
        self.status.mag_offsets = Vector3([m.mag_ofs_x, m.mag_ofs_y, m.mag_ofs_z])#, m.mag_declination)
        self.status.press = (m.raw_press, m.raw_temp)
        
    def on_msg_scaled_imu2(self, m, master):
        accv3 = Vector3([m.xacc, m.yacc, m.zacc])
        gyrov3 = Vector3([m.xgyro, m.ygyro, m.zgyro])
        magv3 = Vector3([m.xmag, m.ymag, m.zmag])
        
        #print accv3, gyrov3, magv3 
    
    def on_msg_raw_imu(self, m, master):
        accv3 = Vector3([m.xacc, m.yacc, m.zacc])
        gyrov3 = Vector3([m.xgyro, m.ygyro, m.zgyro])
        magv3 = Vector3([m.xmag, m.ymag, m.zmag])
        
        #print accv3, gyrov3, magv3 
        
    def on_msg_attitude(self, m,  master):
        self.observer.update_attitude(m.pitch, m.roll, m.yaw) 
        """
        time_boot_ms : 318067, 
        roll : 0.00354599650018, 
        pitch : 0.0179140623659, 
        yaw : 1.61865353584, 
        rollspeed : 8.43061134219e-05, 
        pitchspeed : -0.000376928946935, 
        yawspeed : -0.000310299918056}
        """
        
    def on_msg_ahrs(self, m,  msster):
        pass
        """
        omegaIx : -2.38564134634e-05, 
        omegaIy : 0.00019565311959, 
        omegaIz : -0.00018073352112, 
        accel_weight : 0.0, 
        renorm_val : 0.0, 
        error_rp : 0.000493810279295, 
        error_yaw : 0.00441460032016}
        """

    def on_send_callback(self, m, master):
        '''called on sending a message'''
        
        mtype = m.get_type()
        
        if mtype == 'HEARTBEAT':
            return
            
        #self.observer.info("send: %s" % mtype)
    
    def on_recv_callback(self, m, master):

        mtype = m.get_type()
        
        if mtype == "BAD_DATA":
                self.bad_count += 1
                return
        
        #print mtype
        
        if getattr(m, '_timestamp', None) is None:
            master.post_message(m)
            self.status.counters['MasterIn'] += 1

        #if getattr(m, 'time_boot_ms', None) is not None:
            # update link_delayed attribute
        #    self.handle_msec_timestamp(m, master)
        
        msg_handler_func_name = "on_msg_" + mtype.lower()
        
        msg_handler = getattr(self, msg_handler_func_name,  None)
        if msg_handler :
            msg_handler(m, master) 
        else :
            #print "msg handler not found", msg_handler_func_name
            pass
            
        if mtype not in ["PARAM_VALUE", "STATUSTEXT"]:
            self.status.last_msg[mtype] = m  
            msg = ' '.join(str(m).split()[1:])[1:-1]
            self.observer.show_msg(mtype, msg)
                
    def open(self):
        
        if self.conn :
                return False
                
        self.bad_count = 0
        self.status = MasterStatus()
        self.param_mgr = ParamManager(self.status.mav_param, self.observer)
        
        self.conn = mavutil.mavlink_connection(self.port, autoreconnect=True, baud=self.baud)
        
        self.conn.mav.set_callback(self.on_recv_callback, self.conn)
        self.conn.mav.set_send_callback(self.on_send_callback, self.conn)
            
        self.heartbeat_period = mavutil.periodic_event(1)
        
        # run main loop as a thread
        self.loop_thread = threading.Thread(target = self.loop_run)
        self.loop_thread.daemon = True
        self.running = True
        self.loop_thread.start()
        
        return True
        
    def close(self):
        if not self.conn:
            return
        self.running = False    
        while self.conn :
                time.sleep(0.2)
        
    def loop_run(self):
        while self.running : 
            with self.read_lock :        
                self.conn.recv_msg()
            if self.heartbeat_period.trigger() :
                self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self.param_mgr.fetch_check(self.conn)                                        
        #end while
        self.conn.close()
        self.conn = None 
