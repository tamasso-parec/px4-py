import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, BatteryStatus, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, FailsafeFlags, VehicleStatus, VehicleAttitudeSetpoint, SensorGps
from commander_msg.msg import CommanderAll, CommanderPathPoint, CommanderArm, CommanderMode, CommanderAction
import numpy as np
import json
import math
import os

class GUI(Node):
    def __init__(self, root):
        super().__init__('commander_gui')
        self.root = root
        self.root.title("Commander GUI")
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.offboard_control_mode_publisher = self.create_subscription(
            OffboardControlMode, '/fmu/in/offboard_control_mode', self.commander_heartbeat, qos_profile)
        self.gps_subscriber = self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position', self.gps_callback, qos_profile)
        self.failsafe_subscriber = self.create_subscription(
            FailsafeFlags, '/fmu/out/failsafe_flags', self.failsafe_callback, qos_profile)
        self.failsafe_subscriber = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status', self.battery_callback, qos_profile)
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.trajectory_setpoint_callback,
            qos_profile)
        self.commander_all_sub = self.create_subscription(
            CommanderAll,
            '/com/out/all',
            self.commander_all_callback,
            qos_profile)
        self.commander_arm_pub = self.create_publisher(
            CommanderArm, '/com/in/arm', qos_profile)
        self.commander_force_disarm_pub= self.create_publisher(
            CommanderArm, '/com/in/force_disarm', qos_profile)
        self.commander_mode_pub = self.create_publisher(
            CommanderMode, '/com/in/mode', qos_profile)
        self.commander_action_pub= self.create_publisher(
            CommanderAction, '/com/in/action', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/com/in/trajectory_setpoint', qos_profile)
        
        # Speed Sliders
        self.speed_var_1 = tk.DoubleVar(value=0.2)
        self.speed_var_2 = tk.DoubleVar(value=0.2)
        ttk.Label(root, text="Speed 1:").grid(row=0, column=0)
        ttk.Scale(root, from_=0, to=3, orient="horizontal", variable=self.speed_var_1).grid(row=0, column=1)
        
        ttk.Label(root, text="Speed 2:").grid(row=1, column=0)
        ttk.Scale(root, from_=0, to=3, orient="horizontal", variable=self.speed_var_2).grid(row=1, column=1)
        
        # Movement Buttons
        self.btn_forward = ttk.Button(root, text="↑", command=lambda: self.manual_control("pad_forward"))
        self.btn_left = ttk.Button(root, text="←", command=lambda: self.manual_control("pad_left"))
        self.btn_right = ttk.Button(root, text="→", command=lambda: self.manual_control("pad_right"))
        self.btn_back = ttk.Button(root, text="↓", command=lambda: self.manual_control("pad_back"))
        
        self.btn_forward.grid(row=2, column=1)
        self.btn_left.grid(row=3, column=0)
        self.btn_right.grid(row=3, column=2)
        self.btn_back.grid(row=4, column=1)
        
        # Status Labels
        self.status_label = ttk.Label(root, text="Drone: UNKNOWN")
        self.status_label.grid(row=5, column=0, columnspan=3)
        
        # Control Buttons
        self.arm_button = ttk.Button(root, text="ARM", command=self.arm)
        self.disarm_button = ttk.Button(root, text="DISARM", command=self.disarm)
        self.arm_button.grid(row=6, column=0)
        self.disarm_button.grid(row=6, column=1)
        
        self.takeoff_button = ttk.Button(root, text="TAKEOFF", command=self.takeoff)
        self.land_button = ttk.Button(root, text="LAND", command=self.land)
        self.takeoff_button.grid(row=7, column=0)
        self.land_button.grid(row=7, column=1)

        self.stop_button = ttk.Button(root, text="STOP", command=lambda: self.stop)
        self.stop_button.grid(row=6, column=2)

        self.kill_button = ttk.Button(root, text="KILL", command=self.kill_all)
        self.kill_button.grid(row=7, column=2)
        
        # Mode Selection
        self.mode_var = tk.StringVar(value="None")
        ttk.Label(root, text="Mode:").grid(row=9, column=0)
        self.mode_menu = ttk.Combobox(root, textvariable=self.mode_var, values=["spin", "routine", "goto", "path", "updown", "None"])
        self.mode_menu.grid(row=9, column=1)
        
        self.confirm_button = ttk.Button(root, text="Confirm Mode", command=self.confirm_mode)
        self.confirm_button.grid(row=10, column=0, columnspan=2)
    
    def commander_heartbeat(self,msg):
        new_time = msg.timestamp
        current = self.now()
        #print("new time:", current)
        self.delta_hr=current - new_time
        self.last_heartbeat = new_time
        #print("last hr: ", self.last_heartbeat)
        #logging.debug("hr delta")
        #logging.debug(self.delta_hr)
        #1 secondo arbitrario

    def commander_all_callback(self,msg):
        self.commander_all=msg
    
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        #logging.debug("recived vehicle position")
        #logging.debug(vehicle_local_position)
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        #logging.debug("recived vehicle status")

        self.vehicle_status = vehicle_status
    
    def gps_callback(self,msg):
        self.gps=msg
        
    def failsafe_callback(self,msg):
        #logging.debug("recived failsafe msg")
        self.failsafe=msg
        
    def battery_callback(self,msg):
        self.battery=msg
    
    def trajectory_setpoint_callback(self, msg):
        self.setpoint_position = msg.position
        self.setpoint_position_yaw = msg.yaw

    def arm(self):
        msg = CommanderArm()
        
        msg.timestamp = self.now()
        msg.arm=True
        try:
            self.commander_arm_pub.publish(msg)
        except Exception as e:
            print(e)
        # logging.info("sent arm command")

        self.status_label.config(text="Drone: ARMED")
    
    def disarm(self):
        msg = CommanderArm()
        msg.timestamp = self.now()
        msg.arm = False
        self.commander_arm_pub.publish(msg)
        self.status_label.config(text="Drone: DISARMED")

    def force_disarm(self):
        msg = CommanderArm()
        msg.timestamp = self.now()
        msg.arm=False
        self.commander_force_disarm_pub.publish(msg)

    def kill_all(self):
        os.system("killall -9 ros2")

    def action(self, action):
        msg = CommanderAction()
        msg.timestamp = self.now()
        msg.action = action
        self.commander_action_pub.publish(msg)
        
    def stop(self):
        msg = CommanderMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        # msg.mode = config.MODE_STOP
       
        
        self.commander_mode_pub.publish(msg)
    
    def takeoff(self):
        self.action("takeoff")
        print("TAKEOFF command sent")
    
    def land(self):
        self.action("land")
        print("LAND command sent")
    
    def confirm_mode(self):
        msg = CommanderMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.mode = self.mode_var.get()
        self.commander_mode_pub.publish(msg)
        print(f"Mode set to {self.mode_var.get()}")
    
    def manual_control(self, action):
        print(f"Manual control: {action}")
    
    def vehicle_status_callback(self, msg):
        arm_status = "ARMED" if msg.arming_state == 2 else "DISARMED"
        self.status_label.config(text=f"Drone: {arm_status}")

    def now(self):
        return int(self.get_clock().now().nanoseconds / 1000)
    
    def parse_vlp(self):
        vlp=self.vehicle_local_position
        txt=""
        time="timestamp: "+ str(vlp.timestamp) + "\n"
        pos=f'\nPOSITION [m]\nx: {vlp.x:.3f} \ny:  {vlp.y:.3f} \nz:  {vlp.z:.3f}\n'
        vel=f"\nVELOCITY [m/s]\nvx: {vlp.vx:.3f}\nvy: {vlp.vy:.3f} \nvz: {vlp.vz:.3f}\n"
        acc=f"\nACCELERATION [m/s^2]\nax: {vlp.ax:.3f} \nay: {vlp.ay:.3f} \naz:  {vlp.az:.3f} \n"
        dir=f"\nDIRECTION [rad]\nyaw: {vlp.heading:.3f} \n"
        exyzh=self.pose_error_xyzh()
        off=f"\nOFFSET [m] [rad]\nx: {exyzh[0]:.3f}\ny: {exyzh[1]:.3f}\nz: {exyzh[2]:.3f}\nyaw: {exyzh[3]:.3f}\n\ntotal: {self.pose_error():.3f}"

        return time+pos+vel+acc+dir+off

    def parse_gps(self):
        """uint64 timestamp		# time since system start (microseconds)
        uint64 timestamp_sample

        uint32 device_id                # unique device ID for the sensor that does not change between power cycles

        float64 latitude_deg		# Latitude in degrees, allows centimeter level RTK precision
        float64 longitude_deg		# Longitude in degrees, allows centimeter level RTK precision
        float64 altitude_msl_m		# Altitude above MSL, meters
        float64 altitude_ellipsoid_m	# Altitude above Ellipsoid, meters
        """
        gps=self.gps
        
        time="timestamp: "+ str(gps.timestamp) + "\n"
        pos=f'\nPOSITION\nlatitude: {gps.latitude_deg:.9f} \nlongitude:  {gps.longitude_deg:.9f} \naltitude msl:  {gps.altitude_msl_m:.2f}\naltitude ellipsoid:  {gps.altitude_ellipsoid_m:.2f}\n'
        misc1=f"\nMISC\nvelocity: {gps.vel_m_s:.2f}\nheading: {gps.heading}\noise {gps.noise_per_ms}\n"
        misc2=f"\njamming state: {gps.jamming_state}\nsatellite used: {gps.satellites_used}\n"
        return time+pos+misc1+misc2
    
    def parse_status(self):
        st=self.vehicle_status
        fs = self.failsafe
        time="timestamp: "+ str(st.timestamp) + "\n"
        armed_state={
            1:"DISARMED",
            2:"ARMED"
        }
        dict_arm_reason={
            0: "transistion to standby",
            1: "rc stick input" ,
            2: "rc switch" ,
            3: "internal command" ,
            4: "external command" ,
            5: "mission start" ,
            6: "safety button" ,
            7: "auto disarm on land" ,
            8: "auto disarm preflight" ,
            9: "kill switch" ,
            10: "lockdown" ,
            11: "failure detector" ,
            12: "shutdown" ,
            13: "unit testing"
            
        }
        dict_nav_state={
            0:"manual",
            1:"altitude controll mode",
            2:"position control mode",
            3:"auto mission mode",
            4:"auto loiter mode",
            5:"auto return mode",
            10:"acro mode",
            12:"descend mode",
            13:"terminator mode",
            14:"offboard control",
            15:"stabilized mode",
            17:"takeoff",
            18:"landing",
            19:"auto follow",
            20:"precision land",
            21:"orbit",
            22:"vtol takeoff"
        }
        time="timestamp: "+ str(st.timestamp) + "\n"
        arm=f"\nARMING STATE\narm status: {armed_state[st.arming_state]}\nlatest arming reason: {dict_arm_reason[st.latest_arming_reason]}\nlatest diarming reason: {dict_arm_reason[st.latest_disarming_reason]}\n"
        nav=f"\nNAVIGATION\nnavigation state: {dict_nav_state[st.nav_state]}\n"
        failure=f"\nFAILURE\nfailure detector: {st.failure_detector_status}\nfailsafe: {st.failsafe}\npreflight check pass: {st.pre_flight_checks_pass}\n"
        fs1=f"\noffboard signal lost: {fs.offboard_control_signal_lost}\nmanual control signal lost: {fs.manual_control_signal_lost}"
        fs2=f"\nmission failure: {fs.mission_failure}\nlow local position accuracy: {fs.local_position_accuracy_low}"
        fs3=f"\nesc arming failure: {fs.fd_esc_arming_failure}\nimbalanced props {fs.fd_imbalanced_prop}\nmotor failure: {fs.fd_motor_failure}"
        fs4=f"\ncritical failure: {fs.fd_critical_failure}"
        return time+arm+nav+failure+fs1+fs2+fs3+fs4
        
    def parse_battery(self):
        bt=self.battery
        
        time="timestamp: "+ str(bt.timestamp) + "\n"
        volt=f"\nVOLTAGE\nvoltage: {bt.voltage_v:.3f} V\n"
        amp=f"\nCURRENT\ncurrent: {bt.current_a:.3f}\ncurrent average: {bt.current_average_a:.3f} A\n"
        charge=f"\nCHARGE\nremaining: {(bt.remaining*100):.3f} %\ndischarged {bt.discharged_mah:.3f} mah\n time remaining: {(bt.time_remaining_s/60):.1f} minutes\n"
        info=f"\nINFO\ntemperature: {bt.temperature:.3f}"
        return time+volt+amp+charge+info
        
    def parse_battery_level(self):
        minV=13.2
        maxV=16.7
        bt=self.battery
        perc=bt.voltage_v*minV/maxV
        #return int(perc)
        return int(bt.remaining*100)
    
    def pose_error(self):
        try:
            x1=self.vehicle_local_position.x
            y1=self.vehicle_local_position.y
            z1=self.vehicle_local_position.z
            
            x2=self.setpoint_position[0]
            y2=self.setpoint_position[1]
            z2=self.setpoint_position[2]
        
            dist = math.dist([x1,y1,z1],self.setpoint_position)
            #logging.debug("dist")
            #logging.debug(dist)
            
            if dist > self.max_error:
                self.max_error=dist
            
            return dist
        except:
            return 0
        
    def pose_error_xyzh(self):
        try:
            x1=self.vehicle_local_position.x
            y1=self.vehicle_local_position.y
            z1=self.vehicle_local_position.z
            h1=self.vehicle_local_position.heading
            
            x2=self.setpoint_position[0]
            y2=self.setpoint_position[1]
            z2=self.setpoint_position[2]
            z2=self.setpoint_position[2]
            h2=self.setpoint_position_yaw
        
            dist=np.array([x1,y1,z1,h1])    -   np.array([x2,y2,z2,h2])
            #logging.debug("dist xyzh")
            #logging.debug(dist)
            
            return dist
        except:
            return np.array([0.0,0.0,0.0,0.0])
    


def main(args=None) -> None:
    print('Starting gui...')
    rclpy.init(args=args)
    root = tk.Tk()
    gui = GUI(root)
    root.mainloop()
    rclpy.shutdown()




if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
