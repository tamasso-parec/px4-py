import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint
from geometry_msgs.msg import PoseStamped
from rclpy import qos


import numpy as np
import math

import px4_py.config as config
import random
import logging

logging.basicConfig(level=config.log)
def homogeneous_transform(quaternion, position):
    """Return homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < 1e-13:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], position[0]),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], position[1]),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], position[2]),
        (                0.0,                 0.0,                 0.0,         1.0)
        ), dtype=np.float64)

def transformation_matrix_from_pose_msg(pose_msg):
    """Convert a Pose message to a transformation matrix."""
    # Extract position and orientation from the Pose message
    position = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
    orientation = [pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z]

    # Create the transformation matrix
    T = homogeneous_transform(orientation, position)

    return T

def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix.

    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
    True

    """
    q = np.empty((4, ), dtype=np.float32)
    M = np.array(matrix, dtype=np.float32, copy=False)[:4, :4]
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q

def odometry_msg_from_transformation_matrix(T):
    """Convert a transformation matrix to a Pose message."""
    # Extract position from the transformation matrix
    position = np.array(T[:3, 3], dtype=np.float32)

    # logging.info("Altitude: "+ str(position[2]))
    # Extract orientation from the transformation matrix
    q = quaternion_from_matrix(T)

    # p = [xyz.x, xyz.z, -xyz.y] #z with - for NED
    p_NED = np.array([position[0], position[2], -position[1]], dtype=np.float32)

    # logging.info("p_NED: "+ str(p_NED))



    # q=[qt.w,qt.x,qt.z,-qt.y]  
    q_NED = np.array([q[3], q[0], q[2], -q[1]], dtype=np.float32)



    odometry_msg = VehicleOdometry()
    odometry_msg.position = p_NED


    
    odometry_msg.q = q_NED
    
    return odometry_msg


class Converter(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('opti_to_px4')

        self.odo=VehicleOdometry()
        self.pose=PoseStamped()
        self.ready=True

        self.initial_pose_inverse = np.eye(4)

        self.initial_pose_stored_flag = False
        
        self.pubs=0
        self.sub_odo=0
        self.sub_pose=0
        self.new = False
        
        self.dt=config.opti_to_px4_dt
        
        
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.odometry_publisher= self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_mocap_odometry', qos_profile)
        self.v_odometry_publisher= self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile)
            
        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.optitrack_subscriber= self.create_subscription(
            PoseStamped, '/optiTrack/pose', self.optitrack_pose_callback, qos.qos_profile_sensor_data)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Convert ENU to NED

        #TODO: Fix this
        self.T_ENU_to_NED = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ])
        

        # Create a timer to publish control commands
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.timer_status = self.create_timer(1.0, self.status_callback)
        
        logging.debug("init complete")

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle odometry"""
        logging.debug("recived odometry")

        self.odo = msg
        self.sub_odo = self.sub_odo + 1


    def optitrack_pose_callback(self, msg):
        """Callback function for optitrack pose"""
        logging.debug("received pose")
        self.pose = msg
        self.sub_pose = self.sub_pose + 1
        self.new = True

        if not self.initial_pose_stored_flag:

            logging.info("Storing initial pose")
            # Store the initial pose

            xyz = msg.pose.position

            p = [xyz.x, xyz.y, xyz.z]

            # p = [xyz.x, xyz.z, -xyz.y] #z with - for NED

            
            qt=msg.pose.orientation

            # Based on the way it is defined
            q = [qt.x, qt.y, qt.z, qt.w]
            #qt=self.pose.pose.orientation
            
            # q=[
            #     qt.w,
            #     qt.x,
            #     qt.z,
            #     -qt.y
            # ]

            

            # Get homogeneous transformation matrix from quaternion and position
            T = homogeneous_transform(q, p)

            # Store the initial pose
            self.initial_pose_inverse = np.linalg.inv(T)
            self.initial_pose_stored_flag = True



    def publish_mocap_odometry(self):
        """Publish the mocap odometry"""
        
        logging.debug("publishing mocap")

        if not self.new:
            return 0
        
        
        
        xyz = self.pose.pose.position
        p = [xyz.x, xyz.y, xyz.z]

        

        # p = [xyz.x, xyz.z, -xyz.y] #z with - for NED
        
        qt=self.pose.pose.orientation
        q = [ qt.x, qt.y, qt.z, qt.w]
        #qt=self.pose.pose.orientation
        
        # q=[
        #     qt.w,
        #     qt.x,
        #     qt.z,
        #     -qt.y
        # ]

        # T_mocap = homogeneous_transform(q, p)
        # T_odometry =  self.initial_pose_inverse @ T_mocap 
        # self.get_logger().info(f"self.initial_pose_inverse:\n{self.initial_pose_inverse}")
        # Log the T_odometry transformation matrix
        # self.get_logger().info(f"T_odometry:\n{T_odometry}")


        T_mocap = homogeneous_transform(q, p)

        # Apply the ENU to NED transformation
        

        T_odometry =  self.initial_pose_inverse @ T_mocap
        
        
        msg = odometry_msg_from_transformation_matrix(T_odometry)

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp_sample = int(self.get_clock().now().nanoseconds / 1000)

        msg.pose_frame=1 #NED frame
        msg.velocity_frame=1 #NED frame
        # self.get_logger().info(f"Position:\n{msg.position}")


        # msg.position=p
        # msg.q=q
        # msg.velocity = self.odo.velocity
        # msg.angular_velocity = self.odo.angular_velocity
        # msg.position_variance = self.odo.position_variance
        # msg.orientation_variance = self.odo.orientation_variance
        # msg.velocity_variance = self.odo.velocity_variance
        msg.quality = 1


        self.odometry_publisher.publish(msg)
        self.v_odometry_publisher.publish(msg)
        
        self.pubs = self.pubs + 1
        self.new = False

    def timer_callback(self) -> None:
        if self.ready and self.initial_pose_stored_flag:
            # Publish the mocap odometry
            self.publish_mocap_odometry()
            
    def status_callback(self) -> None:
        logging.info("---------------------")
        logging.info("odometry msg recived: "+ str(self.sub_odo))
        logging.info("pose msg recived: "+ str(self.sub_pose))
        logging.info("odometry msg published: "+ str(self.pubs))
        self.sub_odo=0
        self.sub_pose=0
        self.pubs=0


def main(args=None) -> None:
    print('Starting optitrack to px4 node...')
    rclpy.init(args=args)
    converter = Converter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
