#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import json
import time
from io import BytesIO
import base64
import argparse
from PIL import Image
import numpy as np
from gym_donkeycar.core.sim_client import SDClient
import math as m
import traceback # this is just for seeing the cause of the error if one is thrown during runtime without stopping the code
###########################################

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

def quaternion_to_angle(quat):
	quat_list = [quat[0], quat[1], quat[2], quat[3]]
	(r,p,y) = tf.transformations.euler_to_quaternion(quat_list)
	return r,p,y

class SimpleClient(SDClient):

    def __init__(self, address, args, poll_socket_sleep_time=0.001):
        super(SimpleClient,self).__init__(*address, poll_socket_sleep_time=poll_socket_sleep_time)
        self.last_image = None
        self.car_loaded = False
        self.now = time.time()
        self.pose = np.zeros(7)
        self.twist = np.zeros(6)
        self.accel = np.zeros(3)
        self.speed = 0
        self.heading = 0
        self.cte = 0

    def on_msg_recv(self, json_packet):
        # global car
        if json_packet['msg_type'] == "car_loaded":
            self.car_loaded = True
        
        if json_packet['msg_type'] == "telemetry":
            # the images that come in are grayscale except for the center image, which is RGB. This was done to maintain compatibility with Reinforcement learning script
            # the imitation learning model uses grayscale images and therefore in this particular script, all images are converted to grayscale (as even gray images come in as RGB images with all channels being equal)
            time_stamp = time.time() # this time stamp is useful for time-sensitive control algorithms.
            # {"msg_type":"telemetry","steering_angle":0,"throttle":0,"speed":1.713824E-06,"hit":"none","pos_x":50.01714,"pos_y":0.1858531,"pos_z":49.96981,"vel_x":4.037573E-08,"vel_y":-1.637578E-06,"vel_z":5.03884E-07,"quat_x":-8.79804E-05,"quat_y":0.0001398507,"quat_z":1.122981E-05,"quat_w":1,"heading":1.570517,"gyro_x":5.826035E-06,"gyro_y":-5.950142E-09,"gyro_z":-4.740134E-07,"Ax":-5.411929E-07,"Ay":-8.932128E-06,"Az":2.184387E-06,"time":9.790786,"cte":0.001136682}
            self.pose[:3] = np.array([ json_packet["pos_x"], json_packet["pos_z"], json_packet["pos_y"] ])
            self.pose[:2] -= 50.0 # the sim has a weird 50x50 offset for some reason.
            self.pose[3:] = np.array([ json_packet["quat_x"],json_packet["quat_y"], json_packet["quat_z"], json_packet["quat_w"]])
            self.twist[:3] = np.array([ json_packet["vel_x"],json_packet["vel_z"], json_packet["vel_y"]])
            self.twist[3:] = np.array([ json_packet["gyro_x"], json_packet["gyro_z"], json_packet["gyro_y"]])
            A = np.array([json_packet["Ax"],json_packet["Az"],json_packet["Ay"]])
            self.heading = json_packet["heading"]
            self.speed = json_packet["speed"]
            self.hit = json_packet["hit"]
            self.cte = json_packet["cte"]
            self.accel[0] = A[0]*m.cos(self.heading) - A[1]*m.sin(self.heading)
            self.accel[1] = A[0]*m.sin(self.heading) + A[1]*m.cos(self.heading)
            self.accel[2] = A[2]

            dt = time.time() - self.now
            self.now = time.time()

    def send_controls(self, steering, throttle):
        p = { "msg_type" : "control",
                "steering" : steering.__str__(),
                "throttle" : throttle.__str__(),
                "brake" : "0.0" }
        msg = json.dumps(p)
        self.send(msg)
        #this sleep lets the SDClient thread poll our message and send it out.
        time.sleep(self.poll_socket_sleep_sec)


def input_callback(data,args):
    client = args
    speed = data.drive.speed
    steering = data.drive.steering_angle
    client.send_controls(steering*(57.3/30),speed) # gotta normalize it
    

###########################################
## Make some clients and have them connect with the simulator

def test_clients(args):
    # test params
    host = "127.0.0.1" # local host
    port = 9091
    num_clients = 1
    clients = []

    # Start Clients
    for _ in range(1):
        c = SimpleClient(address=(host, port),args=args,poll_socket_sleep_time=0.001)
        clients.append(c)

        # time.sleep(1)
        msg = '{ "msg_type" : "load_scene", "scene_name" : "'+args.env_name+ '" }'
        clients[_].send(msg)
        time.sleep(1)
        # Wait briefly for the scene to load.         
        # Car config
        msg = '{ "msg_type" : "car_config", "body_style" : "mushr", "body_r" : "0", "body_g" : "0", "body_b" : "255", "car_name" : "MUSHR", "font_size" : "100" }' # do not change
        clients[_].send(msg)
        # print("reached here")
        time.sleep(1)

    loaded = False
    while(not loaded):
        time.sleep(1.0)
        for c in clients:
            loaded = c.car_loaded  
            print("loading")

    pub = []
    imu_pub = []
    goal_pub = []
    sub = []
    # this is basically initializing all the subscribers for counting the number of cars and publishers for initiailizing pose and goal points.
    for i in range(num_clients):
        subscriber = rospy.Subscriber("/car" + str(i+1) + "/mux/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, input_callback,(clients[i]) )
        publisher = rospy.Publisher("/car" + str(i+1) + "/car_odom", Odometry, queue_size=1)
        imu_publisher = rospy.Publisher("/car" + str(i+1) + "/imu", Imu, queue_size = 1)
        # sub.append(subscriber)
        pub.append(publisher)
        imu_pub.append(imu_publisher)

    do_drive = True
    while not rospy.is_shutdown() or do_drive == False:
        try:
            now = rospy.Time.now()
            cur_odom = Odometry()
            cur_odom.header.frame_id = "/map"
            cur_odom.header.stamp = now
            for i in range(num_clients):
                cur_odom.pose.pose.position.x = clients[i].pose[0]
                cur_odom.pose.pose.position.y = clients[i].pose[1]
                cur_odom.pose.pose.position.z = clients[i].pose[2]
                rot = clients[i].heading
                #wrap around
                if(rot>2*m.pi):
                    rot -= 2*m.pi
                if(rot< -2*m.pi):
                    rot += 2*m.pi
                cur_odom.pose.pose.orientation = angle_to_quaternion(rot)
                cur_odom.twist.twist.linear.x = clients[i].twist[0]
                cur_odom.twist.twist.linear.y = clients[i].twist[1]
                cur_odom.twist.twist.linear.z = clients[i].twist[2]
                cur_odom.twist.twist.angular.x = clients[i].twist[3]
                cur_odom.twist.twist.angular.y = clients[i].twist[4]
                cur_odom.twist.twist.angular.z = clients[i].twist[5]
                pub[i].publish(cur_odom)

        	cur_imu = Imu()
        	cur_imu.header.frame_id = "/base_link"
        	cur_imu.header.stamp = now
        	for i in range(num_clients):
        		cur_imu.orientation.x = clients[i].pose[3]
        		cur_imu.orientation.y = clients[i].pose[4]
        		cur_imu.orientation.z = clients[i].pose[5]
        		cur_imu.orientation.w = clients[i].pose[6]

        		cur_imu.angular_velocity.x = clients[i].twist[3]
        		cur_imu.angular_velocity.y = clients[i].twist[4]
        		cur_imu.angular_velocity.z = clients[i].twist[5]

        		cur_imu.linear_acceleration.x = clients[i].accel[0]
        		cur_imu.linear_acceleration.y = clients[i].accel[1]
        		cur_imu.linear_acceleration.z = clients[i].accel[2]

        		imu_pub[i].publish(cur_imu)

                if clients[i].aborted:
                    print("Client socket problem, stopping driving.")
                    do_drive = False
        except KeyboardInterrupt:
            exit()
        
        except Exception as e:
            print(traceback.format_exc())

        except:
            pass
    # Exist Scene
    msg = '{ "msg_type" : "exit_scene" }'
    for i in range(num_clients):
        clients[i].send(msg)

    time.sleep(1.0)

    # Close down clients
    print("waiting for msg loop to stop")
    for i in range(num_clients):
        clients[i].stop()

    print("clients to stopped")


if __name__ == "__main__":
    env_list = [
       "warehouse",
       "generated_road",
       "avc2",
       "generated_track",
       "MUSHR_track",
       "MUSHR_benchmark"
    ]
    rospy.init_node("connector")
    parser = argparse.ArgumentParser(description='mushr')
    parser.add_argument('--agents', type=str, default=1, help='number of cars to use')
    parser.add_argument('--manual_control', type=str, default="keyboard", help='type of control interface')
    parser.add_argument('--env_name', type=str, default='generated_road', help='name of donkey sim environment', choices=env_list)

    args, unknown = parser.parse_known_args()
    test_clients(args)

