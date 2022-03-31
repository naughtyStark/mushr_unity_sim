#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import Quaternion, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
import math as m
from Bezier import *
import yaml  # for loading config
import time

DEG2RAD = 1/57.3
RAD2DEG = 57.3
GRAVITY_VEC = np.array([0,0,9.8])

def constrain(x,llim,ulim):
    if(x>=ulim):
        return ulim
    if(x<=llim):
        return llim
    return x

def length(x):
    return np.linalg.norm(x)

def signum(x):
    if(x >= 0):
        return 1
    else:
        return -1

class autopilot():
    def __init__(self):
        config = None
        with open('car_config.yaml') as f:
            config = yaml.safe_load(f)
        if(config == None):
            print("no config found")
            exit()
        # vehicle parameters
        self.D_const = config['friction_coefficient']
        self.B_const = 1.3
        self.C_const = 0.8  # tire model constants
        self.wb = config['wheelbase']
        self.tw = config['trackwidth']
        self.cgH = config['cg_height']
        self.cgR = config['cg_ratio']
        self.alpha_lim = config['slip_angle_limit']
        self.phi_lim = config['steering_limit']
        self.MOI_Z = config['MOI_Z']
        self.mass = config['mass']
        self.wheelspeed_lim = config['wheelspeed_lim']
        self.cruise_speed = 5
        # state variables
        self.posNED = np.zeros(3)
        self.posNED0 = np.zeros(3)
        self.velNED = np.zeros(3)
        self.lastVelNED = np.zeros(3)
        self.velBF = np.zeros(3)
        self.accNED = np.zeros(3)
        self.accBF = np.zeros(3)
        self.rotBF = np.zeros(3)
        self.quat = np.array([1,0,0,0],dtype=float)
        self.rpy = np.zeros(3)
        self.Tnb = np.zeros((3,3))
        self.Tbn = np.zeros((3,3))
        self.rcin = np.zeros(12)
        self.Beta = 0
        self.gndspeed = 0
        self.RPM = 0
        self.reset = False
        self.cur_time = 0
        self.fresh_data = False

        # control variables
        self.throttle = 0
        self.rate_err = np.zeros(3)
        self.rpy_rt_sp = np.zeros(3)
        self.posW_sp = np.zeros(3)
        self.velW_sp = np.zeros(3)
        self.velBF_sp = np.zeros(3)
        self.speed_sigma = 0
        self.Force_Error = 0
        self.height_sigma = 0
        self.rate_integral = np.zeros(3)
        self.last_rotBF = np.zeros(3)
        self.last_time = time.time()
        self.Vhat_cur = np.zeros(3)
        self.wheelspeed = 0
        self.steering = 0
        self.steering_estimate = 0
        self.wheelspeed_estimate = 0
        # auto mode variables:
        self.target_WP = None
        self.target_WP = None
        self.load_WP("simple_loop.npy")
        self.cur_target = np.zeros(3)
        self.cur_Vhat = np.array([1,0,0])
        self.target_Vhat = None
        self.control_Tc = 0.1 # 50 Hz
        self.speed_time_constant = 0.2
        self.rate_demand = np.zeros(3)
        self.rate_gain = 0.5
        self.trajectory_time_constant = config["trajectory_time_constant"]
        self.wp_dist = 1
        self.wp_dist_final = self.wp_dist
        self.auto_flag = False
        self.setup_complete = False
        self.wp_index = 0
        self.max_index = 0
        self.wp_list = None
        self.segment_t = 0
        self.last_U = np.zeros(2)
        self.last_rate = np.zeros(2)
        ## logging:
        self.log_file = None
        self.data_log = []

        self.path = Odometry()
        self.path.header.frame_id = "map"
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.odom_broadcaster.sendTransform((0, 0, 0),(0,0,0,1),rospy.Time.now(),"odom","map")
        self.path_pub = rospy.Publisher("path", Odometry, queue_size = 10)
        self.odom_sub = rospy.Subscriber("/car1/car_odom", Odometry, self.OdomCallback)
        self.imu_sub = rospy.Subscriber("/car1/imu", Imu, self.IMUCallback)
        self.control_pub = rospy.Publisher("/car1/mux/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=1)
        self.cur_goal_pub = rospy.Publisher("/car1/car_goal",Odometry, queue_size = 10)


        time.sleep(2)
        self.auto_setup()

    
    def load_WP(self,filename):
        self.target_WP = np.load(filename,allow_pickle=True)
        self.target_WP = np.array(self.target_WP,dtype=float)

    def auto_setup(self):
        if(self.target_WP is None):
            print("no waypoints bruh")
            return 0
        self.target_Vhat = np.zeros_like(self.target_WP) # 0 out everything
        self.max_index = len(self.target_WP) - 1
        self.wp_index = 1 # starting from 0

        for i in range(1,len(self.target_WP)-1): # all points except first and last
            V_prev = self.target_WP[i] - self.target_WP[i-1]
            V_next = self.target_WP[i+1] - self.target_WP[i]
            self.target_Vhat[i] = (V_next + V_prev)/np.linalg.norm(V_next + V_prev)
            # print("vhat:",self.target_Vhat[i])
        self.target_Vhat[0] = self.forward_vector()
        self.target_Vhat[0][2] = 0
        self.target_Vhat[-1] = self.forward_vector()
        self.target_Vhat[-1][2] = 0.0
        self.target_Vhat[-1] /= np.linalg.norm(self.target_Vhat[-1])
        self.target_Vhat[0] /= np.linalg.norm(self.target_Vhat[0])
        self.cur_target = self.target_WP[0]
        self.cur_Vhat = self.target_Vhat[0]
        self.auto_flag = True
        # print(self.posNED)
        N = 20
        self.wp_list = np.zeros((N*len(self.target_WP),6)) # x,y,z, x^, y^, z^
        for i in range(len(self.target_WP)-1):
            P0 = self.target_WP[i]
            P3 = self.target_WP[i+1]
            P1,P2 = get_Intermediate_Points_generic(P0,P3,self.target_Vhat[i],self.target_Vhat[i+1],self.cruise_speed,compliment=False)
            bx,by,bz = get_bezier(P0,P1,P2,P3,float(N))
            for j in range(N):
                Curvature,Direction,Normal = get_CTN(P0,P1,P2,P3,float(j)/float(N))
                self.path.header.stamp = rospy.Time.now()
                yaw = m.atan2(Direction[0],Direction[1])
                odom_quat = tf.transformations.quaternion_from_euler(0,0,yaw)
                self.path.pose.pose = Pose(Point(by[j],bx[j],-bz[j]), Quaternion(*odom_quat))
                self.path_pub.publish(self.path)
                time.sleep(0.01)
        self.max_index = len(self.wp_list) - 1  # maximum index 
        self.setup_complete = True

    def auto_manager(self, speed_target):
        if(not self.setup_complete):
            return
        dist = np.linalg.norm(self.posNED - self.cur_target)
        self.wp_dist = min(4,max(0.4, speed_target**2 / (self.D_const*9.8)))
        upper_lim = len(self.target_WP) - 1
        N = float(10)
        increment = 1.0/N
        while(dist < self.wp_dist):
            self.segment_t += increment
            if(self.segment_t > 1 and self.wp_index < upper_lim - 1):
                self.wp_index += 1
                self.segment_t = 0
            self.segment_t = constrain(self.segment_t,0,1)
            self.wp_index = constrain(self.wp_index, 0, upper_lim - 1)
            P0 = self.target_WP[self.wp_index]
            P3 = self.target_WP[self.wp_index + 1]
            P1,P2 = get_Intermediate_Points_generic(P0,P3,self.target_Vhat[self.wp_index],self.target_Vhat[self.wp_index + 1],5,compliment=False, jerk_opt=True)
            bx,by,bz = get_bezier_point(P0, P1, P2, P3, self.segment_t)
            Curvature,Direction,Normal = get_CTN(P0,P1,P2,P3, self.segment_t)
            self.cur_target = np.array([bx,by,bz])
            self.cur_Vhat = Direction
            dist = np.linalg.norm(self.posNED - self.cur_target)
            if(dist < self.wp_dist_final and self.wp_index == upper_lim - 1):
                self.auto_flag = False
                print("hit")

    def OdomCallback(self,data):
        now = time.time()
        self.posNED[1] = data.pose.pose.position.x
        self.posNED[0] = data.pose.pose.position.y
        self.posNED[2] = -data.pose.pose.position.z
        self.quat[0] = data.pose.pose.orientation.w
        self.quat[1] = data.pose.pose.orientation.x
        self.quat[2] = data.pose.pose.orientation.y
        self.quat[3] = data.pose.pose.orientation.z
        self.rpy = self.rpy_from_quat()
        self.rpy[2] -= m.pi/2
        self.rpy[2] *= -1
        self.quat = self.quat_from_rpy()
        self.rotBF[0] = 0.2*(data.twist.twist.angular.y) + 0.8*self.rotBF[0]
        self.rotBF[1] = 0.2*(data.twist.twist.angular.x) + 0.8*self.rotBF[1]
        self.rotBF[2] = 0.2*(-data.twist.twist.angular.z) + 0.8*self.rotBF[2]
        self.velBF[0] = 0.2*(data.twist.twist.linear.y) + 0.8*self.velBF[0]
        self.velBF[1] = 0.2*(data.twist.twist.linear.x) + 0.8*self.velBF[1]
        self.velBF[2] = 0.2*(-data.twist.twist.linear.z) + 0.8*self.velBF[2]
        self.calc_Transform()
        self.lastVelNED = self.velNED
        self.velNED = np.matmul(self.Tbn, self.velBF)
        self.speed = np.linalg.norm(self.velBF)
        self.Beta = 0.2*m.atan2(self.velBF[1], self.velBF[0]) + 0.8*self.Beta
        if(self.speed > 1):
            past_vec = np.matmul(self.Tnb, self.lastVelNED)/np.linalg.norm(self.lastVelNED)
            cur_vec = self.velBF/np.linalg.norm(self.velBF)
            rot_vec = np.cross(past_vec, cur_vec)
            if(np.linalg.norm(rot_vec)!=0):    
                rot_vec /= np.linalg.norm(rot_vec) ## ples no 0/0
            else:
                rot_vec = np.array([0,0,1])
            self.phi_dot = 0.2*(50*rot_vec*np.arccos(constrain(np.dot(past_vec, cur_vec), -1.0, 1.0))) + 0.8*self.phi_dot
        else:
            self.phi_dot = self.rotBF
        # print("beta: ",round(self.Beta*57.3,2))


        if(self.target_WP != None and self.setup_complete):
            target_speed = 4
            self.auto_manager(target_speed)
            self.mode_auto(self.cur_target, self.cur_Vhat, target_speed)
            dt = (time.time() - now)*1e3
            # print("delta_t: ", dt)
        else:
            return

    def IMUCallback(self, data):
        self.accBF[0] = data.linear_acceleration.y
        self.accBF[1] = data.linear_acceleration.x
        self.accBF[2] = -data.linear_acceleration.z

    def calc_Transform(self):
        q00 = self.quat[0]**2;
        q11 = self.quat[1]**2;
        q22 = self.quat[2]**2;
        q33 = self.quat[3]**2;
        q01 =  self.quat[0]*self.quat[1];
        q02 =  self.quat[0]*self.quat[2];
        q03 =  self.quat[0]*self.quat[3];
        q12 =  self.quat[1]*self.quat[2];
        q13 =  self.quat[1]*self.quat[3];
        q23 =  self.quat[2]*self.quat[3];

        self.Tbn = np.zeros((3,3)) # transform body->ned
        self.Tbn[0][0] = q00 + q11 - q22 - q33;
        self.Tbn[1][1] = q00 - q11 + q22 - q33;
        self.Tbn[2][2] = q00 - q11 - q22 + q33;
        self.Tbn[0][1] = 2*(q12 - q03);
        self.Tbn[0][2] = 2*(q13 + q02);
        self.Tbn[1][0] = 2*(q12 + q03);
        self.Tbn[1][2] = 2*(q23 - q01);
        self.Tbn[2][0] = 2*(q13 - q02);
        self.Tbn[2][1] = 2*(q23 + q01);

        self.Tnb = self.Tbn.transpose(); # transform ned->body

    def publish_controls(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.steering_angle = -self.steering
        msg.drive.speed = self.wheelspeed
        self.control_pub.publish(msg)

    def rpy_from_quat(self):
        y = np.zeros(3)
        y[0] = m.atan2((2.0*(self.quat[2]*self.quat[3]+self.quat[0]*self.quat[1])) ,
                        (self.quat[0]**2 - self.quat[1]**2 - self.quat[2]**2 + self.quat[3]**2))
        y[1] = -m.asin(2.0*(self.quat[1]*self.quat[3]-self.quat[0]*self.quat[2]));
        y[2] = m.atan2((2.0*(self.quat[1]*self.quat[2]+self.quat[0]*self.quat[3])) ,
                        (self.quat[0]**2 + self.quat[1]**2 - self.quat[2]**2 - self.quat[3]**2))
        return y

    def quat_from_rpy(self):
        u1 = m.cos(0.5*self.rpy[0]);
        u2 = m.cos(0.5*self.rpy[1]);
        u3 = m.cos(0.5*self.rpy[2]);
        u4 = m.sin(0.5*self.rpy[0]);
        u5 = m.sin(0.5*self.rpy[1]);
        u6 = m.sin(0.5*self.rpy[2]);
        quat = np.zeros(4)
        quat[0] = u1*u2*u3+u4*u5*u6;
        quat[1] = u4*u2*u3-u1*u5*u6;
        quat[2] = u1*u5*u3+u4*u2*u6;
        quat[3] = u1*u2*u6-u4*u5*u3;
        return quat

    def down_vector(self):
        V = np.ones(3)
        V[0] = 2 * (self.quat[1]*self.quat[3] + self.quat[0]*self.quat[2])
        V[1] = 2 * (self.quat[2]*self.quat[3] - self.quat[0]*self.quat[1])
        V[2] = 1 - 2 * (self.quat[1]*self.quat[1] + self.quat[2]*self.quat[2])
        return V

    def right_vector(self):
        U = np.ones(3)
        U[0] = 2 * (self.quat[1]*self.quat[2] - self.quat[0]*self.quat[3])
        U[1] = 1 - 2 * (self.quat[1]*self.quat[1] + self.quat[3]*self.quat[3])
        U[2] = 2 * (self.quat[2]*self.quat[3] + self.quat[0]*self.quat[1])
        return U

    def forward_vector(self):
        W = np.ones(3)
        W[0] = 1 - 2 * (self.quat[2]*self.quat[2] + self.quat[3]*self.quat[3])
        W[1] = 2 * (self.quat[1]*self.quat[2] + self.quat[0]*self.quat[3])
        W[2] = 2 * (self.quat[1]*self.quat[3] - self.quat[0]*self.quat[2])
        return W

    def pub_cur_goal(self):
        cur_goal = Odometry()
        cur_goal.header.frame_id = "map"
        cur_goal.header.stamp = rospy.Time.now()
        yaw = m.atan2(self.cur_Vhat[0], self.cur_Vhat[1])
        odom_quat = tf.transformations.quaternion_from_euler(0,0,yaw)
        cur_goal.pose.pose = Pose(Point(self.cur_target[1],self.cur_target[0],-self.cur_target[2]), Quaternion(*odom_quat))
        self.cur_goal_pub.publish(cur_goal)

    def mode_auto(self, pos_target, vel_target, speed_target):
        P3 = pos_target - self.posNED
        P0 = np.zeros(3)
        if(not self.auto_flag):
            self.wheelspeed = 0
            self.steering = 0
            self.publish_controls()
        speed = np.linalg.norm(self.velNED)
        Vhat_cur = np.zeros(3)
        if(speed < 0.1):
            Vhat_cur = self.forward_vector()
        else:
            Vhat_cur = self.velNED/speed
        self.pub_cur_goal()
        Vhat_final = vel_target/np.linalg.norm(vel_target)
        P1, P2 = get_Intermediate_Points_generic(P0, P3, Vhat_cur,Vhat_final, speed_target,compliment=False, jerk_opt=True)
        u = get_T_generic(P0,P1,P2,P3,max(speed,0.1),self.control_Tc)
        t = u[0]
        t_ = u[1:]
        P1 = np.matmul(self.Tnb,P1)
        P2 = np.matmul(self.Tnb,P2)
        P3 = np.matmul(self.Tnb,P3) #this self.Tnb may be a problem at large alpha since it transforms everything into body frame not velocity frame
        g_bf = np.matmul(self.Tnb,GRAVITY_VEC) # convert gravity vector to body frame
        Curvature_t_bf,Direction_t_bf,Normal_t_bf = get_CTN(P0,P1,P2,P3,t)
        C_max = np.zeros(2)
        for i in range(2):
            C_,T_,N_ = get_CTN(P0,P1,P2,P3,t_[i])
            C_max[i] = C_
        C_max = np.max(C_max)

        ### the following code is specific to cars, sort of like inverse kinematics
        C = Curvature_t_bf*Normal_t_bf
        horizontal_curvature = C[1] ##*m.sin(self.Beta))
        ## get the velocity in body frame, rotate normal vector into vel frame then find phi dot.
        phi_dot = horizontal_curvature*speed

        buf = ((self.D_const*9.8)**2 - (speed*phi_dot)**2)
        buf = max(0, buf)
        acc_buffer = m.sqrt(buf)
        delta_V_max = acc_buffer*self.speed_time_constant

        max_speed_lookahead = m.sqrt(acc_buffer*np.linalg.norm(P3))
        max_speed_current = m.sqrt(self.D_const*9.8/max(m.fabs(horizontal_curvature),0.01))
        max_speed_later = m.sqrt(self.D_const*9.8/max(C_max,0.01))

        speed_target = min(speed_target, max_speed_lookahead, max_speed_current, max_speed_later) ## find the least of them all.
        speed_ref = constrain(speed_target, speed - delta_V_max, speed + delta_V_max)  # constrain change using acceleration buffer.
        speed_dot = (speed_ref - np.linalg.norm(self.velBF))/self.speed_time_constant
        # print("speed_ref =", speed_ref)
        self.low_level_controller(phi_dot, speed_dot, speed_ref, horizontal_curvature)  # low level controller for steering, wheelspeed.
        # self.low_level_controller(1.33, speed_dot, speed_ref, horizontal_curvature)  # low level controller for steering, wheelspeed.
        ## publish controls.
        self.publish_controls()

    def low_level_controller(self, phi_dot, speed_dot, speed_ref, horizontal_curvature):
        ## simple small slip assumption controller:
        # self.wheelspeed = self.velBF[0] + speed_dot*self.speed_time_constant
        # print(self.wheelspeed)
        speed = length(self.velBF[:2])
        if(0):
            self.steering = constrain(m.atan(horizontal_curvature/self.wb), -self.phi_lim, self.phi_lim)#constrain(m.atan(phi_dot/(speed*self.wb)), -self.phi_lim, self.phi_lim)
            self.steering += self.Beta
            self.wheelspeed = speed_ref + speed_dot*self.speed_time_constant
        else:
            self.nonlinear_model_inversion(phi_dot, speed_dot, speed_ref)

    def tire_model(self, W, Vbody, wheelspeed, B, angle, side):
        Vx = Vbody*m.cos(B - angle)
        Vy = Vbody*m.sin(B - angle)
        sx = (wheelspeed - Vx)/max(wheelspeed, Vx, 0.1)  # prevent 0/0 cases.
        sy = m.atan2(Vy + side*self.wb*0.5*(self.rotBF[2]), wheelspeed)  # prevent 0/0 case
        slip = m.sqrt(sx*sx + sy*sy)
        # print("slip: ", slip)
        gamma = m.atan2(-sy, sx)
        return W*self.D_const*m.sin(self.C_const*m.atan(self.B_const*slip)), gamma

    def pvdot(self,mass, Vbody, Wf, Wr, d, wh, B):
        m_inv = 1/float(mass)
        mv_inv = 1/float(mass*Vbody)

        Ff, gf = self.tire_model(Wf, Vbody, wh, B, d, 1)
        Fr, gr = self.tire_model(Wr, Vbody, wh, B, 0,-1)  # rear tires have no steering angle.

        th_f = d - (B - gf)
        th_r = gr - B

        phi_dot_ = mv_inv*( Ff*m.sin(th_f) + Fr*m.sin(th_r) )
        vel_dot_ = m_inv *( Ff*m.cos(th_f) + Fr*m.cos(th_r) )

        if(self.F_m == None):
            M = np.zeros((2,2))
            M[0,0] = mv_inv*m.sin(th_f)
            M[0,1] = mv_inv*m.sin(th_r)
            M[1,0] = m_inv *m.cos(th_f)
            M[1,1] = m_inv *m.cos(th_r)
            F_m = np.zeros(2)
            if(np.linalg.det(M) != 0):
                velvec = self.velBF/self.speed
                vel_dot_local = np.dot(self.accBF, velvec)
                X = np.array([self.phi_dot[2], vel_dot_])
                # print(self.phi_dot[2], phi_dot_, vel_dot_local, vel_dot_)
                # print("gamma: ", 57.3*gf, 57.3*gr)
                F_m = np.linalg.solve(M,X)
            if(F_m[0] != 0 and F_m[1] != 0):
                self.F_m = F_m
                # print("forces: ",np.round(self.F_m,2), np.round(Ff,2), np.round(Fr,2))
            else:
                self.F_m = np.array([Ff, Fr])

        return phi_dot_, vel_dot_

    def nonlinear_model_inversion(self, phi_dot_setp, vel_dot_setp, speed_ref):
        V = np.linalg.norm(self.velBF[:2]) ## just in case the system decides to fly
        st = self.steering_estimate
        wh = self.wheelspeed_estimate  # use the last sent values as measurement (damn could this be worse?)
        Wb2 = self.mass*9.8*0.5
        Lt = 2*self.accBF[0]*self.mass*self.cgH/self.wb
        Wr = Wb2 + Lt
        Wf = Wb2 - Lt
        del_st = 0.1  # this is in radians people
        del_wh = 0.1  # 0.2 m/s delta in 0.02 seconds

        self.F_m = None
        phi_dot_, vel_dot_ = self.pvdot(self.mass, V, Wf, Wr, st, wh, self.Beta)
        # print("calced and measd: ", 57.3*phi_dot_, 57.3*self.phi_dot[2])

        phi_dot_st, vel_dot_st = self.pvdot(self.mass, V, Wf, Wr, st + del_st, wh, self.Beta)
        phi_dot_wh, vel_dot_wh = self.pvdot(self.mass, V, Wf, Wr, st, wh + del_wh, self.Beta)
        
        dphi_dst = (phi_dot_st - phi_dot_)/del_st
        dphi_dwh = (phi_dot_wh - phi_dot_)/del_wh

        dvel_dst = (vel_dot_st - vel_dot_)/del_st
        dvel_dwh = (vel_dot_wh - vel_dot_)/del_wh

        M = np.zeros((2,2))
        M[0,0] = dphi_dst
        M[0,1] = dphi_dwh
        M[1,0] = dvel_dst
        M[1,1] = dvel_dwh
        if(self.speed < 1):
            M[0,0] = constrain(M[0,0], 10, 15)
            M[1,1] = constrain(M[1,1], 1, self.D_const*self.B_const*self.C_const*9.8)
            M[0,1] = 0
            M[1,0] = 0

        # print(np.round(M,2))

        del_phi_dot = phi_dot_setp - self.phi_dot[2]
        velvec = self.velBF/self.speed
        del_vel_dot = vel_dot_setp - np.dot(self.accBF,velvec)  ## this works ONLY because the joosbox has the initial tangent aligned with velocity vector
        X = np.array([del_phi_dot, del_vel_dot])
        # print(np.round(M,2))
        if(np.linalg.det(M) != 0):
            delta = np.linalg.solve(M,X)
        else:
            delta = np.zeros(2)
        # print(delta)
        delta[0] = constrain(delta[0], -del_st*2, del_st*2)
        delta[1] = constrain(delta[1], -del_wh, del_wh)
        output_st = st + delta[0]
        # output_st = constrain(output_st, self.Beta - 0.3, self.Beta + 0.3)
        output_st = constrain(output_st, -self.phi_lim, self.phi_lim)
        output_wh = constrain(wh + delta[1], 0, speed_ref*1.5)
        print("nlmi output: ", round(output_st,2), round(output_wh, 2)) #, round(st,2), round(wh,2))
        # output_st = 0.2
        # output_wh = 3
        self.steering = output_st
        self.steering_estimate = 0.8*self.steering + 0.2*self.steering_estimate  # update steering estimate
        self.wheelspeed = output_wh
        self.wheelspeed_estimate = 0.8*self.wheelspeed + 0.2*self.wheelspeed_estimate



if __name__ == '__main__':
    rospy.init_node("controller")
    init_publisher = rospy.Publisher("/car1/initialpose", PoseWithCovarianceStamped, queue_size=1)
    time.sleep(1)  # wait for the publisher to be created and recognized by subscribers.
    now = rospy.Time.now()
    temp_cur_pose = PoseWithCovarianceStamped()
    temp_cur_pose.header.frame_id = "/map"
    temp_cur_pose.header.stamp = now
    temp_cur_pose.pose.pose.position.x = 0
    temp_cur_pose.pose.pose.position.y = 0
    temp_cur_pose.pose.pose.position.z = 0
    temp_cur_pose.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, m.pi/2))
    init_publisher.publish(temp_cur_pose)
    joosbocs = autopilot()
    rospy.spin()
