#!/usr/bin/env python

# Basic ROS imports
import roslib
roslib.load_manifest('underwater_vehicle_dynamics')
import rospy
import PyKDL
import sys
sys.path.append('../../underwater_snakerobot_controller/msg')

# import msgs
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

# from underwater_snakerobot_controller.msg import HardwareCommand
from underwater_snakerobot_controller.msg import RobotState

#import services
from std_srvs.srv import Empty

# More imports
from numpy import *
import tf

time_flag = 0

class Dynamics :


    def getConfig(self) :
        """ Load parameters from the rosparam server """
        self.num_actuators = rospy.get_param(self.vehicle_name+"/num_actuators")

        self.period = rospy.get_param(self.vehicle_name + "/dynamics" + "/period")
        self.mass = rospy.get_param(self.vehicle_name + "/dynamics" + "/mass")
        self.link_length = rospy.get_param(self.vehicle_name + "/dynamics" + "/link_length")
        self.gravity_center = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/gravity_center"))
        self.g = rospy.get_param(self.vehicle_name + "/dynamics" + "/g")
        self.radius = rospy.get_param(self.vehicle_name + "/dynamics" + "/radius")
        self.ctf = rospy.get_param(self.vehicle_name + "/dynamics" + "/ctf")
        self.ctb = rospy.get_param(self.vehicle_name + "/dynamics" + "/ctb")
        self.actuators_tau = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_tau")
        self.actuators_maxsat= rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_maxsat")
        self.actuators_minsat = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_minsat")
        self.actuators_gain = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_gain")
        self.screw_gain = rospy.get_param(self.vehicle_name + "/dynamics" + "/screw_gain")
        self.dzv = rospy.get_param(self.vehicle_name + "/dynamics" + "/dzv")
        self.dv = rospy.get_param(self.vehicle_name + "/dynamics" + "/dv")
        self.dh = rospy.get_param(self.vehicle_name + "/dynamics" + "/dh")
        self.density = rospy.get_param(self.vehicle_name + "/dynamics" + "/density")
        self.tensor = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/tensor"))
        self.damping = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/damping"))
        self.quadratic_damping = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/quadratic_damping"))

	self.am=rospy.get_param(self.vehicle_name + "/dynamics"+"/allocation_matrix")

        self.p_0 = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/initial_pose"))
        self.v_0 = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/initial_velocity"))
        self.v_dot_0 = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/initial_accel"))
        self.desired_joint_angle = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/desired_joint_angle"))
        self.joint_angle_limit = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/joint_angle_limit"))
        self.joint_angle = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/initial_joint_angle"))
        self.joint_angular_vel = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/initial_joint_angular_vel"))
        self.joint_angular_acc = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/initial_joint_torque"))
        self.I_joint = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/moment_of_inertial_joint"))
        self.screw_vel_limit = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/screw_vel_limit"))
        self.screw_angle = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/screw_angle"))
        self.screw_angular_vel = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/screw_angular_vel"))
        self.screw_angular_acc = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/screw_angular_acc"))
        self.frame_id = rospy.get_param(self.vehicle_name + "/dynamics" + "/frame_id")
        self.external_force_topic = rospy.get_param(self.vehicle_name + "/dynamics" + "/external_force_topic")

#       Currents data
        self.current_mean = array( rospy.get_param("dynamics/current_mean") )
        self.current_sigma = array( rospy.get_param("dynamics/current_sigma") )
        self.current_min = array( rospy.get_param("dynamics/current_min") )
        self.current_max = array( rospy.get_param("dynamics/current_max") )

        self.uwsim_period = rospy.get_param(self.vehicle_name + "/dynamics/uwsim_period")


    def s(self, x) :
        """ Given a 3D vector computes the 3x3 antisymetric matrix """
#        rospy.loginfo("s(): \n %s", x)
        ret = array([0.0, -x[2], x[1], x[2], 0.0, -x[0], -x[1], x[0], 0.0 ])
        return ret.reshape(3,3)


    def generalizedForce(self, du):
	""" Computes the generalized force as B*u,
        being B the allocation matrix and u the control input """
        t = zeros(6)
        # self.u[0] = 2.1
        t[0] = self.u[0]/2
        # t[5] = -0.2

        return t


    def coriolisMatrix(self, v):
        s1 = self.s(dot(self.M[0:3,0:3], v[0:3]) + dot(self.M[0:3,3:6], v[3:6]))
        s2 = self.s(dot(self.M[3:6,0:3], v[0:3]) + dot(self.M[3:6,3:6], v[3:6]))
        c = zeros((6, 6))
        c[0:3,3:6] = -s1
        c[3:6,0:3] = -s1
        c[3:6,3:6] = -s2
        return c

    def dampingMatrix(self, v):
        # lineal hydrodynamic damping coeficients
        Xu = self.damping[0]
        Yv = self.damping[1]
        Zw = self.damping[2]
        Kp = self.damping[3]
        Mq = self.damping[4]
        Nr = self.damping[5]

        # quadratic hydrodynamic damping coeficients
        Xuu = self.quadratic_damping[0]    #[Kg/m]
        Yvv = self.quadratic_damping[1]    #[Kg/m]
        Zww = self.quadratic_damping[2]    #[Kg/m]
        Kpp = self.quadratic_damping[3]    #[Kg*m*m]
        Mqq = self.quadratic_damping[4]    #[Kg*m*m]
        Nrr = self.quadratic_damping[5]    #[Kg*m*m]

        d = diag([Xu + Xuu*abs(v[0]),
                  Yv + Yvv*abs(v[1]),
                  Zw + Zww*abs(v[2]),
                  Kp + Kpp*abs(v[3]),
                  Mq + Mqq*abs(v[4]),
                  Nr + Nrr*abs(v[5])])
        return d

    def gravity(self):
	""" Computes the gravity and buoyancy forces. Assumes a sphere model for now """
        #Weight and Flotability
        W = self.mass * self.g # [Kg]

        #If the vehicle moves out of the water the flotability decreases
	#FIXME: Assumes water surface at 0.0. Get this value from uwsim.
        if self.p[2] < 0.0:
            r = self.radius + self.p[2]
            if r < 0.0:
                r = 0.0
        else :
            r = self.radius

	#TODO: either set as parameter, since different functions may be desired for different vehicles
	#      or define common models and let the user choose one by the name
	#      Eventually let this part to bullet inside uwsim (HfFluid)
        F = ((4 * math.pi * pow(r,3))/3)*self.density*self.g

        # gravity center position in the robot fixed frame (x',y',z') [m]
        zg = self.gravity_center[2]

        g = array([(W - F) * sin(self.p[4]),
                   -(W - F) * cos(self.p[4]) * sin(self.p[3]),
                   -(W - F) * cos(self.p[4]) * cos(self.p[3]),
                   zg*W*cos(self.p[4])*sin(self.p[3]),
                   zg*W*sin(self.p[4]),
                   0.0])

        return g

    def transMatrixI2B(self,theta):
        trans = zeros((6, 6))
        trans[5,5] = 1
        trans[0,0] = cos(theta)
        trans[0,1] = sin(theta)
        trans[1,0] = -sin(theta)
        trans[1,1] = cos(theta)
        return trans

    def transMatrixB2I(self,theta):
        trans = zeros((6, 6))
        trans[5,5] = 1
        trans[0,0] = cos(theta)
        trans[0,1] = -sin(theta)
        trans[1,0] = sin(theta)
        trans[1,1] = cos(theta)
        return trans


    def inverseDynamic(self) :
        """ Given the setpoint for each thruster, the previous velocity and the
            previous position computes the v_dot """
        global time_flag

        self.u[0] = 1.0
        self.u[2] = 1.0
        self.u[1] = 0.1
        ###################Joint and screw dynamics##########################
        self.jointDynamics()
        self.screwDynamics()
        ###################Robot dynamics########################
        c = self.coriolisMatrix(self.v)
        # self.joint_angular_vel[1]
        d = self.dampingMatrix(self.v)
        c_v = -dot((c-d), self.v)
        internalForce_1to2 = dot(self.transMatrixI2B(self.joint_angle[1]), c_v/2)

        c_link2 = self.coriolisMatrix(self.v_link2)
        d_link2 = self.dampingMatrix(self.v_link2)
        c_v_link2 = -dot((c_link2-d_link2), self.v_link2)
        internalForce_2to1 = dot(self.transMatrixB2I(self.joint_angle[1]), c_v_link2/2)
       #######################################################
        t = array(zeros(6))
        t_link2 = array(zeros(6))

        # du = self.thrustersDynamics(self.u)
        # t = self.generalizedForce(du)
        t[0] = self.u[0] + self.u[2]*cos(self.joint_angle[1]) + internalForce_2to1[0]*0.3
        t[1] = self.u[2]/2*sin(self.joint_angle[1]) + internalForce_2to1[1]*0.3
        t[5] = self.link_length/2*(-self.u[2]/2*sin(self.joint_angle[1]))*0.003 + internalForce_2to1[2]*0.003
                                   # +(internalForce_1to2[1]+internalForce_2to1[1])*sin(self.joint_angle[1]))
        # t[0] += innerForce[0]
        # t[1] += innerForce[1]
        # t[5] += self.mass *self.link_length*innerForce[1]/4

        t_link2[0] = self.u[2] + self.u[0]*cos(self.joint_angle[1])+internalForce_1to2[0]*0.3
        t_link2[1] = -self.u[0]/2*sin(self.joint_angle[1])+internalForce_1to2[1]*0.3
        t_link2[5] = self.link_length/2*(-self.u[0]/2 * sin(self.joint_angle[1]))*0.003 + internalForce_1to2[2]*0.003
                                         # -(internalForce_1to2[1]+internalForce_2to1[1])*sin(self.joint_angle[1]))

        temp = (t[5]+t_link2[5])/2
        t[5] = temp
        t_link2[5] = temp
        # temp = (c_v[5]+c_v_link2[5])/2
        # c_v[5] = temp
        # c_v_link2[5] = temp
        # t[5] += temp
        # t_link2[5] += temp
        # temp = (c_v[5]+c_v_link2[5])/2
        # t[5] += temp
        # t_link2[5] += temp
        ###################################################
        # self.collisionForce[0] = self.collisionForce[0]/2
        # self.collisionForce[1] = -self.collisionForce[1]/2
        self.collisionForce[2] = 0
        self.collisionForce[3] = 0
        self.collisionForce[4] = 0
        # self.collisionForce[5] = self.collisionForce[5]/2
        ###################################################

        ##########test###########
        self.hydro[0] = c_v[0]*0.5
        self.hydro[1] = c_v[1]*0.5
        self.hydro[2] = c_v[5]*0.5
        self.innerForce[0] = internalForce_2to1[0]*0.3
        self.innerForce[1] = internalForce_2to1[1]*0.3
        self.innerForce[2] = internalForce_2to1[5]*0.3;
        self.innerForce[3] = self.joint_angular_acc[1]*4.5
        #########################

        v_dot = dot(self.IM, (t+(c_v+self.collisionForce)/2)) #t-c_v-g+collisionForce   +self.collisionForce
        v_dot = squeeze(asarray(v_dot)) #Transforms a matrix into an array
        # v_dot_link2 = (t_link2+c_v_link2+self.collisionForce)
        v_dot_link2 = dot(self.IM, (t_link2+(c_v_link2)/2)) #+self.collisionForce
        v_dot_link2 = squeeze(asarray(v_dot_link2)) #Transforms a matrix into an array

        # temp = (v_dot[5]+v_dot_link2[5])/2
        v_dot[5] = -self.joint_angular_acc[1]*3.5 + v_dot[5]# temp
        v_dot_link2[5] = +self.joint_angular_acc[1]*3.5 + v_dot_link2[5]#temp

        time_flag += 1
        if time_flag == 100 :
            # print('Thruster1:{0}, Thruster2:{1},Thruster3:{2}, Thruster4:{3}'.format(self.u[0], self.u[1], self.u[2], self.u[3]))
            # # print("DampingForce:",self.v_dot[0], "   vel:", self.v[0])
            print("vel_x:", self.v[0]," vel_y:",self.v[1]," vel_omega:",self.v[5])
            print("pos_x:", self.p[0]," pos_y:",self.p[1]," pos_omega:",self.p[5])
            # # print("DampingForce:",self.p[5], "   vel:", self.v[5])
            # print("c_v: ",c_v)
            # print("c_v_link2: ",c_v_link2)
            # # print("v_dot_link21: ",self.v_dot_link21)
            # # print("v_dot_link2: ",self.v_dot_link2)
            # # print("v_link21: ",self.v_link21)
            # print("v: ",self.v)
            # print("v_link2: ",self.v_link2)
            # print("t: ",t)
            # print("t_link2: ",t_link2)
            # # print("v_dot_link2: ",self.v_dot_link2)
            # # print("d: ",d)
            # print("c_v: ",c_v)
            # # print("c_link2: ",c_link2)
            # # print("d_link2: ",d_link2)
            # print("c_v_link2: ",c_v_link2)
            # print("self.u: ",self.u)
            # print("v_dot: ",v_dot)
            # # print("v_dot_link2: ",v_dot_link2)
            # print("self.joint_angle[1]:",self.joint_angle[1])
            # print("self.joint_angular_acc[1]:",self.joint_angular_acc[1])
            # print("innerForce_link2to1:",internalForce_2to1)
            # print("innerForce_link1to2:",internalForce_1to2)
            # # print("innerForce:",innerForce)
            # # print("joystick: ",self.u)
            # print("self.screw_angular_vel[i]: ", self.screw_angular_vel)
            # print("self.collisionForce: ", self.collisionForce)
            # print("**********************************************")
            time_flag = 0

        self.collisionForce=[0,0,0,0,0,0]
        accel = [v_dot, v_dot_link2]
        # print(accel[0])

        return accel


    def integral(self, x_dot, x, t) :
        """ Computes the integral o x dt """
        return (x_dot * t) + x


    def kinematics(self, v, theta) :
        """ Given the current velocity and the previous position computes the p_dot """
        roll = 0.0#self.p[3]
        pitch = 0.0#self.p[4]
        yaw = theta

        rec = [cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+cos(yaw)*sin(pitch)*sin(roll), sin(yaw)*sin(roll)+cos(yaw)*cos(roll)*sin(pitch),
               sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -cos(yaw)*sin(roll)+sin(pitch)*sin(yaw)*cos(roll),
               -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]
        rec = array(rec).reshape(3,3)

        to = [1.0, sin(roll)*tan(pitch), cos(roll)*tan(pitch),
              0.0, cos(roll), -sin(roll),
              0.0, sin(roll)/cos(pitch), cos(roll)/cos(pitch)]
        to = array(to).reshape(3,3)

        p_dot = zeros(6)
        p_dot[0:3] = dot(rec, v[0:3])
        p_dot[3:6] = dot(to, v[3:6])
        return p_dot

    def updateThrusters(self, thrusters) :
    	"""Receives the control input, saturates each component to maxsat or minsat, and multiplies each component by the actuator gain"""
	#TODO: Check the size of thrusters.data
        t = array(thrusters.data)
        for i in range(size(t)):
    	    if t[i]>self.actuators_maxsat:
    		    t[i]=self.actuators_maxsat
    	    elif t[i]<self.actuators_minsat:
    	        t[i]=self.actuators_minsat
	    self.u=t
        for i in range(size(t)):
           self.u[i] = self.u[i]*self.actuators_gain

    def thrustersDynamics(self, u):
        y = zeros(size(u))
        for i in range(size(u)):
            y[i] = (self.period * u[i] + self.actuators_tau[i] * self.y_1[i]) / (self.period + self.actuators_tau[i])

        self.y_1 = y
        return y

    def jointDynamics(self):
        self.joint_angular_acc[1] = (self.u[1]/2+self.quadratic_damping[5]
                                     *abs(self.joint_angular_vel[1])*self.joint_angular_vel[1]
                                     +self.damping[5]*self.joint_angular_vel[1])/self.I_joint
        if self.u[1] != 0:
            self.joint_angular_vel[1] = self.integral(self.joint_angular_acc[1],
                                                      self.joint_angular_vel[1],
                                                      self.period)
        else:
            self.joint_angular_vel[1] = 0
        self.joint_angle[1] = self.integral(self.joint_angular_vel[1],
                                            self.joint_angle[1],
                                            self.period)
        if abs(self.joint_angle[1]) > self.joint_angle_limit[1]:
            self.u[1] = 0.0
            self.joint_angular_acc[1] = 0
            self.joint_angular_vel[1] = 0
            if self.joint_angle[1] > 0:
                self.joint_angle[1] = self.joint_angle_limit[1]
            else:
                self.joint_angle[1] = -self.joint_angle_limit[1]

    def screwDynamics(self):
        self.screw_angular_acc[0] = self.u[0]*self.screw_gain
        self.screw_angular_acc[1] = self.u[2]*self.screw_gain
        for i in range (2):
            if self.screw_angular_acc[i] != 0:
                self.screw_angular_vel[i] = self.integral(self.screw_angular_acc[i],
                                                          self.screw_angular_vel[i],
                                                          self.period)
            else:
                self.screw_angular_vel[i] = 0
            if abs(self.screw_angular_vel[i]) > self.screw_vel_limit[i]:
                if self.screw_angular_vel[i] > 0:
                    self.screw_angular_vel[i] = self.screw_vel_limit[i]
                else:
                    self.screw_angular_vel[i] = -self.screw_vel_limit[i]
            self.screw_angle[i] = self.integral(self.screw_angular_vel[i],
                                                self.screw_angle[i],
                                                self.period)
            if self.screw_angle[i] >= (10*pi):
                self.screw_angle[i] = 0

    def updateCollision(self, force) :
        self.collisionForce=[force.wrench.force.x,force.wrench.force.y,force.wrench.force.z,force.wrench.torque.x,force.wrench.torque.y,force.wrench.torque.z]

    def pubPose(self, event):
        pose = RobotState()

        pose.positionW.x = self.p[0]
        pose.positionW.y = self.p[1]
        pose.positionW.z = self.p[2]

        pose.velocityB.x = self.v[0]
        pose.velocityB.y = self.v[1]
        pose.velocityB.z = 0.0

        pose.rotationE.x = 0.0
        pose.rotationE.y = 0.0
        pose.rotationE.z = self.p[5]

        pose.joint_angle = self.joint_angle[1]

        pose.hydroDynamics = self.hydro

        pose.internalForce = self.innerForce

        self.pub_pose.publish(pose)

        # Broadcast transform
        # br = tf.TransformBroadcaster()
        # br.sendTransform((self.p[0], self.p[1], self.p[2]), orientation,
        # rospy.Time.now(), "world", str(self.frame_id))

    def pubDatanav(self, event):
        odom = Odometry()

        orientation = tf.transformations.quaternion_from_euler(self.p[3], self.p[4],
                                                               self.p[5])#, 'sxyz')

        odom.pose.pose = Pose(Point(self.p[0], self.p[1], self.p[2]),
                              Quaternion(*orientation))

        # odom.twist.twist.linear.x=self.v[0]
        # odom.twist.twist.linear.y=self.v[1]
        # odom.twist.twist.linear.z=self.v[2]
        # odom.twist.twist.angular.x=self.v[3]
        # odom.twist.twist.angular.y=self.v[4]
        # odom.twist.twist.angular.z=self.v[5]

        self.pub_datanav.publish(odom)

    def pubScrew(self, event):
        screw = JointState()

        screw.name = ['base_link_to_screw_1_left_joint', 'base_link_to_screw_1_right_joint', 'base_link_to_link_2','link_2_to_screw_2_left_joint', 'link_2_to_screw_2_right_joint']
        screw.position = [self.screw_angle[0],self.screw_angle[0],
                          self.joint_angle[1],
                          self.screw_angle[1],self.screw_angle[1]]
        # screw.velocity = []
        # screw.velocity = [self.u[0]*self.screw_gain,self.u[0]*self.screw_gain,
        #                   0,
        #                   self.u[2]*self.screw_gain,self.u[2]*self.screw_gain]
        screw.effort = []

        self.pub_screw.publish(screw)

    def computeTf(self, tf):
        r = PyKDL.Rotation.RPY(math.radians(tf[3]), math.radians(tf[4]), math.radians(tf[5]))
        v = PyKDL.Vector(tf[0], tf[1], tf[2])
        frame = PyKDL.Frame(r, v)
        return frame

    def reset(self,req):
        self.v = self.v_0
        self.p = self.p_0
	return []

    def __init__(self):
        """ Simulates the dynamics of an AUV """

        if len(sys.argv) != 8:
          sys.exit("Usage: "+sys.argv[0]+" <namespace> <input_thruster_topic>  <output_pose_topic> <output_datanavigator_topic> <output_screw_topic>")

        self.namespace=sys.argv[1]
        self.vehicle_name=self.namespace
        self.input_thruster_topic=sys.argv[2]
        self.output_pose_topic=sys.argv[3]
        self.output_datanavigator_topic=sys.argv[4]
        self.output_screw_topic=sys.argv[5]

    #  Collision parameters
	self.collisionForce = [0,0,0,0,0,0]

    #   Load dynamic parameters
        self.getConfig()
        #self.altitude = -1.0
        self.y_1 = zeros(5)
        self.hydro = zeros(3)
        self.innerForce = zeros(4)

    #   Create publisher
        self.pub_pose= rospy.Publisher(self.output_pose_topic, RobotState, queue_size=1)
        self.pub_screw= rospy.Publisher(self.output_screw_topic, JointState, queue_size=1)
        self.pub_datanav= rospy.Publisher(self.output_datanavigator_topic, Odometry, queue_size=1)
        rospy.init_node("dynamics_"+self.vehicle_name)

    #   Init pose and velocity and period
        self.v = self.v_0
        self.p = self.p_0
        # self.v_dot_link2 = array(zeros(6))
        self.v_link2 = self.v_0
        self.p_ddot_link2 = array(zeros(6))
        self.p_dot_link2 = array(zeros(6))
        self.p_link2 = [self.p[0]-self.link_length/2*(cos(self.p[5])+cos(self.p[5]+self.joint_angle[1])),
                        self.p[1]-self.link_length/2*(sin(self.p[5])+sin(self.p[5]+self.joint_angle[1])),
                        0,0,0,
                        self.p[5]+self.joint_angle[1]]

        # Inertia Tensor. Principal moments of inertia, and products of inertia [kg*m*m]
        Ixx = self.tensor[0]
        Ixy = self.tensor[1]
        Ixz = self.tensor[2]
        Iyx = self.tensor[3]
        Iyy = self.tensor[4]
        Iyz = self.tensor[5]
        Izx = self.tensor[6]
        Izy = self.tensor[7]
        Izz = self.tensor[8]
        m = self.mass
        xg = self.gravity_center[0]
        yg = self.gravity_center[1]
        zg = self.gravity_center[2]

        Mrb = rospy.get_param(self.vehicle_name + "/dynamics" + "/Mrb")
        Mrb = array(Mrb).reshape(6, 6)

        # Inertia matrix of the rigid body
        # Added Mass derivative
        Ma = rospy.get_param(self.vehicle_name + "/dynamics" + "/Ma")
        Ma = array(Ma).reshape(6, 6)

        self.M = Mrb - Ma    # mass matrix: Mrb + Ma
        self.IM = matrix(self.M).I
#        rospy.loginfo("Inverse Mass Matrix: \n%s", str(self.IM))

        #Init currents
        random.seed()
        self.e_vc = self.current_mean
	#The number of zeros will depend on the number of actuators
        self.u = array(zeros(self.num_actuators)) # Initial thrusters setpoint

    	#Publish pose to UWSim
        rospy.Timer(rospy.Duration(self.uwsim_period), self.pubPose)
        rospy.Timer(rospy.Duration(self.uwsim_period), self.pubScrew)
        rospy.Timer(rospy.Duration(self.uwsim_period), self.pubDatanav)

    #   Create Subscribers for thrusters and collisions
	#TODO: set the topic names as parameters
        rospy.Subscriber(self.input_thruster_topic, Float64MultiArray, self.updateThrusters)
        rospy.Subscriber(self.external_force_topic, WrenchStamped, self.updateCollision)
        # rospy.Subscriber("hardware_command", HardwareCommand, callback)

	s = rospy.Service('/dynamics/reset',Empty, self.reset)

    def iterate(self):
        t1 = rospy.Time.now()

        # Main loop operations
        temp_accel = self.inverseDynamic()
        self.v_dot = temp_accel[0]
        self.v = self.integral(self.v_dot, self.v, self.period)
        self.p_dot = self.kinematics(self.v, self.p[5])
        self.p = self.integral(self.p_dot, self.p, self.period)

        self.p_link2[5] = self.p[5]+self.joint_angle[1]
        self.v_dot_link2 = temp_accel[1]
        self.v_link2 = self.integral(self.v_dot_link2, self.v_link2, self.period)
        self.p_link2[0] = self.p[0]-self.link_length/2*(cos(self.p[5])+cos(self.p_link2[5]))
        self.p_link2[1] = self.p[1]-self.link_length/2*(sin(self.p[5])+sin(self.p_link2[5]))
        # temp = (self.v[5]+self.v_link2[5])/2
        # self.v[5] = temp - self. joint_angular_vel[1]
        # self.v_link2[5] = temp + self. joint_angular_vel[1]

        # self.p_dot_link2[5] = self.p_dot[5]
        # self.p_dot_link2[0] = self.p_dot[0]+self.link_length/2*(sin(self.p[5])*self.p_dot[5]+sin(self.p_link2[5])*self.p_dot_link2[2])
        # self.p_dot_link2[1] = self.p_dot[1]-self.link_length/2*(cos(self.p[5])*self.p_dot[5]+cos(self.p_link2[5])*self.p_dot_link2[2])

        # self.p_ddot = dot(self.transMatrixB2I(self.p[5]),self.v_dot)
        # self.p_ddot_link2[5] = self.p_ddot[5]
        # temp1 = cos(self.p[5])*self.p_dot[5]+cos(self.p_link2[5])*self.p_dot_link2[5]+sin(self.p[5])*self.p_ddot[5]+sin(self.p_link2[5])*self.p_ddot_link2[5]
        # temp2 = sin(self.p[5])*self.p_dot[5]+sin(self.p_link2[5])*self.p_dot_link2[5]-cos(self.p[5])*self.p_ddot[5]-cos(self.p_link2[5])*self.p_ddot_link2[5]
        # self.p_ddot_link2[0] = self.p_ddot[0]+self.link_length/2*(temp1)
        # self.p_ddot_link2[1] = self.p_ddot[1]+self.link_length/2*(temp2)

        # self.v_link2 = dot(self.transMatrixI2B(self.p_link2[5]),self.p_dot_link2)
        # self.v_dot_link2 = dot(self.transMatrixI2B(self.p_link2[5]),self.p_ddot_link2)

        # self.v_link2 = dot(self.transMatrixI2B(self.joint_angle[1]),self.v)
        # self.v_dot_link21 = dot(self.transMatrixB2I(self.joint_angle[1]),self.v_dot)

        t2 = rospy.Time.now()
        p = self.period - (t2-t1).to_sec()
        if p < 0.0 : p = 0.0
        rospy.sleep(p)


if __name__ == '__main__':
    try:
        dynamics = Dynamics()
        while not rospy.is_shutdown():
            dynamics.iterate()

    except rospy.ROSInterruptException: pass
