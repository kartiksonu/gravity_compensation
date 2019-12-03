#!/usr/bin/env python

import rospy
import sys
# Due to rospy we are unable to load directly, hence a hack
try:
	sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')	
	import rospy
	
except Exception as e:
	print(e)
	pass
	
import std_msgs.msg
from ambf_msgs.msg import ObjectState, ObjectCmd
# from kuka7DOF_Spatial_Transform_case import *
from InverseDynamicsModule import *
from geometry_msgs.msg import Vector3

# our global variables

state_msg = ObjectState()
active = True
pos = []

# ROS Subscriber callback function
def get_joint_values(data):
	global state_msg, active, pos
	pos = data.joint_positions
	pos=np.asarray(pos)
	# print(pos)
	# print "state: ", currentState
	return pos



def main():
	sub = rospy.Subscriber('/ambf/env/base/State', ObjectState, get_joint_values, queue_size=1)
	pub = rospy.Publisher('/ambf/env/base/Command', ObjectCmd, queue_size=1)
	rospy.init_node('gravity_compensation_controller')
	rate = rospy.Rate(1000)   #1000hz


	cmd_msg = ObjectCmd()
	cmd_msg.enable_position_controller = False
	cmd_msg.position_controller_mask = [False]

	while not rospy.is_shutdown():


		#current pose
		q = pos         		# print('current pos:', q[1]*3.14/180)
		q_dot = np.zeros(7)
		q_ddot = np.zeros(7)   	


		# G = get_G(q)
		G = Inverse_dynamics_calc_func(q, q_dot, q_ddot)
		G = [-4.61852778e-17, 8.37889941e+00, -3.12902711e+00, 1.13688986e+01, 7.97401873e-03, 3.72971015e-01, -4.57673510e-02]
		print('tau', G)
			
		#define header
		Header = std_msgs.msg.Header()
		Header.stamp = rospy.Time.now()
		cmd_msg.header = Header
		# cmd_msg.joint_cmds = [ tau[0], tau[1],tau[2],tau[3],tau[4],tau[5],tau[6]]#,11.65* 0, 0, 0, 0, 0]
		cmd_msg.joint_cmds = G

		# # print(" torque is ", cmd_msg.joint_cmds)

		pub.publish(cmd_msg)
		rate.sleep()
	# rospy.spin()
	

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass































