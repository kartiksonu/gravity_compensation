
#!/usr/bin/env python

import std_msgs.msg
import yaml
import numpy as np
import mathutils
import math
import rbdl
import sys


## Loading the file
def yaml_loader(filepath):
    """ Loads a file path """
    with open(filepath, 'r') as file_descriptor:
        data = yaml.load(file_descriptor, Loader=yaml.FullLoader)
    return data
## Dumping it
def yaml_dumper(filepath):
    """ Dumps data to a yaml file"""
    with open (filepath, 'w') as file_descriptor:
        yaml.dump(data, file_descriptor)

file_path = "/home/sonu/KUKA_7Dof/blender-kuka7dof.yaml"
data = yaml_loader(file_path)

# Bodies and Joints count
Bodies = []
Joints = []


# Function for couting number of bodies in robot
def Bodies_count(data):
    bodies = data.get('bodies')

    for ele in bodies:
        Bodies.append(ele)

    num_of_bodies = len(bodies)

    # return num_of_bodies, Bodies
    return num_of_bodies, Bodies

# Function for couting number of joints in robot
def Joint_count(data):
    joints = data.get('joints')
    for ele in joints:
        Joints.append(ele)

    num_of_joints = len(joints)

    return num_of_joints, Joints

# Joint and Bodies count testing
n_j,j = Joint_count(data)
n_b,b = Bodies_count(data)

# Function to read mass values
def get_mass_array(data,Bodies):
    mass_arr=[1]
    for body in Bodies:
        mass_arr.append(data[body]['mass'])
    mass_arr.pop(1)
    return np.array(mass_arr).reshape(len(mass_arr),-1)


# Function to read inertia values
def get_inertia_values(data, Bodies):

    inn_val_arr = []
    for bdy in Bodies:
        inn_val_temp=[]
        inn_val_temp.append(data[bdy]['inertia']['ix'])
        inn_val_temp.append(data[bdy]['inertia']['iy'])
        inn_val_temp.append(data[bdy]['inertia']['iz'])
        inn_val_arr.append(inn_val_temp)

    return np.array(inn_val_arr)

# Function to get the position of centre of mass values
def get_inertial_offset(data, Bodies):

    inn_off_arr = []
    for bdy in Bodies:
        inn_off_temp=[]
        inn_off_temp.append(data[bdy]['inertial offset']['position']['x'])
        inn_off_temp.append(data[bdy]['inertial offset']['position']['y'])
        inn_off_temp.append(data[bdy]['inertial offset']['position']['z'])
        inn_off_arr.append(inn_off_temp)

    return np.array(inn_off_arr)


# Function to get type of joint
def get_joint_type(data, Joints):
    J_type = []
    for Joint in Joints:
        J_temp = []
        J_temp.append(data[Joint]['type'])
        J_type.append(J_temp)

    return np.array(J_type)


# Function to get the distance vector between two adjacent bodies
def get_parent_pivot(data, Joints):
	pivot_type = [[0,0,0]]

	for Joint in Joints:
		pivot_temp_type=[]
		pivot_temp_type.append(data[Joint]['parent pivot']['x'])
		pivot_temp_type.append(data[Joint]['parent pivot']['y'])
		pivot_temp_type.append(data[Joint]['parent pivot']['z'])
		pivot_type.append(pivot_temp_type)
	return np.array(pivot_type)


# Function to get type of joint
def get_child_axes(data, Joints):
    child_axis = []
    for Joint in Joints:
        child_axis_temp = []
        child_axis_temp.append(data[Joint]['child axis'])
        child_axis.append(child_axis_temp)

    return np.array(child_axis)

# Function to get type of joint
def get_parent_axes(data, Joints):
    parent_axis = []
    for Joint in Joints:
        parent_axis_temp = []
        parent_axis_temp.append(data[Joint]['parent axis'])
        parent_axis.append(parent_axis_temp)

    return np.array(parent_axis)

def dicttoVec(dict):
	x = dict['x']
	y = dict['y']
	z = dict['z']
	vec = [x, y, z]

	return np.array(vec)

# https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/897677#897677

def skew_mat(v):
    m = mathutils.Matrix.Identity(3)
    m.Identity(3)
    m[0][0] = 0
    m[0][1] = -v.z
    m[0][2] = v.y
    m[1][0] = v.z
    m[1][1] = 0
    m[1][2] = -v.x
    m[2][0] = -v.y
    m[2][1] = v.x
    m[2][2] = 0

    return m


def vec_norm(v):
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)


def round_vec(v):
    for i in range(0, 3):
        v[i] = round(v[i], 3)
    return v

def rot_matrix_from_vecs(v1, v2):
    out = mathutils.Matrix.Identity(3)
    vcross = v1.cross(v2)
    vdot = v1.dot(v2)
    rot_angle = v1.angle(v2)
    if 1.0 - vdot < 0.1:
        return out
    elif 1.0 + vdot < 0.1:
        # This is a more involved case, find out the orthogonal vector to vecA
        nx = mathutils.Vector([1, 0, 0])
        temp_ang = v1.angle(nx)
        if 0.1 < abs(temp_ang) < 3.13:
            axis = v1.cross(nx)
            out = out.Rotation(rot_angle, 3, axis)
        else:
            ny = mathutils.Vector([0, 1, 0])
            axis = v1.cross(ny)
            out = out.Rotation(rot_angle, 3, axis)
    else:
        skew_v = skew_mat(vcross)
        out = mathutils.Matrix.Identity(3) + skew_v + skew_v * skew_v * ((1 - vdot) / (vec_norm(vcross) ** 2))
    return out


def MatToNpArray(matrix):

	col0 = [matrix[0][0], matrix[1][0], matrix[2][0]]
	col1 = [matrix[0][1], matrix[1][1], matrix[2][1]]
	col2 = [matrix[0][2], matrix[1][2], matrix[2][2]]

	rot_matrix = [ col0, col1, col2 ]

	return np.array(rot_matrix)


def YamlToRBDLmodel(data, Bodies, Joints):


	file_path = "/home/sonu/KUKA_7Dof/blender-kuka7dof.yaml"
	data = yaml_loader(file_path)

	model = rbdl.Model()
	model.gravity=[0,0,-9.81] 

	no_of_bodies, random_var = Bodies_count(data)

	mass = get_mass_array(data, Bodies)
	mass_arr = np.array([[1],[1],[1],[1],[1],[1],[1],[1]])#good

	com_val = get_inertial_offset(data, Bodies)
	# print("COM val is\n", com_val)

	inertia_val = get_inertia_values(data, Bodies)
	# print(inertia_val)

	parent_dist = get_parent_pivot(data, Joints)
	# print(r_val)

	J_type = get_joint_type(data, Joints)

	joint_rot_z = rbdl.Joint.fromJointType ("JointTypeRevoluteZ")
	joint_fixed= rbdl.Joint.fromJointType ("JointTypeFixed")


	# Parsing child and parent axes to create the transformation matrix between two adjacent bodies
	child_axes  = get_child_axes(data, Joints)
	parent_axes = get_parent_axes(data, Joints)

	
	for i in range(0, no_of_bodies):

		# Creating of the transformation matrix between two adjacent bodies
		trans = rbdl.SpatialTransform()
		print("\nParameters for body", i)

		if i == 0 :
			trans.E = np.eye(3);
			# print("R is\n", trans.E);
			trans.r = [0.0, 0.0, 0.0];
			# print("T is\n", trans.r);
			print("T is\n", trans);
		else:
			# get parent and child axes of the pair
			child_axis = dicttoVec(child_axes[i-1][0])
			parent_axis = dicttoVec(parent_axes[i-1][0])

			# Find the rotation matrix between child and parent
			r_mat = rot_matrix_from_vecs(mathutils.Vector(child_axis), mathutils.Vector(parent_axis))
			r_mat_np = MatToNpArray(r_mat)
			# print("R is", r_mat_np)
			trans.E = r_mat_np
			# print("R is\n", trans.E);
			trans.r = parent_dist[i];
			# print("T is\n", trans.r);
			print("T is\n", trans);
		
		# Creating Inertia matrix with just principle Inertia values
		I_x = inertia_val[i][0]
		I_y = inertia_val[i][1]
		I_z = inertia_val[i][2]
		inertia_matrix = np.array([[I_x, 0, 0], [0, I_y, 0], [0, 0, I_z]])
		print("\nInertia matrix is\n", inertia_matrix)

		print("\nCOM is\n", com_val[i])

		print("\nMass of the body is\n", mass[i])


		# Creating each body of the robot
		body = rbdl.Body.fromMassComInertia(mass[i], com_val[i], inertia_matrix)

		# Specifying joint Type
		if i == 0:
			joint_type = joint_fixed
		else:
			joint_type = joint_rot_z
		# joint_type = rbdl.Joint.fromJointType(joint_name[i][0])
		print("joint type is", joint_type);


		# Adding body to the model to create the complete robot
		model.AppendBody(trans, joint_type, body)
		# print(i)

	return model


def Inverse_dynamics_calc_func(q_val, qdot_val, qddot_val):

	model = YamlToRBDLmodel(data, Bodies, Joints)
	no_of_bodies, random_num = Bodies_count(data)

	q_ = np.asarray(q_val)
	q = np.zeros(7)
	q[0]=q_[0]
	q[1]=q_[1]
	q[2]=q_[2]
	q[3]=q_[3]
	q[4]=q_[4]
	q[5]=q_[5]
	q[6]=q_[6]

	qdot  = np.zeros(7)
	qddot = np.zeros(7)

	tau   = np.zeros(model.qdot_size)

	# RBDL inverse dynamics function
	rbdl.InverseDynamics(model, q, qdot, qddot, tau)

	return tau












