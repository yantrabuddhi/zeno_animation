#!/usr/bin/env python3

import rospy
import bpy
import yaml
#run script from blender


#import actionlib

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryActionGoal

from zeno_animation.srv import *

from mathutils import Matrix

class Utils:
    def read_yaml(self,yamlname):
        stream = open(yamlname, 'r')
        result = yaml.load(stream)
        stream.close()
        return result

    def get_pose_matrix_in_other_space(self, mat, pose_bone):
        """
        Returns the transform matrix relative to pose_bone's current
        transform space. In other words, presuming that mat is in
        armature space, slapping the returned matrix onto pose_bone
        should give it the armature-space transforms of mat.
        TODO: try to handle cases with axis-scaled parents better.
        """
        rest = pose_bone.bone.matrix_local.copy()
        rest_inv = rest.inverted()
        if pose_bone.parent:
            par_mat = pose_bone.parent.matrix.copy()
            par_inv = par_mat.inverted()
            par_rest = pose_bone.parent.bone.matrix_local.copy()
        else:
            par_mat = Matrix()
            par_inv = Matrix()
            par_rest = Matrix()

        # Get matrix in bone's current transform space
        smat = rest_inv * (par_rest * (par_inv * mat))

        return smat

    def get_local_pose_matrix(self,pose_bone):
        """Returns the local transform matrix of the given pose bone."""
        return self.get_pose_matrix_in_other_space(pose_bone.matrix, pose_bone)

    def get_bones_rotation_rad(self,armature, bone, axis):
        mat = self.get_local_pose_matrix(bpy.data.objects[armature].pose.bones[bone])
        return getattr(mat.to_euler(), axis)

class TimelineAnimation:
    """
    This output can build and send an animation out, if you give the
    animation's location (FrameMap instance).
    """
    motor_category={}
    debug_only=True

    def create_new_structure(self):
        self.motor_category={}#dictionary of categories
        for joint_group in self.motors:
            joint_info={}#dictionary of joints
            for joint in joint_group['Joint']:
                joint_info[joint['JointName']]=(joint,[],[])#tuple of joint data
            self.motor_category[joint_group['JointGroupName']]=joint_info#dictionary of joints


    def read_all_motors_curr_frame(self,x,spf):
        for cat in self.motor_category:
            jDict=self.motor_category[cat]
            for joint_name in jDict:
                joint_tuple=jDict[joint_name]
                joint_info=joint_tuple[0]
                joint_arr=joint_tuple[1]
                joint_vel_arr=joint_tuple[2]
                #apply scale and translation
                joint_pos=utils.get_bones_rotation_rad(joint_info['Armature'],joint_info['Bone'],joint_info['Axis'])
                joint_pos=joint_pos*float(joint_info['AngleScale'])+float(joint_info['AngleAdd'])
                joint_arr.append(joint_pos)
                if x==0:
                    joint_vel_arr.append(0.0)
                else:
                    joint_vel_arr.append((joint_pos-joint_arr[x-1])/spf)

    def animate_bl(self, frame_start,frame_stop,secs):
        secs_per_frame=secs/float(abs(frame_stop-frame_start))
        self.create_new_structure()
        #loop through animations
        count=0
        for x in range (frame_start,frame_stop):
            bpy.context.scene.frame_set(frame=x)
            self.read_all_motors_curr_frame(count,secs_per_frame)
            count=count+1

        all_cat_traj=[]
        #joint_category is same as joint controller
        for cat in self.motor_category:
            jDict=self.motor_category[cat]
            traj=JointTrajectory()
            for joint_name in jDict:
                traj.joint_names.append(joint_name)

            count=0
            for x in range(frame_start,frame_stop):
                pos_arr=JointTrajectoryPoint()
                pos_arr.time_from_start=secs

                for joint_name in jDict:
                    joint_tuple=jDict[joint_name]
                    #joint_info=joint_tuple[0]
                    joint_arr=joint_tuple[1]
                    joint_vel_arr=joint_tuple[2]
                    pos_arr.positions.append(joint_arr[count])
                    pos_arr.velocities.append(joint_vel_arr[count])
                    if self.debug_only:print("Frame:",x," Joint:",joint_name," angle:",joint_arr[count]," velocity:",joint_vel_arr[count])

                traj.points.append(pos_arr)
                count=count+1

            all_cat_traj.append((cat,traj))
        #send trajectory for the category
        if self.debug_only: return
        self.send_trajectories(all_cat_traj)

    # def send_trajectories(self, all_trajectories):
    #     """
    #     Will send trajectory messages.
    #     """
    #     client={}
    #     for trajectory in all_trajectories:
    #         cat_topic=trajectory[0]
    #         trajectory_msg=trajectory[1]
    #         client[cat_topic] = actionlib.SimpleActionClient(cat_topic, FollowJointTrajectoryAction)
    #         client[cat_topic].wait_for_server()
    #         goal=FollowJointTrajectoryActionGoal(trajectory_msg)
    #         client[cat_topic].send_goal(goal)
    #     for cat in client:
    #         client[cat].wait_for_result()

    def __init__(self):
        self.motors = utils.read_yaml(blender_joint_file)

class AnimationControl:

    def handle_cmd(self,msg):
        found=False
        for config in self.animation_map:
            found= (msg.command == config['name'])
            if found: break

        if not found:
            print("Animation not found: %s\n",msg.command)
            return AnimateResponse(-1)
        frame_start = int(config["frame_start"])
        frame_stop=int(config["frame_stop"])
        min_secs=float(config["min_secs"])
        print("frame mapped")
        if (frame_start<=0) or (frame_stop<=0):
            print("frame value can't be less than zero")
            return AnimateResponse(-3)
        if frame_start==frame_stop:
            print("No animation range")
            return AnimateResponse(-4)
        if msg.secs==0:
            print("doing fastest time")
            self.animate_bl(frame_start,frame_stop,min_secs)
            return AnimateResponse(1)
        if msg.secs<min_secs:
            print("Duration too low. Minimum duration=%f\n",min_secs)
            return AnimateResponse(-2)
        self.TLA.animate_bl(frame_start,frame_stop,msg.secs)
        return AnimateResponse(0)

    def __init__(self):
        self.animation_map = utils.read_yaml(blender_animation_map)
        self.TLA=TimelineAnimation()
        rospy.init_node('RobotAnimator')
        #self.sub = rospy.Subscriber("AnimationCommand", AnimateCommand, self.handle_cmd)
        self.srvc=rospy.Service("AnimationService",Animate,self.handle_cmd)

#blender_joint_file="zeno_blender_joints.yaml"
#blender_animation_map="zeno_animation_map.yaml"
blender_joint_file="test_blender_joints.yaml"
blender_animation_map="test_animation_map.yaml"
utils=Utils()
animate=AnimationControl()