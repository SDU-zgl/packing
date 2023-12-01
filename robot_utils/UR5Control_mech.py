#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Real UR5 robot control script
'''
import time
import matplotlib.pyplot as plt
import cv2



# from utils.fit_mini_rectangle import detect_center_points_from_mask
# from utils.image2robot import image2robot_mech
import urx
import numpy as np
import os
import socket
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

'''
初始化传参说明：(tcp_host_ip,vel=0.05,acc=0.3,gripper_control='urx',camera_type='kinect',workspace_limits=None)
- vel: 速度，可不传参，默认值为0.05
- acc: 加速度，可不传参，默认值为0.3
- gripper_control: 爪子控制方式，可通过urx/ros进行控制，可不传参，默认为urx方式
- camera_type: 相机类型，可使用kinect/realsense，可不传参，默认使用kinect
- workspace_limits: 机械臂工作空间范围，如不传值，则使用默认的
'''
class UR5Robot(object):
    def __init__(self,tcp_host_ip,vel=0.05,acc=0.3,gripper_control='urx',camera_type='kinect',workspace_limits=None):
        '''
        self.tool_offset_z = 0#-0.013#-0.266 #suction,gripper=-0.09~-0.13#ought to revise rob.set_tcp
        self.rob.set_tcp((0, 0, 0, 0, 0, 0)) #set robot flange to tool tip transformation
        self.rob.set_payload(0.5, (0, 0, 0)) #set payload in Kg
        保持机械臂原来tcp设置不变，应用时，根据实际的夹具设置，相应设置z方向的offset，将累加偏移的实际结果发给机械臂运行
        '''
        ############## kinect&realsense使用设置 ###############
        kinect_intrinsic_source = 2#1 # 1-calib_color.yaml ; 2-kinect_camera_intr_params.txt
        realsense_control = 1 # 1-use tcp control ; 2-use pyrealsense
        ######################################################
        if workspace_limits is None:
            self.workspace_limits = np.asarray([[0.226, 0.616], [-0.256, 0.184], [0.358, 0.408]]) # you can modify the limits according to your own application
        else:
            self.workspace_limits = workspace_limits
        self.rob = urx.Robot(tcp_host_ip)
        print('Connect ur5 successful, current pos:',self.rob.getl())
        self.l = 0.05
        self.vel = vel
        self.acc = acc
        self.rad = 0.01
        self.gripper_control = gripper_control
        self.camera_type = camera_type
        if gripper_control == 'ros' or camera_type=='kinect':
            rospy.init_node('calibration')
        # Init gripper #
        if gripper_control == 'ros':
            print("Make sure launch gripper via ROS...")
            from robotiq_close import Robotiq85GripperTestClose as robotiq_close
        else:
            self.robotiqgrip = Robotiq_Two_Finger_Gripper(self.rob)

        # Init Camera ##
        if camera_type=='kinect':
            '''
            Kinect通过ros来控制和获取数据。使用kinect前需要先进行标定校准，Kinect的内参可以通过两种方式提供：
            - 1 calib_color.yaml 对Kinect进行手动标定，生产的内参文件
            - 2 kinect_camera_intr_params.txt 使用pylibkinect库调用kinect，自动生成的内参文件
            '''
            self.kinect_bridge = CvBridge()
            if kinect_intrinsic_source == 1:
                CameraIntrinsicData = parseYamlFile('/home/arm/project/real_fast_picking_v2/real_world_v1/real/calibration/calib_color.yaml')
                self.cam_intrinsics = CameraIntrinsicData
            elif kinect_intrinsic_source == 2:
                self.cam_intrinsics = np.loadtxt("kinect_camera_params.txt")#/home/arm/project/real_fast_picking_v2/calibration_v2/kinect_camera_params.txt
                pass
            else:
                raise ValueError('kinect_intrinsic_source is invalid')
        elif camera_type == 'realsense':
            '''
            realsense可通过tcp通信或者pyrealsense库来控制和获取数据。
            '''
            # ----- for realsense init ------- #
            print("Replugn the usb! if change method between tcp and pyrs")
            if realsense_control == 1:
                self.realsense_cam = RealsenseCamera_tcp()
            elif realsense_control == 2:
                self.realsense_cam = RealsenseCamera_pyrs()
            else:
                raise ValueError('realsense_control is invalid')
            self.cam_intrinsics = np.loadtxt("D435_intrinsics.txt")#/home/arm/project/real_fast_picking_v2/calibration_v2/kinect_camera_params.txt
        elif camera_type == 'mech':
            '''
            mech相机初始化
            '''
            # ----- for MechMind camera init ------- #
            #import ipdb; ipdb.set_trace()
            pass
            #self.mech_cam =MechMindCam()
            #相机内参暂时未知
            #self.cam_intrinsics = #





    def wait_for_execution_finish(self):
        while True:
            pose1 = self.get_robot_state()
            time.sleep(0.5)
            pose2 = self.get_robot_state()
            move_distance = np.linalg.norm(np.array(pose1[0:3])-np.array(pose2[0:3]))
            if move_distance < 0.002:
                break
            else:
                pass

    def safe_check(self, pos):
        return True
        x_range_check = (pos[0] >= self.workspace_limits[0][0] and pos[0] <= self.workspace_limits[0][1])
        y_range_check = (pos[1] >= self.workspace_limits[1][0] and pos[1] <= self.workspace_limits[1][1])
        z_range_check = (pos[2] >= self.workspace_limits[2][0] and pos[2] <= self.workspace_limits[2][1])
        if not x_range_check:
            print('Dangerous action: ', pos, ', not in the spacelimit along X')
        if not y_range_check:
            print('Dangerous action: ', pos, ', not in the spacelimit along Y')
        if not z_range_check:
            print('Dangerous action: ', pos, ', not in the spacelimit along Z')
        return (x_range_check and y_range_check and z_range_check)

    def get_robot_state(self):
        '''
        get current robot's position
        :return: [x,y,z,rx,ry,rz]
        '''
        current_pos = self.rob.getl()
        # print("Current Robot Position is:", current_pos)
        return current_pos

    def get_robot_joint_state(self):
        '''
        get current robot's joint position
        :return: [ang1,ang2,ang3,ang4,ang5,ang6]
        '''
        current_pos = self.rob.getj()
        # print("Current Robot Position is:", current_pos)
        return current_pos

    def move_to_position(self,robot_position,robot_orientation,enable_stop=None,acc=None,vel=None):
        '''
        robot move to given positon [x,y,z,rx,ry,rz]
        input paras:
        wait:wait for next data packet from robot(0.5s)
        relative:if move start from current pos
        :return:
        '''
        if True:#self.safe_check(robot_position):
            if acc == None:
                acc = self.acc
                vel = self.vel
            pose = [robot_position[0],robot_position[1],robot_position[2],robot_orientation[0],robot_orientation[1],robot_orientation[2]]
            self.rob.movel(pose, acc=acc, vel=vel, wait=False, relative=False)
            if enable_stop is None or enable_stop == False:
                self.wait_for_execution_finish()
        else:
            print('Action execution cancel!!!')
    
    def move_to_position_with_fixed_rotation(self,robot_position,enable_stop=None,acc=None,vel=None):
        '''
        robot move to given positon with fixed orientation [x,y,z]
        input paras:
        wait:wait for next data packet from robot(0.5s)
        relative:if move start from current pos
        :return:
        '''
        if self.safe_check(robot_position):
            if acc == None:
                acc = self.acc
                vel = self.vel
            current_pos = self.rob.getl()
            pose = [robot_position[0],robot_position[1],robot_position[2],current_pos[3],current_pos[4],current_pos[5]]
            self.rob.movel(pose, acc=acc, vel=vel, wait=False, relative=False)
            if enable_stop is None or enable_stop == False:
                self.wait_for_execution_finish()
        else:
            print('Action execution cancel!!!')

    def move_to_position_with_angle(self,robot_position,rotation,acc=None,vel=None):
        '''
        robot move to given positon [x,y,z] + roattion
        :return:
        '''
        current_pose = self.rob.get_pose()
        # base_pose = [current_pose[0],current_pose[1],current_pose[2],2.22, -2.22, 0]
        current_pose.orient.rotate_zb(rotation)#pi/8
        current_pose.pos.x = robot_position[0]
        current_pose.pos.y = robot_position[1]
        current_pose.pos.z = robot_position[2]
        self.rob.set_pose(current_pose,vel=0.2,acc=0.1)
        # print("start move")
        self.wait_for_execution_finish()
        # print("finish move")
        # print('current pos:',self.rob.getl())

    def set_joint_pose(self,six_joint_pose,acc=None,vel=None):
        '''
        set angles for six joints
        :param six_joint_pose:
        :return:
        '''
        if acc == None:
            acc = self.acc
            vel = self.vel
        six_joint_pose = np.array(six_joint_pose)
        six_joint_pose = six_joint_pose/180.0*np.pi
        self.rob.movej(six_joint_pose,acc=acc,vel=vel)

    def move_by_increment(self,robot_position_increment,robot_orientation_increment):
        '''
        robot move by given increment [i_x,i_y,i_z,i_rx,i_ry,i_rz]
        input paras:
        wait:vwait for next data packet from robot(0.5s)
        relative:if move start from current pos
        :return:
        '''
        pose = [robot_position_increment[0],robot_position_increment[1],robot_position_increment[2],robot_orientation_increment[0],robot_orientation_increment[1],robot_orientation_increment[2]]
        self.rob.movel(pose, acc=self.acc, vel=self.vel, wait=False, relative=True)

    def rotate_gripper(self,ang):
        '''
        relative rotate
        :param ang:
        :return:
        '''
        current_pose = self.rob.get_pose()
        current_pose.orient.rotate_zb(ang)#pi/8
        self.rob.set_pose(current_pose,vel=self.vel,acc=self.acc)

    def close_gripper(self):
        '''
        use ros robotiq gripper control
        :return: close
        '''
        if self.gripper_control == 'ros':
            robotiq_close(False, True)
            time.sleep(1.5)
            # return self.check_grasp_status()
        else:
            self.robotiqgrip.close_gripper()

    def open_gripper(self):
        '''
        use ros robotiq gripper control
        :return: open
        '''
        if self.gripper_control == 'ros':
            robotiq_close(True, False)
            time.sleep(1.5)
            # return True
        else:
            self.robotiqgrip.open_gripper()

    def close_suction_gripper(self):
        '''
        suction gripper control
        :return: close
        '''
        self.rob.set_digital_out(0,False)

    def open_suction_gripper(self):
        '''
        suction gripper control
        :return: open
        '''
        self.rob.set_digital_out(0,True)

    def close_suction4_gripper(self):
        '''
        suction gripper control
        :return: close
        '''
        self.rob.set_digital_out(1,False)

    def open_suction4_gripper(self):
        '''
        suction gripper control
        :return: open
        '''
        self.rob.set_digital_out(1,True)

    ############## juck, not use ############
    # def get_mechmind_camera_data(self):
    #     # capture depth image and color image and save them
    #     depth = self.camera.captureDepthImg()
    #     color = self.camera.captureColorImg()
    #
    #     if len(depth) == 0 or len(color) == 0:
    #         exit(-2)
    #
    #     return color,depth

    def get_mech_sensor_data(self):
        '''
        得到梅卡相机的深度图和rgb图像
        :return:
        '''
        color_img, depth_img = self.mech_cam.get_camera_data()
        return color_img, depth_img

    def get_kinect_camera_data_rgb(self):
        '''
        get color data from kinect sensor
        :return 1920*1080 RGB image
        '''
        try:
            rgb_img = rospy.wait_for_message("/kinect2/hd/image_color", Image,timeout=20)
        except:
            import ipdb;ipdb.set_trace()
            rgb_img = rospy.wait_for_message("/kinect2/hd/image_color", Image,timeout=20)
        cv2_img = self.kinect_bridge.imgmsg_to_cv2(rgb_img, "bgr8")

        return cv2_img

    def get_kinect_camera_data_rgb_qhd(self):
        '''
        get color data from kinect sensor
        :return 1280*720 image
        '''
        rgb_img = rospy.wait_for_message("/kinect2/qhd/image_color", Image)
        cv2_img = self.kinect_bridge.imgmsg_to_cv2(rgb_img, "bgr8")
        return cv2_img

    def get_kinect_camera_data_depth(self):
        '''
        get depth data from kinect sensor
        :return 1920*1080 depth image
        '''
        try:
            depth = rospy.wait_for_message("/kinect2/hd/image_depth_rect", Image,timeout=20)
        except:
            import ipdb;ipdb.set_trace()
            depth = rospy.wait_for_message("/kinect2/hd/image_depth_rect", Image,timeout=20)
        cv2_depth = self.kinect_bridge.imgmsg_to_cv2(depth, "passthrough")
        return cv2_depth

    def get_kinect_camera_data_depth_qhd(self):
        '''
        get depth data from kinect sensor
        :return:
        '''
        depth = rospy.wait_for_message("/kinect2/qhd/image_depth_rect", Image)
        cv2_depth = self.kinect_bridge.imgmsg_to_cv2(depth, "passthrough")
        return cv2_depth

    def get_realsense_sensor_data(self):
        '''
        :return:
        '''
        color_img, depth_img = self.realsense_cam.get_data()
        return color_img, depth_img
    
    

    def get_force_sensor_data(self):
        '''
        get the force data along Z-Axis
        :return:
        '''
        force_topic = "/robotiq_force_torque_sensor"
        force_state = rospy.wait_for_message(force_topic, ft_sensor)
        state = str(force_state)
        locate = state.find("Fz:")
        locate_end = state.find("Mx:")
        return float(state[locate + 4:locate_end])

    def check_grasp_status(self):
        '''
        check if the gripper grasps successfully
        :return: True-Success, False-Fail
        '''
        gripper_still_moving = True
        while gripper_still_moving:
            irq_topic = "/iqr_gripper/stat"
            irq_state = rospy.wait_for_message(irq_topic, GripperStat)
            state = str(irq_state)
            all_lines = state.split('\n')
            move_stau = all_lines[8].split(': ')[1]
            gripper_still_moving = (move_stau=='True')
            locate = state.find("position")
            locate_end = state.find("requested_posi")
            margin = float(state[locate + 10:locate_end])
            time.sleep(0.2)

        return margin > 0.005


    #--------------------------- new functions ---------------------------#
    '''
    movej, movel, movep, movec, |movejs, movels
    '''

    def move_to_position_linear(self,robot_position,robot_orientation,acc=None,vel=None):
        '''
        robot move to a given pose [x,y,z,rx,ry,rz]
        input paras: the position and orientation
        '''
        if acc == None:
            acc = self.acc
            vel = self.vel
        pose = [robot_position[0],robot_position[1],robot_position[2],robot_orientation[0],robot_orientation[1],robot_orientation[2]]
        self.rob.movel(pose, acc=acc, vel=vel, wait=True, relative=False)

    # def move_to_position_circle(self,robot_position,robot_orientation,acc=None,vel=None,rad=0.01):
    #     '''
    #     robot move to a given pose [x,y,z,rx,ry,rz]
    #     input paras: the position and orientation
    #     '''
    #     if acc == None:
    #         acc = self.acc
    #         vel = self.vel
    #     pose = [robot_position[0],robot_position[1],robot_position[2],robot_orientation[0],robot_orientation[1],robot_orientation[2]]
    #     self.rob.movep(pose, acc=acc, vel=vel, radius=rad, wait=True)

    def move_through_positions_with_arc(self, position_list, rad=0.01, acc=None, vel=None):
        '''
        Description: given several points, robot moves through these points with a circular trajectory
        Params: position_list: [p1, p2, p3...], p1=[x,y,z,rx,ry,rz]
                radius: the radius for planning the circular trajectory, e.g. 0.01
        '''
        if acc == None:
            acc = self.acc
            vel = self.vel
        self.rob.movels(position_list, vel=vel, acc=acc, radius=rad, wait=True, threshold=None)
        # self.wait_for_execution_finish()

    def move_to_B_via_A_with_arc(self, pose_A, pose_B, acc=None, vel=None, wait=True, threshold=None):
        '''
        Description: move from current pose to pose_A via pose B with a circular trajectory
        '''
        if acc == None:
            acc = self.acc
            vel = self.vel
        self.rob.movec(pose_A, pose_B, vel=vel, acc=acc, wait=True)

    def move_along_x(self, distance, acc=None, vel=None):
        if acc == None:
            acc = self.acc
            vel = self.vel
        vect = [distance, 0, 0]
        self.rob.translate(vect, vel=vel, acc=acc, wait=True, command='movel')

    def move_along_y(self, distance, acc=None, vel=None):
        if acc == None:
            acc = self.acc
            vel = self.vel
        vect = [0, distance, 0]
        self.rob.translate(vect, vel=vel, acc=acc, wait=True, command='movel')

    def move_along_z(self, distance, acc=None, vel=None):
        if acc == None:
            acc = self.acc
            vel = self.vel
        self.rob.up(distance, vel=vel, acc=acc, wait=True)
    
    '''
    def get_tcp_force(self, wait=True):
        """
        return measured force in TCP
        if wait==True, waits for next packet before returning
        """
        return self.rtmon.getTCFForce(wait)

    def get_force(self, wait=True):
        """
        length of force vector returned by get_tcp_force
        if wait==True, waits for next packet before returning
        """
        tcpf = self.get_tcp_force(wait)
        force = 0
        for i in tcpf:
            force += i**2
        return force**0.5
    '''
    def get_force_from_sensor(self):
        force = self.rob.get_tcp_force()
        force2 = self.rob.get_force()
        print('force: ', force)
        print('force2: ', force2)
        return force
    



    """ def detect_center_and_angle(self, depth, rgb, item):
        #首先进行roi裁剪，裁剪完成之后判断深度的特征 820数值需要修改
        mask = np.zeros_like(depth)
        #import ipdb; ipdb.set_trace()
        #这部分实际应用的时候需要查看是否检测成功,显示暂时是每次需要手动关闭一下
        mask[180:1080, 320:1366] = (depth[180:1080, 320:1366] < 1240) * (depth[180:1080, 320:1366] >20) * 1
        mask = np.uint8(mask)
        # plt.imshow(mask)
        # plt.savefig('mask.png')
        center, rotation, box, (w, h) = detect_center_points_from_mask(mask)
        rotation = rotation[0]
        center = center[0]#这里感觉不合理，需要在检测方形函数返回时给去掉
        box = np.int0(box)
        img = rgb
        cv2.circle(img, center, 10, (0,0,255), -1)
        cv2.line(img, box[0], box[1], (0,255,0), 5)
        cv2.line(img, box[1], box[2], (0,255,0), 5)
        cv2.line(img, box[2], box[3], (0,255,0), 5)
        cv2.line(img, box[3], box[0], (0,255,0), 5)
        
        cv2.imshow('img',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        suc_position = image2robot_mech(center, depth) # (x,y,z)
        print(rotation)
        #rotation = rotation[0]/180*np.pi
        #import ipdb; ipdb.set_trace()
        #这里是否加一个和之前的判断类似算两点的距离比值，看是否和预估的来的物体类似
        print(abs(w/float(item.width) - h/float(item.height)),  w/float(item.height) - h/float(item.width))
        if item.rotation_type == 0:
            if float(item.width) >= float(item.height):
                if w >= h :
                    #print(abs(w/float(item.width) - h/float(item.height)),  w/float(item.height) - h/float(item.width))
                    rotation = abs(rotation)#
                if w < h :
                    rotation = -(90 - abs(rotation))
            if float(item.width) < float(item.height):
                if w <= h :
                    #print(abs(w/float(item.width) - h/float(item.height)),  w/float(item.height) - h/float(item.width))
                    rotation = abs(rotation)#
                if w > h :
                    rotation = -(90 - abs(rotation))

        elif item.rotation_type == 1:
            if float(item.width) >= float(item.height):
                if w >= h :
                    #print(abs(w/float(item.width) - h/float(item.height)),  w/float(item.height) - h/float(item.width))
                    rotation = -(90 - abs(rotation))
                if w < h :
                    rotation = abs(rotation)
            if float(item.width) < float(item.height):
                if w <= h :
                    #print(abs(w/float(item.width) - h/float(item.height)),  w/float(item.height) - h/float(item.width))
                    rotation = -(90 - abs(rotation))
                if w > h :
                    rotation = abs(rotation)


        #import ipdb; ipdb.set_trace()

        # if rotation_flag == 1:
        #     if width >= length :                #### 逆时针 |rotation|
        #         rotation = abs(rotation)
        #     if width < length :                 #### 顺时针 90 - |rotation|
        #         rotation = -(90 - abs(rotation))
        # elif rotation_flag == 2:
        #     if width >= length :                #### 顺时针 90 - |rotation|
        #         rotation = -(90 - abs(rotation))
        #     if width < length :                 #### 逆时针 |rotation|
        #         rotation = abs(rotation)
        print('suction position:',suc_position)
        #import ipdb;ipdb.set_trace()
        rotation = rotation/180 *np.pi
        self.move_to_position_with_angle([suc_position[0],suc_position[1],suc_position[2]+0.05], -rotation)
        self.open_suction_gripper()
        self.move_to_position_with_fixed_rotation(suc_position)


        self.move_to_position_with_fixed_rotation([suc_position[0],suc_position[1],suc_position[2]+0.01])
        self.move_to_position_with_angle([suc_position[0],suc_position[1],suc_position[2]+0.05], rotation)
 """

