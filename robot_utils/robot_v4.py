import numpy as np
import vrep
import random
import cv2
import time

class Robot(object):
    def __init__(self,workspace_limits):
        #################################
        self.five_loaded_item_name = []
        self.five_loaded_item_position = [[0.5,-1.5,0],[0,-1.5,0],[-0.5,-1.5,0],[-1,-1.5,0],[-1.5,-1.5,0]] #[[-0.5,-1.5,0],[0,-1.5,0],[-1,-1.5,0],[0.5,-1.5,0],[-1.5,-1.5,0]]
        self.five_loaded_item_height = []
        self.bin_max_height = 0
        self.current_item_ind = -1
        self.current_item_position = [0,0,0]
        self.current_item_name = None
        self.current_item_size = [0,0,0]
        self.all_item_list = ['object1','object2','object3','object4','object5','object6','object7','object8','object9','object10','object11','object12','object13','object14',
                              'object1a','object2a','object3a','object4a','object5a','object6a','object7a','object8a','object9a','object10a','object11a','object12a','object13a','object14a',
                              'object1b','object2b','object3b','object4b','object5b','object6b','object7b','object8b','object9b','object10b','object11b','object12b','object13b','object14b',
                              'object1c','object2c','object3c','object4c','object5c','object6c','object7c','object8c','object9c','object10c','object11c','object12c','object13c','object14c',
                              'object1d','object2d','object3d','object4d','object5d','object6d','object7d','object8d','object9d','object10d','object11d','object12d','object13d','object14d',
                              'object1e','object2e','object3e','object4e','object5e','object6e','object7e','object8e','object9e','object10e','object11e','object12e','object13e','object14e',
                              'object1f','object2f','object3f','object4f','object5f','object6f','object7f','object8f','object9f','object10f','object11f','object12f','object13f','object14f',
                              'object1g','object2g','object3g','object4g','object5g','object6g','object7g','object8g','object9g','object10g','object11g','object12g','object13g','object14g',
                              'object1h','object2h','object3h','object4h','object5h','object6h','object7h','object8h','object9h','object10h','object11h','object12h','object13h','object14h',
                              'object1i','object2i','object3i','object4i','object5i','object6i','object7i','object8i','object9i','object10i','object11i','object12i','object13i','object14i']

        self.all_item_size_dict = {'object1':[35,30,20],'object2':[35,30,30],'object3':[35,35,20],'object4':[35,35,30],'object5':[40,30,20],'object6':[40,30,30],'object7':[40,40,20],'object8':[40,40,30],'object9':[40,40,40],'object10':[50,30,20],'object11':[50,30,30],'object12':[50,40,20],'object13':[50,40,30],'object14':[50,40,40],
                                   'object1a':[35,30,20],'object2a':[35,30,30],'object3a':[35,35,20],'object4a':[35,35,30],'object5a':[40,30,20],'object6a':[40,30,30],'object7a':[40,40,20],'object8a':[40,40,30],'object9a':[40,40,40],'object10a':[50,30,20],'object11a':[50,30,30],'object12a':[50,40,20],'object13a':[50,40,30],'object14a':[50,40,40],
                                   'object1b':[35,30,20],'object2b':[35,30,30],'object3b':[35,35,20],'object4b':[35,35,30],'object5b':[40,30,20],'object6b':[40,30,30],'object7b':[40,40,20],'object8b':[40,40,30],'object9b':[40,40,40],'object10b':[50,30,20],'object11b':[50,30,30],'object12b':[50,40,20],'object13b':[50,40,30],'object14b':[50,40,40],
                                   'object1c':[35,30,20],'object2c':[35,30,30],'object3c':[35,35,20],'object4c':[35,35,30],'object5c':[40,30,20],'object6c':[40,30,30],'object7c':[40,40,20],'object8c':[40,40,30],'object9c':[40,40,40],'object10c':[50,30,20],'object11c':[50,30,30],'object12c':[50,40,20],'object13c':[50,40,30],'object14c':[50,40,40],
                                   'object1d':[35,30,20],'object2d':[35,30,30],'object3d':[35,35,20],'object4d':[35,35,30],'object5d':[40,30,20],'object6d':[40,30,30],'object7d':[40,40,20],'object8d':[40,40,30],'object9d':[40,40,40],'object10d':[50,30,20],'object11d':[50,30,30],'object12d':[50,40,20],'object13d':[50,40,30],'object14d':[50,40,40],
                                   'object1e':[35,30,20],'object2e':[35,30,30],'object3e':[35,35,20],'object4e':[35,35,30],'object5e':[40,30,20],'object6e':[40,30,30],'object7e':[40,40,20],'object8e':[40,40,30],'object9e':[40,40,40],'object10e':[50,30,20],'object11e':[50,30,30],'object12e':[50,40,20],'object13e':[50,40,30],'object14e':[50,40,40],
                                   'object1f':[35,30,20],'object2f':[35,30,30],'object3f':[35,35,20],'object4f':[35,35,30],'object5f':[40,30,20],'object6f':[40,30,30],'object7f':[40,40,20],'object8f':[40,40,30],'object9f':[40,40,40],'object10f':[50,30,20],'object11f':[50,30,30],'object12f':[50,40,20],'object13f':[50,40,30],'object14f':[50,40,40],
                                   'object1g':[35,30,20],'object2g':[35,30,30],'object3g':[35,35,20],'object4g':[35,35,30],'object5g':[40,30,20],'object6g':[40,30,30],'object7g':[40,40,20],'object8g':[40,40,30],'object9g':[40,40,40],'object10g':[50,30,20],'object11g':[50,30,30],'object12g':[50,40,20],'object13g':[50,40,30],'object14g':[50,40,40],
                                   'object1h':[35,30,20],'object2h':[35,30,30],'object3h':[35,35,20],'object4h':[35,35,30],'object5h':[40,30,20],'object6h':[40,30,30],'object7h':[40,40,20],'object8h':[40,40,30],'object9h':[40,40,40],'object10h':[50,30,20],'object11h':[50,30,30],'object12h':[50,40,20],'object13h':[50,40,30],'object14h':[50,40,40],
                                   'object1i':[35,30,20],'object2i':[35,30,30],'object3i':[35,35,20],'object4i':[35,35,30],'object5i':[40,30,20],'object6i':[40,30,30],'object7i':[40,40,20],'object8i':[40,40,30],'object9i':[40,40,40],'object10i':[50,30,20],'object11i':[50,30,30],'object12i':[50,40,20],'object13i':[50,40,30],'object14i':[50,40,40]}
        
        # preset_list = [[7, 6, 4], [7, 6, 6], [7, 7, 4], [7, 7, 6], [8, 6, 4], [8, 6, 6], [8, 8, 4], [8, 8, 6], [8, 8, 8], [10, 6, 4], [10, 6, 6], [10, 8, 4], [10, 8, 6], [10, 8, 8]]


        #################################

        # self.workspace_limits = workspace_limits
        self.workspace_limits = np.asarray([[-0.65, -0.35], [-0.25, 0.25], [-0.0001, 0.4]])

        # Define colors for object meshes (Tableau palette)
        self.color_space = np.asarray([[66.0, 165.0, 245.0], # blue
                                       [76.0, 175.0, 80.0], # green
                                       [121, 85, 72], # brown
                                       [255, 167, 38], # orange
                                       [255.0, 235.0, 59.0], # yellow
                                       [159, 168, 218], # indigo
                                       [239.0, 83.0, 80.0], # red
                                       [171, 71, 188], # purple
                                       [128, 222, 234], # cyan
                                       [255, 182, 193]])/255.0 #pink

        # self.color_name = ['blue','green','brown','orange','yellow','indigo','red','purple','cyan','pink']
        # # Define the obj path for importing object meshes
        # self.obj_mesh_dir = 'objects/'
        # self.obj_mesh_color = self.color_space[np.asarray(range(self.num_obj)) % 10, :]
        self.emptyBuff = bytearray()
        self.running = True
        self.object_name_list = ['object1','object2','object3','object4','object5','object6','object7','object8','object9','object10','object11','object12','object13','object14','object15','object16','object17','object18','object19','object20','object21','object22','object23','object24']

        # Make sure to have the server side running in V-REP:
        # Connect to simulator
        vrep.simxFinish(-1) # Just in case, close all opened connections
        self.sim_client = vrep.simxStart('127.0.0.1', 19997, True, True, -500000, 5) # Connect to V-REP on port 19997
        if self.sim_client == -1:
            print('Failed to connect to simulation (V-REP remote API server). Exiting.')
            exit()
        else:
            print('Connected to simulation, initializing...')
            # Start the simulation:
            vrep.simxStartSimulation(self.sim_client,vrep.simx_opmode_oneshot_wait)
            time.sleep(4)

        res,self.robotHandle=vrep.simxGetObjectHandle(self.sim_client,'IRB4600',vrep.simx_opmode_oneshot_wait) #'UR5'
        res,self.agvHandle=vrep.simxGetObjectHandle(self.sim_client,'OmniPlatform',vrep.simx_opmode_oneshot_wait)
        # Find the camera handle
        # sim_ret, self.rgb_cam_handle = vrep.simxGetObjectHandle(self.sim_client, 'kinect_rgb', vrep.simx_opmode_blocking)
        # sim_ret, self.dep_cam_handle = vrep.simxGetObjectHandle(self.sim_client, 'kinect_depth', vrep.simx_opmode_blocking)

        sim_ret, self.cam_handle = vrep.simxGetObjectHandle(self.sim_client, 'Vision_sensor2', vrep.simx_opmode_blocking)
        sim_ret, self.tip_handle = vrep.simxGetObjectHandle(self.sim_client, 'IkTip', vrep.simx_opmode_blocking)
        time.sleep(1.5)

        # # for v2 version only #
        # self.GRASP_POSE = None
        # self.CAMERA_TASK_ID = 1
        # self.UR5_TASK_ID = 0
        # self.bottom_depth_value = 0.9990012117475272
        # #######################

    def wait_for_execution(self):
        # Wait until the end of the movement:
        self.running = True
        while self.running:
            res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.sim_client,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'isRunning',[self.robotHandle],[],[],self.emptyBuff,vrep.simx_opmode_oneshot_wait)
            self.running=retInts[0]==1
    def pick_and_place(self, item_drop_pos, item, bin_start_pos):
        '''
        @author: Chenkun Zhao
        @input: item_drop_pos: 也就是item初始化的位置
                item: 需要抓取的物体 具有位置 尺寸等属性
                bin_start_pos: 箱子起始点坐标,因为需要考虑到agv小车到达的位置不定以及机械臂两边二维坐标的对称性
        @output: 无
        '''
        wid, hei, dep = item.getDimension()
        self.move_to_position([item_drop_pos[0],item_drop_pos[1], 2.5, 0.70710641, -0.70710641,  0, 0.00101927])
        self.control_suctionPad('open')
        self.move_to_position([item_drop_pos[0],item_drop_pos[1],float(dep)/100 + 0.0025+0.005/2, 0.70710641, -0.70710641,  0, 0.00101927])
        #abb.move_to_position([0.5,-1.5,2, 0.70710641, -0.70710641,  0, 0.00101927])
        x, y, z = item.position
        wid, hei, dep = item.getDimension()
        #已修改，将这部分的长度和宽度和旋转方向对应
        #这里需要修改
        #不应该是width height 和depth 应该是得到dimension之后的值或者  判断旋转方向再赋值
        position = float(x) * 1.225/1.2 + float(wid)/2, float(y) * 1.025 + float(hei)/2, float(z + dep)
        
        
        if bin_start_pos[1] > 0:
            bin_y = -position[0]/100.0 + bin_start_pos[1]
            bin_x = bin_start_pos[0] + position[1]/100.0 # action[1]/100.0 + (0.2)
        elif bin_start_pos[1] < 0:
            bin_y = +position[0]/100.0 + bin_start_pos[1]
            bin_x = bin_start_pos[0] + position[1]/100.0 # action[1]/100.0 + (0.2)
        #bin_x = -position[0]/100.0 + (1.4)
        #bin_y = 0.5 - position[1]/100.0 # action[1]/100.0 + (0.2)
        #bin_z = position[2]/100.0 + 0.076923/2 + 0.35
        bin_z = position[2]/100.0 + bin_start_pos[2]
        print(bin_x,bin_y,bin_z)
        self.move_to_position([item_drop_pos[0],item_drop_pos[1],2.5, 0.70710641, -0.70710641,  0, 0.00101927])
        #abb.move_to_position([bin_x,-1.5,1.6, 0.70710641, -0.70710641,  0, 0.00101927])
        self.move_to_position([bin_x,bin_y,2.5, 0.70710641, -0.70710641,  0, 0.00101927])
        self.move_to_position([bin_x,bin_y,bin_z, 0.70710641, -0.70710641,  0, 0.00101927])
        self.control_suctionPad('close')
        self.move_to_position([bin_x,bin_y,2.5, 0.70710641, -0.70710641,  0, 0.00101927])
    
    def start_conveyor_left(self):
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor1Signal', 31, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor2Signal', 32, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor6Signal', 36, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor8Signal', 38, vrep.simx_opmode_oneshot_wait)
        time.sleep(0.1)
    def stop_conveyor_left(self):
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor1Signal', -31, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor2Signal', -32, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor6Signal', -36, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor8Signal', -38, vrep.simx_opmode_oneshot_wait)
        time.sleep(0.1)
    def start_conveyor_right(self):
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor3Signal', 33, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor4Signal', 34, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor5Signal', 35, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor7Signal', 37, vrep.simx_opmode_oneshot_wait)
        time.sleep(0.1)
    def stop_conveyor_right(self):
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor3Signal', -33, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor4Signal', -34, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor5Signal', -35, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetIntegerSignal(self.sim_client, 'Conveyor7Signal', -37, vrep.simx_opmode_oneshot_wait)
        time.sleep(0.1)
    
    
    
    
    
    def move_to_position(self,position):
        '''
        Description: move the robot to given position
        Params: position, [x,y,z,q1,q2,q3,q4], (x,y,z) is the coordinate in world coordinate system, (q1,q2,q3,q4) is the quaternion that defines the target orientation
        '''
        # print('move position: ', position)
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.sim_client,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'runActionfromClient',[self.robotHandle],position,[],self.emptyBuff,vrep.simx_opmode_oneshot_wait)
        self.wait_for_execution()

    # def run_agv1(self):
    #     '''
    #     Description: move the robot to given position
    #     Params: position, [x,y,z,q1,q2,q3,q4], (x,y,z) is the coordinate in world coordinate system, (q1,q2,q3,q4) is the quaternion that defines the target orientation
    #     '''
    #     # print('move position: ', position)
    #     print('here ......')
    #     # res,retInts,retFloats,retStrings,retBuffer = vrep.simxCallScriptFunction(self.sim_client,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'runAGVcar',[self.agvHandle],position,[],self.emptyBuff,vrep.simx_opmode_oneshot_wait)
    #     vrep.simxSetIntegerSignal(self.sim_client, 'runAGV1', 5, vrep.simx_opmode_oneshot_wait)

    def set_object_color(self,obj_name,color_arr):
        sim_ret, obj_handle = vrep.simxGetObjectHandle(self.sim_client, obj_name, vrep.simx_opmode_blocking)
        res,retInts,retFloats,retStrings,retBuffer = vrep.simxCallScriptFunction(self.sim_client,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setObjectColor',[obj_handle],color_arr,[],self.emptyBuff,vrep.simx_opmode_oneshot_wait)

    def move_to_position_asyc(self,position):
        '''
        Description: move the robot to given position
        Params: position, [x,y,z,q1,q2,q3,q4], (x,y,z) is the coordinate in world coordinate system, (q1,q2,q3,q4) is the quaternion that defines the target orientation
        '''
        res,retInts,retFloats,retStrings,retBuffer = vrep.simxCallScriptFunction(self.sim_client,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'runActionfromClient',[self.robotHandle],position,[],self.emptyBuff,vrep.simx_opmode_oneshot_wait)

    def control_gripper(self, action_type):
        '''
        Description: close/open the Robotiq two-finger gripper
        Params: action_type, 'open'-open the gripper; 'close'-close the gripper
        '''
        if action_type == 'close':
            res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.sim_client,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'closeAction',[self.robotHandle],[0],[],self.emptyBuff,vrep.simx_opmode_oneshot_wait)
            time.sleep(2)
        elif action_type == 'open':
            res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.sim_client,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'closeAction',[self.robotHandle],[1],[],self.emptyBuff,vrep.simx_opmode_oneshot_wait)
            time.sleep(2)
        else:
            print('control gripper command cancel, invalid param!')
    def get_object_mass(self, object_name):
        sim_ret, object_handle = vrep.simxGetObjectHandle(self.sim_client, object_name, vrep.simx_opmode_blocking)        
        ret_resp,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.sim_client,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getObjMass',[object_handle],[],[],self.emptyBuff,vrep.simx_opmode_oneshot_wait)
        return retFloats[0]

    """ def set_object_mass(self, object_name, mass_value):
	    sim_ret, object_handle = vrep.simxGetObjectHandle(self.sim_client, object_name, vrep.simx_opmode_blocking)	
        ret_resp,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.sim_client,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setObjMass',[object_handle],[mass_value],[],self.emptyBuff,vrep.simx_opmode_oneshot_wait)
 """

    def check_suction(self):
        z_force = 0 #self.get_force_sensor_data()[2]
        return z_force > 0.5

    def control_suctionPad(self,action_type):
        '''
        Description: close/open the Robotiq two-finger gripper
        Param flag, 'open' for open suc and pick object; 'close' for close suc and release object
        '''
        if action_type == 'close':
            vrep.simxSetIntegerSignal(self.sim_client, 'suctionActive', 0, vrep.simx_opmode_oneshot_wait)
            time.sleep(0.1)
        elif action_type == 'open':
            vrep.simxSetIntegerSignal(self.sim_client, 'suctionActive', 1, vrep.simx_opmode_oneshot_wait)
            time.sleep(0.1)
        else:
            print('control suction gripper command cancel, invalid param!')

    def control_agv(self,signal_name,signal_value):
        vrep.simxSetIntegerSignal(self.sim_client, signal_name, signal_value, vrep.simx_opmode_oneshot_wait)
        time.sleep(0.1)

    def get_kinect_data(self):
        ######## get color(rgb) data, 640*480 #########
        sim_ret, resolution, raw_image = vrep.simxGetVisionSensorImage(self.sim_client, self.rgb_cam_handle, 0, vrep.simx_opmode_blocking)
        color_img = np.asarray(raw_image)
        color_img.shape = (resolution[1], resolution[0], 3)
        color_img = color_img.astype(np.float)/255
        color_img[color_img < 0] += 1
        color_img *= 255
        color_img = np.fliplr(color_img)
        color_img = color_img.astype(np.uint8)
        color_img = cv2.cvtColor(color_img,cv2.COLOR_RGB2BGR)
        # cv2.imwrite('rgb.png',color_img)

        ######## get depth data, 640*480 #########
        sim_ret, resolution, depth_buffer = vrep.simxGetVisionSensorDepthBuffer(self.sim_client, self.dep_cam_handle, vrep.simx_opmode_blocking)
        depth_img = np.asarray(depth_buffer)
        depth_img.shape = (resolution[1], resolution[0])
        depth_img = np.fliplr(depth_img)
        zNear = 0.01
        zFar = 3.5
        depth_img = depth_img * (zFar - zNear) + zNear #mm
        # cv2.imwrite('depth.png',depth_img*100)
        # depth_new = (depth_img*1000).astype(np.uint16) # mm
        # cv2.imwrite('depth_new.png', depth_new)

        return color_img, depth_img

    def get_vision_sensor_data(self):
        # Get color image from simulation
        sim_ret, resolution, raw_image = vrep.simxGetVisionSensorImage(self.sim_client, self.cam_handle, 0, vrep.simx_opmode_blocking)
        color_img = np.asarray(raw_image)
        color_img.shape = (resolution[1], resolution[0], 3)
        color_img = color_img.astype(np.float)/255
        color_img[color_img < 0] += 1
        color_img *= 255
        color_img = np.fliplr(color_img)
        color_img = color_img.astype(np.uint8)
        color_img = cv2.cvtColor(color_img,cv2.COLOR_RGB2BGR)
        color_img = cv2.rotate(color_img, cv2.ROTATE_180)

        # Get depth image from simulation
        sim_ret, resolution, depth_buffer = vrep.simxGetVisionSensorDepthBuffer(self.sim_client, self.cam_handle, vrep.simx_opmode_blocking)
        depth_img = np.asarray(depth_buffer)
        depth_img.shape = (resolution[1], resolution[0])
        depth_img = np.fliplr(depth_img)
        zNear = 0.01
        zFar = 10
        depth_img = depth_img * (zFar - zNear) + zNear
        depth_img = cv2.rotate(depth_img, cv2.ROTATE_180)
        ###########################
        depth_new = (depth_img*1000).astype(np.uint16) # mm
        cv2.imwrite('depth.png', depth_new)
        cv2.imwrite('color.jpg', color_img)
        ###########################
        return color_img, depth_img

    def get_point_cloud_from_vision_sensor(self):
        # Get color image from simulation
        sim_ret, resolution, raw_image = vrep.simxGetVisionSensorImage(self.sim_client, self.cam_handle, 0, vrep.simx_opmode_blocking)
        color_img = np.asarray(raw_image)
        color_img.shape = (resolution[1], resolution[0], 3)
        color_img = color_img.astype(np.float)/255
        color_img[color_img < 0] += 1
        color_img *= 255
        color_img = np.fliplr(color_img)
        color_img = color_img.astype(np.uint8)
        color_img = cv2.cvtColor(color_img,cv2.COLOR_RGB2BGR)
        color_img = cv2.rotate(color_img, cv2.ROTATE_180)

        # Get depth image from simulation
        sim_ret, resolution, depth_buffer = vrep.simxGetVisionSensorDepthBuffer(self.sim_client, self.cam_handle, vrep.simx_opmode_blocking)
        depth_img = np.asarray(depth_buffer)
        depth_img.shape = (resolution[1], resolution[0])
        depth_img = np.fliplr(depth_img)
        zNear = 0.01
        zFar = 10
        depth_img = depth_img * (zFar - zNear) + zNear
        depth_img = cv2.rotate(depth_img, cv2.ROTATE_180)
        return color_img, depth_img

    def add_objects(self, object_size, drop_pos, name):
        shape_name = name#'test_object'
        # object_size = [0.2, 0.2, 0.15]
        drop_x = drop_pos[0] #(self.workspace_limits[0][1] - self.workspace_limits[0][0] - 0.2) * np.random.random_sample() + self.workspace_limits[0][0] + 0.1
        drop_y = drop_pos[1] #(self.workspace_limits[1][1] - self.workspace_limits[1][0] - 0.2) * np.random.random_sample() + self.workspace_limits[1][0] + 0.1
        drop_z = drop_pos[2]
        object_position = [drop_x, drop_y, drop_z]
        object_orientation = [0, 0, 90/180.0*np.pi]
        color_id = np.random.randint(0,10)
        object_color = [self.color_space[color_id][0],self.color_space[color_id][1],self.color_space[color_id][2]]
        ret_resp,ret_ints,ret_floats,ret_strings,ret_buffer = vrep.simxCallScriptFunction(self.sim_client, 'remoteApiCommandServer',vrep.sim_scripttype_childscript,'createObject',[0,0,255,0], object_position + object_orientation + object_color + object_size, [shape_name], bytearray(), vrep.simx_opmode_blocking)
        if ret_resp == 8:
            print('Failed to create new object !')
            exit()
        curr_shape_handle = ret_ints[0]


    def set_random_positions_for_object(self):
        for i in range(len(self.object_name_list)):
            obj_name = self.object_name_list[i]
            sim_ret, object_handle = vrep.simxGetObjectHandle(self.sim_client, obj_name, vrep.simx_opmode_blocking)
            drop_x = random.uniform(self.workspace_limits[0][0] + 0.01, self.workspace_limits[0][1] - 0.01)
            drop_y = random.uniform(self.workspace_limits[1][0] + 0.01, self.workspace_limits[1][1] - 0.01)
            object_position = [drop_x, drop_y, 0.12]
            # nishizhen positive; shunshizhen negtive
            rand_angle = np.random.randint(0, 60)
            # sim_ret, object_orientation = vrep.simxGetObjectOrientation(self.sim_client, object_handle, -1, vrep.simx_opmode_blocking)
            object_orientation = [0, 0, rand_angle/180.0*np.pi] #[-np.pi/2, 0, rand_angle/180.0*np.pi]#rand_angle/180.0*np.pi#[-np.pi/2, 0, rand_angle/180.0*np.pi]
            vrep.simxSetObjectPosition(self.sim_client, object_handle, -1, object_position, vrep.simx_opmode_blocking)
            vrep.simxSetObjectOrientation(self.sim_client, object_handle, -1, object_orientation, vrep.simx_opmode_blocking)
            time.sleep(0.2)

    def set_object_position_by_name(self, object_name, object_position, object_orientation):
        sim_ret, object_handle = vrep.simxGetObjectHandle(self.sim_client, object_name, vrep.simx_opmode_blocking)
        vrep.simxSetObjectPosition(self.sim_client, object_handle, -1, object_position, vrep.simx_opmode_blocking)
        vrep.simxSetObjectOrientation(self.sim_client, object_handle, -1, object_orientation, vrep.simx_opmode_blocking)

    def get_object_position_by_name(self, object_name):
        sim_ret, tmp_handle = vrep.simxGetObjectHandle(self.sim_client, object_name, vrep.simx_opmode_blocking)
        sim_ret, object_position = vrep.simxGetObjectPosition(self.sim_client, tmp_handle, -1, vrep.simx_opmode_blocking)
        sim_ret, object_orientation = vrep.simxGetObjectOrientation(self.sim_client, tmp_handle, -1, vrep.simx_opmode_blocking)

        return object_position, object_orientation

    def get_static_object_position_and_angle(self):
        obj_positions = []
        obj_orientations = []
        for name in self.object_name_list:
            sim_ret, tmp_handle = vrep.simxGetObjectHandle(self.sim_client, name, vrep.simx_opmode_blocking)
            sim_ret, object_position = vrep.simxGetObjectPosition(self.sim_client, tmp_handle, -1, vrep.simx_opmode_blocking)
            sim_ret, object_orientation = vrep.simxGetObjectOrientation(self.sim_client, tmp_handle, -1, vrep.simx_opmode_blocking)
            obj_positions.append(object_position)
            obj_orientations.append(object_orientation)

        return obj_positions, obj_orientations

    def get_robot_current_state(self):
        sim_ret, object_position = vrep.simxGetObjectPosition(self.sim_client, self.tip_handle, -1, vrep.simx_opmode_blocking)
        sim_ret, object_orientation = vrep.simxGetObjectOrientation(self.sim_client, self.tip_handle, -1, vrep.simx_opmode_blocking)
        return object_position, object_orientation




    ########## only for bin packing ###########
    def load_five_item_at_start(self):
        curr_item_name_list = random.sample(self.all_item_list,5)
        # self.current_item_name = curr_item_name
        # self.current_item_size = self.all_item_size_dict[curr_item_name]
        ind = 0
        for curr_item_name in curr_item_name_list:
            self.all_item_list.remove(curr_item_name)
            self.set_object_color(curr_item_name, self.color_space[np.random.randint(0,10)])
            self.set_object_position_by_name(curr_item_name,[self.five_loaded_item_position[ind][0], self.five_loaded_item_position[ind][1], self.all_item_size_dict[curr_item_name][2]/200.0 +0.01+0.67], [0, 0, 90/180.0*np.pi]) #90/180.0*np.pi
            self.five_loaded_item_name.append(curr_item_name)
            self.five_loaded_item_height.append(self.all_item_size_dict[curr_item_name][2])
            ind += 1
            time.sleep(1)

        return self.current_item_size

    def select_max_item_from_five_items(self):
        volume_list = np.zeros(5)
        for i in range(5):
            tmp_name = self.five_loaded_item_name[i]
            tmp_size = self.all_item_size_dict[tmp_name]
            volume_list[i] = tmp_size[0]*tmp_size[1]*tmp_size[2]

        rand_ind = np.argmax(volume_list)#np.argmax(volume_list)
        self.current_item_ind = rand_ind
        self.current_item_name = self.five_loaded_item_name[rand_ind]
        self.current_item_size = self.all_item_size_dict[self.current_item_name]
        self.current_item_position = [self.five_loaded_item_position[rand_ind][0],self.five_loaded_item_position[rand_ind][1],self.current_item_size[2]/100.0+0.01+0.68]

        # self.all_item_list.remove(curr_item_name)
        # self.set_object_position_by_name(curr_item_name,[-0.15, -0.6, self.current_item_size[2]/200.0 +0.01+0.22], [0, 0, 90/180.0*np.pi]) #90/180.0*np.pi

        return self.current_item_size

    def find_the_smallest_one(self):
        volume_list = np.zeros(5)
        for i in range(5):
            tmp_name = self.five_loaded_item_name[i]
            tmp_size = self.all_item_size_dict[tmp_name]
            volume_list[i] = tmp_size[0]*tmp_size[1]*tmp_size[2]

        rand_ind = np.argmin(volume_list)#np.argmax(volume_list)
        return rand_ind

    def select_one_item_from_five_items(self, ind=None):
        if ind == None:
            rand_ind = np.random.randint(0,5)
        else:
            rand_ind = ind
        self.current_item_ind = rand_ind
        self.current_item_name = self.five_loaded_item_name[rand_ind]
        self.current_item_size = self.all_item_size_dict[self.current_item_name]
        self.current_item_position = [self.five_loaded_item_position[rand_ind][0],self.five_loaded_item_position[rand_ind][1],self.current_item_size[2]/100.0+0.01+0.68]

        # self.all_item_list.remove(curr_item_name)
        # self.set_object_position_by_name(curr_item_name,[-0.15, -0.6, self.current_item_size[2]/200.0 +0.01+0.22], [0, 0, 90/180.0*np.pi]) #90/180.0*np.pi

        return self.current_item_size

    def select_item_from_five_items_by_id(self, ind):
        rand_ind = ind
        current_item_name = self.five_loaded_item_name[rand_ind]
        current_item_size = self.all_item_size_dict[current_item_name]

        return current_item_size

    def get_random_current_item(self):
        curr_item_name = random.sample(self.all_item_list,1)[0]
        self.current_item_name = curr_item_name
        self.current_item_size = self.all_item_size_dict[curr_item_name]
        self.all_item_list.remove(curr_item_name)
        self.set_object_position_by_name(curr_item_name,[-0.15, -0.6, self.current_item_size[2]/200.0 +0.01+0.22], [0, 0, 90/180.0*np.pi]) #90/180.0*np.pi

        return self.current_item_size

    # def pick_item_from_converyor(self, rotation_flag):
    #     # self.current_item_ind
    #     # self.current_item_size
    #     print('five height: ', self.five_loaded_item_height)
    #     if self.current_item_ind == 4:
    #         height_limit = np.max(np.array([self.five_loaded_item_height[2],self.five_loaded_item_height[3],self.five_loaded_item_height[4]]))
    #     elif self.current_item_ind == 3:
    #         height_limit = np.max(np.array([self.five_loaded_item_height[1],self.five_loaded_item_height[2],self.five_loaded_item_height[3]]))
    #     elif self.current_item_ind == 2:
    #         height_limit = np.max(np.array([self.five_loaded_item_height[0],self.five_loaded_item_height[1],self.five_loaded_item_height[2]]))
    #     elif self.current_item_ind == 1:
    #         height_limit = np.max(np.array([self.five_loaded_item_height[0],self.five_loaded_item_height[1]]))
    #     elif self.current_item_ind == 0:
    #         height_limit = np.max(np.array([self.five_loaded_item_height[0],self.five_loaded_item_height[1]]))

    #     # rotation quat [1.00000000e+00 0.00000000e+00 0.00000000e+00 2.67948966e-08]
    #     curr_item_pos, curr_item_ang = self.get_object_position_by_name(self.current_item_name)
    #     item_pick_position = [curr_item_pos[0],curr_item_pos[1],self.all_item_size_dict[self.current_item_name][2]/100.0]
    #     item_pick_position[2] = item_pick_position[2] + 0.67 #0.22
    #     # self.control_suctionPad('open')
    #     if rotation_flag == 1:
    #         self.move_to_position([item_pick_position[0], item_pick_position[1], item_pick_position[2] + 0.15, 0.70710641, -0.70710641,  0, 0.00101927])
    #         self.move_to_position([item_pick_position[0], item_pick_position[1], item_pick_position[2], 0.70710641, -0.70710641,  0, 0.00101927])
    #         self.control_suctionPad('open')
    #         self.move_to_position([item_pick_position[0], item_pick_position[1], item_pick_position[2] + height_limit/100.0, 0.70710641, -0.70710641,  0, 0.00101927])
    #     if rotation_flag == 2:#rotate
    #         self.move_to_position([item_pick_position[0], item_pick_position[1], item_pick_position[2] + 0.15, 1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 2.67948966e-08])
    #         self.move_to_position([item_pick_position[0], item_pick_position[1], item_pick_position[2], 1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 2.67948966e-08])
    #         self.control_suctionPad('open')
    #         self.move_to_position([item_pick_position[0], item_pick_position[1], item_pick_position[2] + height_limit/100.0, 0.70710641, -0.70710641,  0, 0.00101927])
    
    # def step_place_action(self, action, item_height): #1-not 2-rotate
    #     # # for UR5 robot #
    #     # rotation_flag = action[3]
    #     # self.pick_item_from_converyor(rotation_flag)
    #     # # self.place_item_in_bin(action[0] / 10 * 0.5, action[1] / 10 * 0.5, action[2] / 10 * 0.5)
    #     # bin_y = action[0]/100.0 + (-0.25)
    #     # bin_x = action[1]/100.0 + (-0.65)
    #     # bin_z = action[2]/100.0 + 0.008 + 0.15
    #     # if bin_z <= 0.15:
    #     #     add_height = 0.2
    #     # elif bin_z > 0.15 and bin_z <= 0.25:
    #     #     add_height = 0.1
    #     # elif bin_z > 0.25:
    #     #     add_height = 0.06
    #     # self.move_to_position([bin_x,bin_y,0.45,0.70710641, -0.70710641,  0, 0.00101927])
    #     # self.move_to_position([bin_x,bin_y,bin_z,0.70710641, -0.70710641,  0, 0.00101927])
    #     # self.control_suctionPad('close')
    #     # self.move_to_position([bin_x,bin_y,0.45,0.70710641, -0.70710641,  0, 0.00101927])


    #     # for ABB robot #
    #     rotation_flag = action[3]
    #     self.pick_item_from_converyor(rotation_flag)
    #     # self.place_item_in_bin(action[0] / 10 * 0.5, action[1] / 10 * 0.5, action[2] / 10 * 0.5)
    #     bin_y = action[0]/100.0 + (-0.6)
    #     bin_x = 1.4 - action[1]/100.0 # action[1]/100.0 + (0.2)
    #     bin_z = action[2]/100.0 + 0.008 + 0.11
    #     #####################
    #     if self.bin_max_height < bin_z:
    #         self.bin_max_height = bin_z
    #     #####################
    #     self.move_to_position([bin_x,bin_y,max(self.bin_max_height+0.05, 0.75),0.70710641, -0.70710641,  0, 0.00101927])#1.35
    #     self.move_to_position([bin_x,bin_y,bin_z,0.70710641, -0.70710641,  0, 0.00101927])
    #     self.control_suctionPad('close')
    #     self.move_to_position([bin_x,bin_y,max(self.bin_max_height+0.05, 0.75),0.70710641, -0.70710641,  0, 0.00101927])
    #     # add item to keep five #
    #     rand_item_name = random.sample(self.all_item_list,1)[0]
    #     self.five_loaded_item_name[self.current_item_ind] = rand_item_name
    #     self.five_loaded_item_height[self.current_item_ind] = self.all_item_size_dict[rand_item_name][2]
    #     self.all_item_list.remove(rand_item_name)
    #     self.set_object_position_by_name(rand_item_name,[self.five_loaded_item_position[self.current_item_ind][0], self.five_loaded_item_position[self.current_item_ind][1], self.all_item_size_dict[rand_item_name][2]/200.0 +0.01+0.68], [0, 0, 90/180.0*np.pi]) #90/180.0*np.pi


    def pick_item_from_converyor(self, rotation_flag):
        # self.current_item_ind
        # self.current_item_size
        # print('five height: ', self.five_loaded_item_height)
        if self.current_item_ind == 4:
            height_limit = np.max(np.array([self.five_loaded_item_height[1],self.five_loaded_item_height[2],self.five_loaded_item_height[3],self.five_loaded_item_height[4]]))
        elif self.current_item_ind == 3:
            height_limit = np.max(np.array([self.five_loaded_item_height[0],self.five_loaded_item_height[1],self.five_loaded_item_height[2],self.five_loaded_item_height[3]]))
        elif self.current_item_ind == 2:
            height_limit = np.max(np.array([self.five_loaded_item_height[0],self.five_loaded_item_height[1],self.five_loaded_item_height[2]]))
        elif self.current_item_ind == 1:
            height_limit = np.max(np.array([self.five_loaded_item_height[0],self.five_loaded_item_height[1]]))
        elif self.current_item_ind == 0:
            height_limit = np.max(np.array([self.five_loaded_item_height[0],self.five_loaded_item_height[1]]))

        # rotation quat [1.00000000e+00 0.00000000e+00 0.00000000e+00 2.67948966e-08]
        curr_item_pos, curr_item_ang = self.get_object_position_by_name(self.current_item_name)
        item_pick_position = [curr_item_pos[0],curr_item_pos[1],self.all_item_size_dict[self.current_item_name][2]/100.0]
        item_pick_position[2] = item_pick_position[2] + 0.47 #0.22
        self.control_suctionPad('open')
        if rotation_flag == 1:
            self.move_to_position([item_pick_position[0], item_pick_position[1], item_pick_position[2] + 0.15, 0.70710641, -0.70710641,  0, 0.00101927])
            # self.control_suctionPad('open')
            self.move_to_position([item_pick_position[0], item_pick_position[1], item_pick_position[2], 0.70710641, -0.70710641,  0, 0.00101927])
            self.move_to_position([item_pick_position[0], item_pick_position[1], item_pick_position[2] + height_limit/100.0 +0.08, 0.70710641, -0.70710641,  0, 0.00101927])
            # self.move_to_position([item_pick_position[0], item_pick_position[1]+0.51, item_pick_position[2] + 0.1, 0.70710641, -0.70710641,  0, 0.00101927])
        if rotation_flag == 2:#rotate
            self.move_to_position([item_pick_position[0], item_pick_position[1], item_pick_position[2] + 0.18, 1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 2.67948966e-08])
            # self.control_suctionPad('open')
            self.move_to_position([item_pick_position[0], item_pick_position[1], item_pick_position[2], 1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 2.67948966e-08])
            self.move_to_position([item_pick_position[0], item_pick_position[1], item_pick_position[2] + height_limit/100.0 + 0.08, 0.70710641, -0.70710641,  0, 0.00101927])
            # self.move_to_position([item_pick_position[0], item_pick_position[1]+0.51, item_pick_position[2] + 0.1, 0.70710641, -0.70710641,  0, 0.00101927])

        return [item_pick_position[0], item_pick_position[1], item_pick_position[2]]

    def step_place_action(self, action, item_height): #1-not 2-rotate
        # for ABB robot #
        rotation_flag = action[3]
        self.pick_item_from_converyor(rotation_flag)

        # # 原来的设置
        # bin_y = action[0]/100.0 + (-0.75)
        # bin_x = 1.55 - action[1]/100.0 # action[1]/100.0 + (0.2)
        # bin_z = action[2]/100.0 + 0.008 + 0.11

        # 改左下角开始放 为 右上角开始放，避免放置后期的碰撞问题
        bin_y = 0.75 - action[0]/100.0
        bin_x = action[1]/100.0 + 0.05# action[1]/100.0 + (0.2)
        bin_z = action[2]/100.0 + 0.008 + 0.11

        if self.bin_max_height <= 35:
            up_h = 1.05
        elif self.bin_max_height > 35 and self.bin_max_height < 70:
            up_h = 1.25
        else:
            up_h = 1.35

        self.move_to_position([bin_x,bin_y,up_h,0.70710641, -0.70710641,  0, 0.00101927])#1.35
        self.move_to_position([bin_x,bin_y,bin_z,0.70710641, -0.70710641,  0, 0.00101927])
        self.control_suctionPad('close')
        self.move_to_position([bin_x,bin_y,max(bin_z+0.1,up_h),0.70710641, -0.70710641,  0, 0.00101927])
        # add item to keep five #
        rand_item_name = random.sample(self.all_item_list,1)[0]
        self.five_loaded_item_name[self.current_item_ind] = rand_item_name
        self.five_loaded_item_height[self.current_item_ind] = self.all_item_size_dict[rand_item_name][2]
        self.all_item_list.remove(rand_item_name)
        self.set_object_color(rand_item_name, self.color_space[np.random.randint(0,10)])
        self.set_object_position_by_name(rand_item_name,[self.five_loaded_item_position[self.current_item_ind][0], self.five_loaded_item_position[self.current_item_ind][1], self.all_item_size_dict[rand_item_name][2]/200.0 +0.01+0.68], [0, 0, 90/180.0*np.pi]) #90/180.0*np.pi

        #####################
        if self.bin_max_height < action[2]:
            self.bin_max_height = action[2]
        #####################

    def step_place_action_with_path_optimization(self, action, item_height, heightmap): #1-not 2-rotate
        # for ABB robot #
        rotation_flag = action[3]
        place_position = self.pick_item_from_converyor(rotation_flag)

        # 原来的设置
        bin_y = action[0]/100.0 + (-0.75)
        bin_x = 1.55 - action[1]/100.0 # action[1]/100.0 + (0.2)
        bin_z = action[2]/100.0 + 0.008 + 0.11
        pick_position = [bin_x, bin_y, bin_z]

        # # 改左下角开始放 为 右上角开始放，避免放置后期的碰撞问题
        # bin_y = 0.75 - action[0]/100.0
        # bin_x = action[1]/100.0 + 0.05# action[1]/100.0 + (0.2)
        # bin_z = action[2]/100.0 + 0.008 + 0.11
        # pick_position = [bin_x, bin_y, bin_z]



        # if self.bin_max_height <= 35:
        #     up_h = 1.05
        # elif self.bin_max_height > 35 and self.bin_max_height < 70:
        #     up_h = 1.25
        # else:
        #     up_h = 1.35


        mini_safe_height = self.find_shortest_safe_path(pick_position, place_position, heightmap)
        mini_safe_height = mini_safe_height/100.0 + 0.1 + self.current_item_size[2]/100.0 + 0.05
        mini_safe_height = max(mini_safe_height, 1)

        self.move_to_position([bin_x,bin_y,mini_safe_height,0.70710641, -0.70710641,  0, 0.00101927])#1.35
        self.move_to_position([bin_x,bin_y,bin_z,0.70710641, -0.70710641,  0, 0.00101927])
        self.control_suctionPad('close')
        self.move_to_position([bin_x,bin_y,mini_safe_height + 0.1,0.70710641, -0.70710641,  0, 0.00101927])

        # add item to keep five #
        rand_item_name = random.sample(self.all_item_list,1)[0]
        self.five_loaded_item_name[self.current_item_ind] = rand_item_name
        self.five_loaded_item_height[self.current_item_ind] = self.all_item_size_dict[rand_item_name][2]
        self.all_item_list.remove(rand_item_name)
        self.set_object_color(rand_item_name, self.color_space[np.random.randint(0,10)])
        self.set_object_position_by_name(rand_item_name,[self.five_loaded_item_position[self.current_item_ind][0], self.five_loaded_item_position[self.current_item_ind][1], self.all_item_size_dict[rand_item_name][2]/200.0 +0.01+0.68], [0, 0, 90/180.0*np.pi]) #90/180.0*np.pi

        #####################
        if self.bin_max_height < action[2]:
            self.bin_max_height = action[2]
        #####################

    # def find_shortest_safe_path(pick_position, place_position, current_heightmap, selected_item_id, five_items_size, five_item_position):
    def find_shortest_safe_path(self, pick_position, place_position, current_heightmap):
        '''
        part 1: find a path for picking the current item
        '''



        '''
        part 2: find a path for placing the current item
        '''
        # 定义整个场景的二维图，尺寸覆盖为y:[-1.8,0.75], x:[-1.75,1.55]
        scene_mask = cv2.imread('scene_mask.jpg') #单位cm, w-y, h-x
        
        # 根据pick和place位置确定路径起点和终点，并转换到scene_mask相同尺度
        u_s = int((pick_position[1] + 1.8)*100)
        v_s = int((pick_position[0] + 1.75)*100)
        u_e = int((place_position[1] + 1.8)*100)
        v_e = int((place_position[0] + 1.75)*100)
        start_position = [u_s, v_s]
        end_position = [u_e, v_e]
        print('pick_position: ', pick_position)
        print('place_position: ', place_position)
        print('start_position: ', start_position)
        print('end_position: ', end_position)
        
        # 将bin的当前heightmap状态更新到scene_mask副本上，最后一步时用于检查最大值
        current_heightmap = current_heightmap[::-1]
        scene_mask_copy = np.zeros((330,255))
        scene_mask_copy[180:,105:] = current_heightmap

        # 确定当前待放置的item在x方向上的尺寸
        x_size = self.current_item_size[1]

        # 连接起点和终点，确定路径线
        scene_mask = cv2.line(scene_mask, (u_s,v_s), (u_e,v_e),(255,255,255), x_size)

        # 保存可视化结果
        cv2.imwrite('path_plan_vis.png', scene_mask)
        cv2.imwrite('height_map_vis.png',current_heightmap*5)

        # 将scene_mask处理为二值化
        scene_mask = cv2.cvtColor(scene_mask, cv2.COLOR_BGR2GRAY)
        ret, binary = cv2.threshold(scene_mask, 127, 255, cv2.THRESH_BINARY)

        # 在路径线上寻找最大值
        path_region = scene_mask_copy[np.where(binary==255)]
        path_region_max_h = np.max(path_region)
        print('path_region_max_h: ', path_region_max_h)

        return path_region_max_h


