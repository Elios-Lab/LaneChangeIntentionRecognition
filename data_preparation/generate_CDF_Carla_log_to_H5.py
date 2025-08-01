# Log to h5 script
# This code take as input log files in the folder logs of data collection and prepare the h5 files in the folder h5, following the L3pilot/Hi-drive Common Data Format.

# Insert the name of the file you want to generate from the carla log folder (e.g. log_001 for the file logs/log_001.txt)
names= ["log_001"]

# Don't change the following lines, they are used to produce the hdf5 file in h5 folder
for log_name in names:
    import os
    #Carla 
    Carla = os.path.join(os.getcwd(), "..", "maps","LC_Simulator","CarlaUE4.exe")

    # input path
    log_file = os.path.join(os.getcwd(), "..", "data_collection", "logs", log_name + ".txt")

    #output path
    raw_file = os.path.join(os.getcwd(), "h5", log_name+".h5")

    """ Generates hdf5 structure from carla log file
        This is just a partial code, the features not covered here, will be left to '0' in hdf5 file
    """

    import os
    import datetime
    import time
    from utility.structure_definitions import *
    from h5py import Dataset
    import numpy as np
    import re
    import math
    import platform
    import subprocess
    import psutil


    __author__ = ""
    __version__ = "0.1"
    __maintainer__ = ""
    __email__ = ""
    __status__ = "development"



    def launch_carla_server(port, render=False):
        print('Launching Carla server on port', port)

        if platform.system() == 'Windows':
            cmd = [Carla, "-benchmark", f"-carla-rpc-port={port}"]

        else:
            #platform.system() == 'Linux':  # Changed from 'Linux' to 'Windows'
            #cmd = [r'path_to_CarlaUE4.sh',"-benchmark", f"-carla-rpc-port={port}"]
            raise NotImplementedError("For Linux usage compile the map .xodr file for linux and add path_to_CarlaUE4.sh in line 50.")

        if not render:
            cmd.append("-RenderOffScreen")

        subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(5) if not render else time.sleep(7)
        print('Carla server launched')
            
            
    def kill_old_instances(port, stop_execution=False):
        
        print('Killing old instances')
        PROCNAMES = ['CarlaUE4-Win64-Shipping.exe', 'CARLA UE4', 'CarlaUE4-Linux-Shipping']  # Added 'CARLA UE4'
        processes = [proc for proc in psutil.process_iter() if any(procname in proc.name() for procname in PROCNAMES)]
        
        for proc in processes:
            for c in proc.connections():
                if c.laddr.port == port and c.status == 'LISTEN':
                    proc.kill()
        if stop_execution:
            exit()
                
        print('No old instances found')

    #### File log.txt is a file where information are written line by line. To avoid high ram consumption, the file is read line by line using a for loop.
    # The log file is divided in sections based on the content. For example, a section introduced with a line with keyword "Positions" indicates that next lines contain position information
    # Each of these lines contain Id of the vehicle and then the position data. To keep trace of the section we are analyzing, boolean flags are used. When section starts, rise the corresponding flag, when ends, turn it off
    # ego_array_global and obj_array_global contain single frame data. During the lecture of the frame, these structs are filled. When the frame ends, "fill_frame_content" function fill the dataset with data in the structs


                
    ## this function saves only ego parameters contained in "dynamic actor" section of carla log file (linear_velocity and angular_velocity)
                
    def save_dynamic(line, deltaTime):
        
        global oldSpeed3D                                                                     # refer to global oldSpeed3D to keep trace of old state
        global acceleration3D
        
        #print("saving dynamics")
        vehicleSpeed = re.search('linear_velocity: \((.+?)\) ', line).group(1)
        vehicleSpeed3DArray = np.array([(np.double)(v) for v in vehicleSpeed.split(', ')])    # converting from string to 3 strings, one for each component and then converting strings to np.double
        ego_array_global["VehicleSpeed"][0] = np.linalg.norm(vehicleSpeed3DArray)             # euclidean norm for scalar speed value (is it right?)
        #print(f'vehicle speed: {ego_array_global["VehicleSpeed"][0]}')

        acceleration3D = (vehicleSpeed3DArray - oldSpeed3D) / deltaTime                       # acceleration computed as speed difference between consecutive frames, and divided by timedelta
        #print(f'acceleration3D: {acceleration3D} m/s^2')
        oldSpeed3D = vehicleSpeed3DArray                    # setting oldSpeed for next iteration (first iteration could be wrong, because we are supposing initial oldSpeed = 0)
        
        vehicleAngularVelocity = re.search('angular_velocity: \((.+?)\)', line).group(1)
        YawRate = (np.double)(vehicleAngularVelocity.split(', ')[2])                    # should be the [1] based on carla api documentation, but from the log seems to be the [2] (the only one not negligible)
        ego_array_global["YawRate"][0] = - YawRate * np.pi / 180                        # converting to rad/s and changing sign accordingly to toward left positive(l3pilot vs carla opposite sign)
        #print(f'angular speed: {ego_array_global["YawRate"][0]}')


    ## this function saves parameters contained in "positions" section (position and rotation). These data are stored in world coordinate, so we use temporary lists and when all the
    ## data are available, compute relative values referred to ego coordinate

    def save_absolute_positions(line, ID):

        global ego_pos, ego_rot, ego_base_vector
        
        string_pos = re.search('Location: \((.+?)\) ', line).group(1)
        pos.append(np.array([np.double(p)/100 for p in string_pos.split(", ")]))    # /100 is necessary to convert to meters
        
        string_rot = re.search('Rotation: \((.+?)\)', line).group(1)
        rot.append(np.array([(np.double)(r) for r in string_rot.split(", ")])) 
        
        if(ID == ego_ID):
            ego_pos = pos[- 1]      # last element (the just added one)
            ego_rot = rot[- 1]

            # rotation matrix that will be used for lateral, longitudinal velocity
            # sign accordingly to right hand l3pilot axis (is it right??)
            # if ego_rot[2] > 0:
            ego_base_vector = [[math.cos(math.radians(ego_rot[2])),  math.sin(math.radians(ego_rot[2]))],
                                [math.sin(math.radians(ego_rot[2])), -math.cos(math.radians(ego_rot[2]))]]
            # else:
            #     positive_ego_yaw = - ego_rot[2]
            #     ego_base_vector = [[math.cos(math.radians(positive_ego_yaw)), -math.sin(math.radians(positive_ego_yaw))],
            #                        [-math.sin(math.radians(positive_ego_yaw)), -math.cos(math.radians(positive_ego_yaw))]]

    ## this function saves parameters contained in "dynamic actor" section (linear_velocity and angular_velocity). These data are stored in world coordinate, 
    ## so we use temporary lists and when all the data are available, compute relative values referred to ego coordinate

    def save_absolute_velocity(line, ID, deltaTime):
        global ego_vel, ego_ang_vel
        
        vehicleSpeed = re.search('linear_velocity: \((.+?)\) ', line).group(1)
        vel.append(np.array([(np.double)(v) for v in vehicleSpeed.split(', ')]))    # converting from string to np.double
            
        vehicleAngularVelocity = re.search('angular_velocity: \((.+?)\)', line).group(1)
        ang_vel.append((np.double)(vehicleAngularVelocity.split(', ')[2]))
        
        
        # if the statistics are from ego vehicle, call save_dynamic to fill ego section
        
        if(ID == ego_ID):
            ego_vel = vel[-1]
            ego_ang_vel = ang_vel[-1]
            
            save_dynamic(line, deltaTime)           # call save_dynamic to fill ego section of hdf5 file
            

    ## at the end of the frame, all the absolute data are available. This function compute difference between ego and the other cars position and velocity
    ## also project data on the ego axis (which in general are rotated differently from world axis)
            
    def save_relative_values(acceleration3D, pos, rot, vel, ang_vel, ego_pos, ego_rot, IDs):    

        # save general obj statistics 
        
        obj_array_global["LeadVehicleID"][0] = ego_ID   
        obj_array_global["NumberOfObjects"][0] = len(vel)
        
        acceleration2D = acceleration3D[:2]     # considering approximatly plane world
        
        # project acceleration on the longitudinal and lateral ego axis
        
        ego_array_global["LatAcceleration"][0] = np.matmul(ego_base_vector, acceleration2D)[1]       #(np.matmul(ego_base_vector, ego_vel[:2])[0])*math.radians(ego_ang_vel)    # np.matmul(ego_base_vector, acceleration2D)[1]      # LatAcceleration = Vtang*ang_vel
        ego_array_global["LongAcceleration"][0] = np.matmul(ego_base_vector, acceleration2D)[0]      # projection on longitudinal axis

        ego_rot_360 = ego_rot[2] + 360
        if ego_rot_360 > 360:
            ego_rot_360 -= 360
        #loop until velocity vector end (should be the same using rot,pos or ang_vel arrays to obtain last index of the loop)
        # foreach vehicle compute relative values referred to ego values
        
        for ii in range(0, len(vel)):
            obj_array_global[str_obj_obj]["Classification"][0][ii] = 1              # for now all object are cars
            obj_array_global[str_obj_obj]["ID"][0][ii] = vehiclesID[ii]
            obj_array_global[str_obj_obj]["LatPosition"][0][ii] = np.matmul(ego_base_vector, (pos[ii] - ego_pos)[:2])[1]
            obj_array_global[str_obj_obj]["LatVelocity"][0][ii] = np.matmul(ego_base_vector, (vel[ii] - ego_vel)[:2])[1]    # ego vel
            try:
                obj_array_global[str_obj_obj]["Length"][0][ii] = dimensions_list[ii][0]
            except:
                obj_array_global[str_obj_obj]["Length"][0][ii] = dimensions_list[ii-1][0]
            obj_array_global[str_obj_obj]["LongPosition"][0][ii] = np.matmul(ego_base_vector, (pos[ii] - ego_pos)[:2])[0]
            obj_array_global[str_obj_obj]["LongVelocity"][0][ii] = np.matmul(ego_base_vector, (vel[ii] - ego_vel)[:2])[0]   # ego vel
            try:
                obj_array_global[str_obj_obj]["Width"][0][ii] = dimensions_list[ii][1]
                obj_array_global[str_obj_obj]["Height"][0][ii] = dimensions_list[ii][2]
            except:
                obj_array_global[str_obj_obj]["Width"][0][ii] = dimensions_list[ii-1][1]
                obj_array_global[str_obj_obj]["Height"][0][ii] = dimensions_list[ii-1][2]

            rot_360 = rot[ii][2] + 360
            if rot_360 > 360:
                rot_360 -= 360

            obj_array_global[str_obj_obj]["YawAngle"][0][ii] = math.radians(get_angle_difference(ego_rot_360, rot_360))
            obj_array_global[str_obj_obj]["YawRate"][0][ii] = ego_ang_vel - ang_vel[ii]
            #print(f' longitudinal position {ii}: {obj_array_global[str_obj_obj]["LongPosition"][0][ii]}')
            #print(f' lateral position {ii}: {obj_array_global[str_obj_obj]["LatPosition"][0][ii]}')


    # this function return the difference between angles defined in the interval [0,2pi] into an angle between (-pi,pi]
    # negative angles correspond to unit2 at the left of unit1 and viceversa positive with unit2 at the right of unit1
    def get_angle_difference(unit1, unit2):
        phi = abs(unit2-unit1) % 360
        sign = 1
        # used to calculate sign
        if not ((unit1-unit2 >= 0 and unit1-unit2 <= 180) or (
                unit1-unit2 <= -180 and unit1-unit2 >= -360)):
            sign = -1
        if phi > 180:
            result = 360-phi
        else:
            result = phi

        return result*sign


    ## this function saves only ego parameters contained in "Vehicle animations" section of carla log file (throttle, brake, steering)

    def save_animations(line):
        throttle = float(re.search('Throttle: (.+?) ', line).group(1))*100      # just scale to %
        ego_array_global["ThrottlePedalPos"][0] = (np.int32)(throttle)
        #print(f'throttle: {ego_array_global["ThrottlePedalPos"][0]}')
        
        brake = float(re.search('Brake: (.+?) ', line).group(1))*100             # just scale to %
        ego_array_global["BrakePedalPos"][0] = (np.int32)(brake)
        #print(f'brakePedal: {ego_array_global["BrakePedalPos"][0]}')
        
        steeringAngle = - (np.double)(re.search('Steering: (.+?) ', line).group(1))* np.pi * 5        # steering is normalized in carla -> passo to the full scale in l3pilot (which is +- pi*5)
        ego_array_global["SteeringAngle"][0] = (np.double)(steeringAngle)                             # "-" because in l3pilot turn left is positive (opposite of carla)
        #print(f'steeringAngle: {ego_array_global["SteeringAngle"][0]}')



    ## dimension of the bounding box is in extension: (length, width, height)

    def save_dimensions_single_vehicle(line):
        dimensions = re.search('extension: \((.+?)\)', line).group(1)
        dimensions_list.append(np.array([np.double(d)/50 for d in dimensions.split(", ")]))           # d/50 = d*2 /100 to convert cm -> m and *2 because extension in log file is taken from the center (i.e. half of the total)
        
    ## fill current frame, using ego_array_global where defined


    def save_geolocation():
        ego_transform = carla.Transform(carla.Location(ego_pos[0],ego_pos[1],ego_pos[2]), carla.Rotation(ego_rot[0],ego_rot[1],ego_rot[2]))
        ego_geolocation = world_map.transform_to_geolocation(ego_transform.location)
        position_array_global["Altitude"][0] = ego_geolocation.altitude
        position_array_global["Latitude"][0] = ego_geolocation.latitude
        position_array_global["Longitude"][0] = ego_geolocation.longitude
        #opendrive = world_map.to_opendrive()
        #print(opendrive)
        # orientation based on 0 radians heading north (i.e. "-y" in carla)
        heading_north = -(math.radians(ego_rot[2]) - math.pi) + math.pi/2
        if heading_north > 2*math.pi:
            heading_north -= 2*math.pi
        position_array_global["Heading"][0] = heading_north


    def map_lane_marking_type(lane_marking):
        if lane_marking.type == carla.LaneMarkingType.NONE:
            return 0
        elif lane_marking.type == carla.LaneMarkingType.Solid:
            return 1
        elif lane_marking.type == carla.LaneMarkingType.Broken or lane_marking.type == carla.LaneMarkingType.BrokenBroken:
            return 2
        elif lane_marking.type == carla.LaneMarkingType.SolidSolid or lane_marking.type == carla.LaneMarkingType.BrokenSolid or lane_marking.type == carla.LaneMarkingType.SolidBroken:
            return 4
        elif lane_marking.type == carla.LaneMarkingType.Other:
            return 5
        else:
            return 9


    def save_lane_lines():
        ego_transform = carla.Transform(carla.Location(ego_pos[0],ego_pos[1],ego_pos[2]), carla.Rotation(ego_rot[0],ego_rot[1],ego_rot[2]))
        
        current_location = ego_transform.location

        # if the car is not in a road(project_to_road = False) -> no lane lines information
        if world_map.get_waypoint(current_location, project_to_road=False, lane_type=carla.LaneType.Driving) is None:
            lane_array_global[str_lan_obj]["Type"][0][0] = -1
            lane_array_global[str_lan_obj]["Type"][0][1] = -1
            lane_array_global[str_lan_obj]["Type"][0][2] = -1
            lane_array_global[str_lan_obj]["Type"][0][3] = -1
            return False

        # find the nearest waypoint
        drive_waypoint = world_map.get_waypoint(current_location, project_to_road=True, lane_type=carla.LaneType.Driving)
        lane_array_global["EgoLaneWidth"][0] = drive_waypoint.lane_width
        
        left_drive_waypoint = drive_waypoint.get_left_lane()
        right_drive_waypoint = drive_waypoint.get_right_lane()
        
        distance = 0.1    # distance in meter between road waypoints to compute curvature

        right_lane_marking = drive_waypoint.right_lane_marking
        left_lane_marking = drive_waypoint.left_lane_marking

        ego_rot_360 = ego_rot[2] + 360
        if ego_rot_360 > 360:
            ego_rot_360 -= 360

        # left lane
        if right_drive_waypoint is not None and right_drive_waypoint.lane_type == carla.LaneType.Driving:
            # check if road has different direction basing on different lane id sign
            if np.sign(right_drive_waypoint.lane_id) != np.sign(drive_waypoint.lane_id):
                right_right_lane_marking = right_drive_waypoint.left_lane_marking
            else:
                right_right_lane_marking = right_drive_waypoint.right_lane_marking

            lane_array_global[str_lan_obj]["Type"][0][2] = map_lane_marking_type(right_right_lane_marking)
            lane_array_global[str_lan_obj]["Curvature"][0][2] = calcu_c_1(right_drive_waypoint, distance)
            lane_array_global[str_lan_obj]["CurvatureDx"][0][2] = calcu_c_derivative(right_drive_waypoint, distance)
            lane_array_global[str_lan_obj]["YawAngle"][0][2] = (np.double)(math.radians(get_angle_difference(ego_rot_360, right_drive_waypoint.transform.rotation.yaw)))       # towards left positive
            lane_array_global[str_lan_obj]["Dy"][0][0] = math.sqrt((right_drive_waypoint.transform.location.x - ego_pos[0]) ** 2 + (right_drive_waypoint.transform.location.y - ego_pos[1]) ** 2) - right_drive_waypoint.lane_width / 2
            lane_array_global[str_lan_obj]["Dy"][0][1] = drive_waypoint.lane_width - lane_array_global[str_lan_obj]["Dy"][0][0]
            lane_array_global[str_lan_obj]["Dy"][0][2] = lane_array_global[str_lan_obj]["Dy"][0][0] + right_drive_waypoint.lane_width
        else:
            lane_array_global[str_lan_obj]["Type"][0][2] = -1
            if right_drive_waypoint is not None:
                lane_array_global[str_lan_obj]["Dy"][0][0] = math.sqrt(
                    (right_drive_waypoint.transform.location.x - ego_pos[0]) ** 2 + (
                                right_drive_waypoint.transform.location.y - ego_pos[
                            1]) ** 2) - right_drive_waypoint.lane_width / 2
                lane_array_global[str_lan_obj]["Dy"][0][1] = drive_waypoint.lane_width - lane_array_global[str_lan_obj]["Dy"][0][0]

        # right lane
        if left_drive_waypoint is not None and left_drive_waypoint.lane_type == carla.LaneType.Driving:
            # check if road has different direction basing on different lane id sign
            if np.sign(left_drive_waypoint.lane_id) != np.sign(drive_waypoint.lane_id):
                left_left_lane_marking = left_drive_waypoint.right_lane_marking
            else:
                left_left_lane_marking = left_drive_waypoint.left_lane_marking

            lane_array_global[str_lan_obj]["Type"][0][3] = map_lane_marking_type(left_left_lane_marking)
            lane_array_global[str_lan_obj]["Curvature"][0][3] = calcu_c_1(left_drive_waypoint, distance)
            lane_array_global[str_lan_obj]["CurvatureDx"][0][3] = calcu_c_derivative(left_drive_waypoint, distance)
            lane_array_global[str_lan_obj]["YawAngle"][0][3] = (np.double)(math.radians(get_angle_difference(ego_rot_360, left_drive_waypoint.transform.rotation.yaw)))
            lane_array_global[str_lan_obj]["Dy"][0][1] = math.sqrt((left_drive_waypoint.transform.location.x - ego_pos[0]) ** 2 + (left_drive_waypoint.transform.location.y - ego_pos[1]) ** 2) - left_drive_waypoint.lane_width / 2
            lane_array_global[str_lan_obj]["Dy"][0][0] = drive_waypoint.lane_width - lane_array_global[str_lan_obj]["Dy"][0][1]
            lane_array_global[str_lan_obj]["Dy"][0][3] = lane_array_global[str_lan_obj]["Dy"][0][1] + left_drive_waypoint.lane_width
        else:
            lane_array_global[str_lan_obj]["Type"][0][3] = -1
            if left_drive_waypoint is not None:
                lane_array_global[str_lan_obj]["Dy"][0][1] = math.sqrt(
                    (left_drive_waypoint.transform.location.x - ego_pos[0]) ** 2 + (
                                left_drive_waypoint.transform.location.y - ego_pos[
                            1]) ** 2) - left_drive_waypoint.lane_width / 2
                lane_array_global[str_lan_obj]["Dy"][0][0] = drive_waypoint.lane_width - lane_array_global[str_lan_obj]["Dy"][0][1]

        if right_drive_waypoint is None and left_drive_waypoint is None:
            road_base_vector = [[math.cos(math.radians(drive_waypoint.transform.rotation.yaw)), math.sin(drive_waypoint.transform.rotation.yaw)],
                                [math.sin(math.radians(drive_waypoint.transform.rotation.yaw)), -math.cos(math.radians(drive_waypoint.transform.rotation.yaw))]]
            lat_pos_respect_road_center = np.matmul(road_base_vector, ego_pos[:2] - np.array([drive_waypoint.transform.location.x, drive_waypoint.transform.location.y]))[1]      # [1] is the lateral axis (right hand rule)
            lane_array_global[str_lan_obj]["Dy"][0][0] = drive_waypoint.lane_width + lat_pos_respect_road_center
            lane_array_global[str_lan_obj]["Dy"][0][1] = drive_waypoint.lane_width - lat_pos_respect_road_center

        # current lane
        lane_array_global[str_lan_obj]["Type"][0][0] = map_lane_marking_type(right_lane_marking)
        lane_array_global[str_lan_obj]["Type"][0][1] = map_lane_marking_type(left_lane_marking)
        lane_array_global[str_lan_obj]["Curvature"][0][0] = calcu_c_1(drive_waypoint, distance)
        lane_array_global[str_lan_obj]["Curvature"][0][1] = lane_array_global[str_lan_obj]["Curvature"][0][0]
        lane_array_global[str_lan_obj]["CurvatureDx"][0][0] = calcu_c_derivative(drive_waypoint, distance)
        lane_array_global[str_lan_obj]["CurvatureDx"][0][1] = lane_array_global[str_lan_obj]["CurvatureDx"][0][0]
        lane_array_global[str_lan_obj]["YawAngle"][0][0] = (np.double)(math.radians(get_angle_difference(ego_rot_360, drive_waypoint.transform.rotation.yaw)))
        lane_array_global[str_lan_obj]["YawAngle"][0][1] = lane_array_global[str_lan_obj]["YawAngle"][0][0]

        return True

    def calcu_c_1(host_waypoint, route_distance):
        previous_waypoint = host_waypoint.previous(route_distance)[0]
        next_waypoint = host_waypoint.next(route_distance)[0]
        _transform = next_waypoint.transform
        _location, _rotation  = _transform.location, _transform.rotation
        x1, y1 = _location.x, _location.y
        yaw1 = _rotation.yaw

        _transform = previous_waypoint.transform
        _location, _rotation  = _transform.location, _transform.rotation
        x2, y2 = _location.x, _location.y
        yaw2 = _rotation.yaw

        c = 2*math.sin(math.radians((yaw1-yaw2)/2)) / math.sqrt((x1-x2)**2 + (y1-y2)**2)
        return c


    # def find_curvature_from_opendrive(s, road_id):
    #     opendrive = world_map.to_opendrive()
    #     bs_data = BeautifulSoup4(opendrive, 'xml')
    #     road = bs_data.find('road', {'id': road_id})
    #     piece_road = road.find('geometry', {'s'< s, 's' + 'length' > s})
    #     curvature = piece_road.find('arc').get('curvature')
    #     if curvature is None:
    #         curvature = 0
    #
    #     return  curvature


    def calcu_c_derivative(host_waypoint, route_distance):
        previous_waypoint = host_waypoint.previous(route_distance)[0]
        next_waypoint = host_waypoint.next(route_distance)[0]
        _transform = next_waypoint.transform
        _location, _rotation  = _transform.location, _transform.rotation
        x1, y1 = _location.x, _location.y
        yaw1 = _rotation.yaw

        _transform = host_waypoint.transform
        _location, _rotation = _transform.location, _transform.rotation
        x_centr, y_centr = _location.x, _location.y
        yaw_centr = _rotation.yaw

        _transform = previous_waypoint.transform
        _location, _rotation  = _transform.location, _transform.rotation
        x2, y2 = _location.x, _location.y
        yaw2 = _rotation.yaw

        c_dx = (math.radians(yaw1 - 2*yaw_centr + yaw2)) / (math.sqrt((x_centr-x2)**2 + (y_centr-y2)**2))**2
        return c_dx


    def fill_frame_content(ii, deltaTime, ego_light):

        d_ego.resize((ii + 1,))
        d_lane.resize((ii + 1,))
        d_objects.resize((ii + 1, ))
        d_positioning.resize((ii + 1, ))
        # Fill the datasets
        fileTime = ii*deltaTime
        utcTime = datetime_in_milliseconds + fileTime * 1000
        ego_array_global["UTCTime"][0] = utcTime
        obj_array_global["UTCTime"][0] = utcTime
        lane_array_global["UTCTime"][0] = utcTime
        position_array_global["UTCTime"][0] = utcTime
        ego_array_global["FileTime"][0] = fileTime
        obj_array_global["FileTime"][0] = fileTime
        lane_array_global["FileTime"][0] = fileTime
        position_array_global["FileTime"][0] = fileTime
        
        ego_array_global["DirectionIndicator"][0] = ego_light

        print(f'filling frame: {ii}')
        d_ego[ii] = ego_array_global
        d_lane[ii] = lane_array_global
        d_objects[ii] = obj_array_global
        d_positioning[ii] = position_array_global







    ###### This part of code is original for l3pilot, avoid to read if not necessary

    ######
    ###### START L3Pilot CODE


    def generate_attributes(dataset, struct_selection: str):
        """
        This method writes the description and the unit of the variables to the attributes of the datasets

        :param dataset: The dataset, to which the attributes are added
        :param struct_selection: String used for the selection of the correct attribute structure
        :return:
        """
        # Define the datatype for the attributes
        dtype = h5py.h5t.special_dtype(vlen=str)
        if struct_selection == str_ego:
            struct = ego_struct
        elif struct_selection == str_obj:
            struct = object_struct
        elif struct_selection == str_lan:
            struct = lane_struct
        elif struct_selection == str_pos:
            struct = pos_struct
        elif struct_selection == str_ed_weather:
            struct = weather_struct
        elif struct_selection == str_ed_map:
            struct = map_struct
        else:
            return dataset
        # Generate the attributes for the ego variables
        for variable in struct:
            dataset.attrs.create(variable.name, [("Description", variable.description), ("Unit", variable.unit)],
                                dtype=dtype)
        return dataset


    def generate_top_level_attributes(f):
        """
        This functions adds the top level attributes to the trip file
        The entries of the array should be filled accordingly

        :param f: The hdf5 file, to which the attributes are added
        :return: The file with the attached attributes
        """
        f.attrs["creation_script"] = os.path.basename(__file__)
        f.attrs["creation_date"] = datetime.datetime.now().strftime("%d-%b-%Y %H:%M:%S")
        f.attrs["author"] = __maintainer__
        f.attrs["comment"] = "example of a hdf5 file for the L3Pilot project".encode()
        f.attrs["institution"] = "Institut fuer Kraftfahrzeuge (ika)".encode()
        f.attrs["hdf5_version"] = str(h5py.version.hdf5_version).encode()
        f.attrs["h5py_version"] = str(h5py.version.version).encode()

        meta_array = np.array(np.zeros(1,), dtype=c_meta_data)
        meta_array["General"]["ADFVersion"][0] = 0.2
        meta_array["General"]["FormatVersion"][0] = cdf_version
        meta_array["General"]["Partner"][0] = "ika".encode()
        meta_array["General"]["RecordDate"][0] = datetime.datetime.now().strftime("%d-%b-%Y %H:%M:%S").encode()
        meta_array["General"]["UTCOffset"][0] = 2

        meta_array["Driver"]["DriverID"][0] = "42".encode()
        meta_array["Driver"]["DriverType"][0] = np.int8(2)

        meta_array["Car"]["DriveType"][0] = 4
        meta_array["Car"]["FuelType"][0] = 5
        meta_array["Car"]["NumberOfOccupants"][0] = 1
        meta_array["Car"]["PositionFrontBumper"][0] = 2.37
        meta_array["Car"]["PositionRearBumper"][0] = 1.37
        meta_array["Car"]["Transmission"][0] = np.int8(1)
        meta_array["Car"]["VehicleID"][0] = "42".encode()
        meta_array["Car"]["VehicleLength"][0] = 4.902
        meta_array["Car"]["VehicleWeight"][0] = 1515
        meta_array["Car"]["VehicleWidth"][0] = 2.090

        meta_array["Experiment"]["AnalysisEligible"][0] = np.int8(0)
        meta_array["Experiment"]["Baseline"][0] = np.int8(0)
        meta_array["Experiment"]["Country"][0] = "DE".encode()
        meta_array["Experiment"]["TestEndOdo"][0] = 42
        meta_array["Experiment"]["TestEndTime"][0] = time.time()
        meta_array["Experiment"]["TestSiteType"][0] = np.int8(3)
        meta_array["Experiment"]["TestStartOdo"][0] = 17
        meta_array["Experiment"]["TestStartTime"][0] = time.time() - 100
        meta_array["Experiment"]["TripID"][0] = "52074".encode()
        f.attrs.create("metaData", meta_array, (1, ), c_meta_data)
        return f


    ###### END CDF CODE
    ######


    # global variables definition

    # arrays filled each frame
    ego_array_global = np.array(np.zeros(1, ), dtype=c_ego)
    obj_array_global = np.array(np.zeros(1, ), dtype=c_objects)
    lane_array_global = np.array(np.zeros(1, ), dtype=c_lane)
    position_array_global = np.array(np.zeros(1, ), dtype=c_pos)

    # start filling arrays with default values
    ego_array_global[0] = c_ego_fill[0]
    obj_array_global[0] = c_objects_fill[0]
    lane_array_global[0] = c_lane_fill[0]
    position_array_global[0] = c_pos_fill[0]
    # default value for storing oldSpeed, what is better than '0'??
    global oldSpeed3D
    oldSpeed3D = np.array([0,0,0])

    acceleration3D = np.array([0,0,0])

    global ego_vel, ego_ang_vel
    global ego_pos, ego_rot 
    pos = []
    rot = []
    vel = []
    ang_vel = []
    dimensions_list = []

    deltaTime = 0.1     # in seconds 

    vehiclesID = [] #[179, 180, 181, 182, 183, 184, 185, 178, 186, 188, 187, 177]

    #first frame passed
    fv = False

    ####
    import carla
    import argparse

    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()

    # kill_old_instances(args.port)
    launch_carla_server(args.port, render=False)
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    #world = client.load_world('Town04')    # run this row if and only if desired map is not already charged in carla server
    world = client.get_world()
    world_map = world.get_map()

    ####### START L3PILOT CODE
    #######


    print("Generating an example file with version {0} of the L3Pilot common data format.".format(cdf_version))

    log_string = client.show_recorder_file_info(log_file, True).split('\n')


    # Open and initialize the file
    with h5py.File(raw_file, "w") as file:
        # Add some attributes to the file.
        file = generate_top_level_attributes(file)
        """ Here the datasets are created, that are then filled with carla data. The structures used here are defined in 
            the structure_definitions.py file. """

        # Create the ego dataset
        ego_space = h5py.h5s.create_simple((1, ), (h5py.h5s.UNLIMITED, ))
        ego_plist = h5py.h5p.create(h5py.h5p.DATASET_CREATE)
        ego_plist.set_chunk((500, ))
        ego_plist.set_deflate(9)
        ego_plist.set_fill_value(c_ego_fill)
        ego_type = h5py.h5t.py_create(c_ego, logical=1)
        d_ego_id = h5py.h5d.create(file.id, str_ego.encode(), ego_type, ego_space, ego_plist)
        d_ego = Dataset(d_ego_id)
        print("Created egoVehicle dataset with a size of {0} bytes per timestamp".format(c_ego.itemsize))
        d_ego = generate_attributes(d_ego, str_ego)

        # Create the lane dataset
        lane_space = h5py.h5s.create_simple((1, ), (h5py.h5s.UNLIMITED, ))
        lane_plist = h5py.h5p.create(h5py.h5p.DATASET_CREATE)
        lane_plist.set_chunk((500, ))
        lane_plist.set_deflate(9)
        lane_plist.set_fill_value(c_lane_fill)
        lane_type = h5py.h5t.py_create(c_lane, logical=1)
        d_lane_id = h5py.h5d.create(file.id, str_lan.encode(), lane_type, lane_space, lane_plist)
        d_lane = Dataset(d_lane_id)
        print("Created lanes dataset with a size of {0} bytes per timestamp".format(c_lane.itemsize))
        d_lane = generate_attributes(d_lane, str_lan)

        # Create the objects dataset
        objects_space = h5py.h5s.create_simple((1, ), (h5py.h5s.UNLIMITED, ))
        objects_plist = h5py.h5p.create(h5py.h5p.DATASET_CREATE)
        objects_plist.set_chunk((20, ))
        objects_plist.set_deflate(9)
        objects_plist.set_fill_value(c_objects_fill)
        objects_type = h5py.h5t.py_create(c_objects, logical=1)
        d_objects_id = h5py.h5d.create(file.id, str_obj.encode(), objects_type, objects_space, objects_plist)
        d_objects = Dataset(d_objects_id)
        print("Created objects dataset with a size of {0} bytes per timestamp".format(c_objects.itemsize))
        d_objects = generate_attributes(d_objects, str_obj)

        # Create the positioning dataset
        pos_space = h5py.h5s.create_simple((1, ), (h5py.h5s.UNLIMITED, ))
        pos_plist = h5py.h5p.create(h5py.h5p.DATASET_CREATE)
        pos_plist.set_chunk((1000, ))
        pos_plist.set_deflate(9)
        pos_plist.set_fill_value(c_pos_fill)
        pos_type = h5py.h5t.py_create(c_pos, logical=1)
        d_positioning_id = h5py.h5d.create(file.id, str_pos.encode(), pos_type, pos_space, pos_plist)
        d_positioning = Dataset(d_positioning_id)
        print("Created positioning dataset with a size of {0} bytes per timestamp".format(c_pos.itemsize))
        d_positioning = generate_attributes(d_positioning, str_pos)

        # Create the external data group
        # For demonstration purposes here!
        ext_group = h5py.h5g.create(file.id, str_ed.encode(), h5py.h5p.DEFAULT, h5py.h5p.DEFAULT)
        # # Add the weather dataset
        # weather_space = h5py.h5s.create_simple((1, ), (h5py.h5s.UNLIMITED, ))
        # weather_plist = h5py.h5p.create(h5py.h5p.DATASET_CREATE)
        # weather_plist.set_chunk((1000, ))
        # weather_plist.set_deflate(9)
        # weather_plist.set_fill_value(c_weather_fill)
        # weather_type = h5py.h5t.py_create(c_ext_weather, logical=1)
        # d_weather_id = h5py.h5d.create(ext_group, str_ed_weather.encode(), weather_type, weather_space, weather_plist)
        # d_weather = h5hl.Dataset(d_weather_id)
        # print("Created weather dataset with a size of {0} bytes per timestamp".format(c_ext_weather.itemsize))
        # d_weather = generate_attributes(d_weather, str_ed_weather)
        # Add the map dataset
        map_space = h5py.h5s.create_simple((1, ), (h5py.h5s.UNLIMITED, ))
        map_plist = h5py.h5p.create(h5py.h5p.DATASET_CREATE)
        map_plist.set_chunk((1000, ))
        map_plist.set_deflate(9)
        map_plist.set_fill_value(c_map_fill)
        map_type = h5py.h5t.py_create(c_ext_map, logical=1)
        d_map_id = h5py.h5d.create(ext_group, str_ed_map.encode(), map_type, map_space, map_plist)
        d_map = Dataset(d_map_id)
        print("Created map dataset with a size of {0} bytes per timestamp".format(c_ext_map.itemsize))
        d_map = generate_attributes(d_map, str_ed_map)


    ####### END L3PILOT CODE
    #######
        
        
        
        ## reading file line by line, in the first frame we can find vehicle IDs, vehicle dimensions
        
        # with open(log_file, 'r') as fp:
            
        check_bounding_boxes = False

        for line in log_string:
            # search string
            if line.startswith('Date:'):
                line_date = line[6:]
                datetime_str = line_date.split(' ')
                date = datetime_str[0].split('/')
                time = datetime_str[1].split(':')
                date_time_obj = datetime.datetime(int("20"+date[2]), int(date[0]), int(date[1]), int(time[0]), int(time[1]), int(time[2]))
                epoch = datetime.datetime.utcfromtimestamp(0)
                datetime_in_milliseconds = (date_time_obj - epoch).total_seconds() * 1000.0
            elif line.startswith(' Actor bounding boxes'):
                check_bounding_boxes = True
            elif check_bounding_boxes:
                m = re.search('  Id: (.+?) ', line)                # find ID
                if(m == None):
                    check_bounding_boxes = False
                elif int(m.group(1)) in vehiclesID:
                    save_dimensions_single_vehicle(line)
            
            if 'Create' in line and 'vehicle' in line:
                m = re.search('Create (.+?): vehicle', line)
                vehiclesID.append((int)(m.group(1)))
                prev_line = line
            elif 'role_name = hero' in line:
                m = re.search('Create (.+?): vehicle', prev_line)
                if(m):
                    ego_ID = (int)(m.group(1))
                    print(ego_ID)
            if ((line.startswith('Frame')) and (not('Frame 1 ' in line))):
                break
            
            
        ## understand frequency of the log file
        timeValues=[None,None]
        for line in log_string:
            # search string        
            if (line.startswith('Frame')) and (not('Frame 3 ' in line)):
                m = re.search('Frame (.+?) at (.+?) seconds', line)
                if(m):
                    timeValues[(int)(m.group(1))-1]=(float)(m.group(2))
                    if timeValues[1] != None:
                        period = timeValues[1]-timeValues[0]
                        break
        


        if period == None:
            print('Error: period not found')
            exit()
        
        #convert to 10 Hz
        #do it for different values of period
        skip = 0
        deltaTime = 0.1
        #10 Hz
        if period >= deltaTime-0.02 and period <= deltaTime+0.02:
            skip = 0
        #20 Hz
        if period >= deltaTime/2-0.02 and period <= deltaTime/2+0.02:
            skip = 1                
                    
        ## reading file line by line, to keep trace of what data we are storing (e.g. velocity, position,...) we use boolean flags
        
        # with open(log_file, 'r') as fp:

        check_ego_light = False
        check_ego_animations = False
        check_dynamic = False
        save_positions = False
        light_animation = False

        n_frame = 0
        n_vehicle = 0
        ego_light = 0
        read_lines = True

        for line in log_string:
            #skip to keep 10 Hz
            if (line.startswith('Frame') and skip!=0):
                #20 Hz
                m = re.search('Frame (.+?) at (.+?) seconds', line)
                if(m and (int)(m.group(1))>1):
                    fv = True
                    print((int)(m.group(1)))
                if(m):
                    if(((int)(m.group(1))-1)%2==0):
                        read_lines = True
                    else:
                        read_lines = False
            if fv and 'Create' in line and 'vehicle' in line:
                m = re.search('Create (.+?): vehicle', line)
                #add vehicle to the list            
                vehiclesID.append((int)(m.group(1)))
            if fv and 'extension' in line:  
                dimensions = re.search('extension: \((.+?)\)', line).group(1)
                dimensions_list.append(np.array([np.double(d)/50 for d in dimensions.split(", ")])) 
                
            if fv and 'Destroy' in line:
                m = re.search(' Destroy (\d+)', line)
                if((int)(m.group(1)) in vehiclesID):
                    vehiclesID.remove((int)(m.group(1)))   
                    dimensions_list.pop()                           
            if not read_lines:
                continue    
        
            if check_dynamic:
                m = re.search('Id: (.+?) ', line)
                if (m == None):                                              # i.e. section is ended, because no Id found
                    check_dynamic = False
                    save_relative_values(acceleration3D, pos, rot, vel, ang_vel, ego_pos, ego_rot, vehiclesID)                    # at the end of dynamic actors section we have all the data to process relative values
                    # ego data (available at this point) are also used to store lane line and geolocation info
                    save_geolocation()
                    save_lane_lines()

                    # reset lists for next frame
                    pos = []
                    rot = []
                    vel = []
                    ang_vel = []
                else:
                    save_absolute_velocity(line, (int)(m.group(1)), deltaTime)                                # storing temporary data which will be used to compute relative values at the end                 
            elif check_ego_animations:
                if ('Id: '+ str(ego_ID)) in line:
                    save_animations(line)
                    check_ego_animations = False
            elif save_positions:
                m = re.search('  Id: (.+?) ', line)                # find ID
                if(m == None):
                    save_positions = False
                elif((int)(m.group(1)) in vehiclesID):              # check if the found id is in vehiclelist -> save positions
                    save_absolute_positions(line, (int)(m.group(1)))  
            elif light_animation:  
                if "Id: " + str(ego_ID) in line:
                    if "LeftBlinker" in line:
                        ego_light = 1
                    elif "RightBlinker" in line:
                        ego_light = 2
                    else:
                        ego_light = 0
                    light_animation = False
                    
            # basing the line content, setup the right flag to store data
            # when 'Frame' keyword has found, a new Frame is starting and so the previous one must be stored. The first 'Frame' keyword appearance should be skipped because no "previous frame" exists
            
            if ((line.startswith('Frame')) and (not('Frame 1 ' in line))):    
                fill_frame_content(n_frame, deltaTime, ego_light)
                #reset the values for the next frame
                obj_array_global = np.array(np.zeros(1, ), dtype=c_objects)
                obj_array_global[0] = c_objects_fill[0]
                n_frame += 1
            elif line.startswith(' Dynamic actors:'):
                check_dynamic = True
            elif line.startswith(' Vehicle animations:'):
                check_ego_animations = True
            elif line.startswith(' Positions:'):
                save_positions = True
            elif line.startswith(' Vehicle light animations'):
                light_animation = True
            #elif line.startswith(' Destroy'):
            #    break
            


kill_old_instances(args.port, stop_execution=True)