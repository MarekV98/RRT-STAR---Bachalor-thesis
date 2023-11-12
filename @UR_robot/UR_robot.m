classdef UR_robot<handle
    %% public properties
    properties
       user = 'student';                            % admin/student 
       mode;                                        % robot/sim
       password;
       model;
       sim_pose = [-0.8172;-0.2329;0.0628;pi/2;0;0];% for simulation
       sim_config = [0;0;0;0;0;0];                  % for simulation
       sim_freedrive;
       a_joint =  0.35;                             % joint acceleration limit
       a_tool  =  0.05 ;                            % tcp acceleration limit
       v_joint =  5/180*pi;                         % joint velocity limit
       v_tool  =  0.05;                             % tcp velocity limit
       r       =  0;                                % blend radius for move instructions
       ip_UR = '10.1.1.2';                          % default ip/hostname of the robot
       tcp_data;                                    % tcp data set information
       n_tcp_data = 1;                              % number of active tcp data set
       target_pose;                                 % target tcp pose
       collision = 'off';                           % collision detection on/off
       gripper_fingertips = 4.6;                    % gripper fingertips width
       gripper = 'off'
       wait_mode = 'on'                             % Matlab waits until the instruction is executed
    end
    %% private properties
    properties (SetAccess = private, GetAccess = public)
        s1        % handles of port1 (dashborad 29999 for power control)
        s2        % handles of port2 (realtime interface 30003 for motion/force control and tcp setup) 
        s3        % handles of port3 (primary interface 30001, for robot moving status acquisition, i.e. in teachmode or not)

        pose      % current tcp pose
        config    % current joint position
        force     % current tcp force
        active_tcp   %active tcp data
        freedrive_status % boolean, is in teachmode or not 
        gripper_pose % current gripper wideness
    end
%% common functions
    methods
        % constructor
        function obj = UR_robot(mode, opt)
            arguments
                mode
                opt.user = 'student'
                opt.password = []
                opt.ip = '10.1.1.2'
            end

            model = load('ur5e_robot_6dof');
            obj.model = model.robot;
            obj.ip_UR = opt.ip;

            if strcmp(opt.user,'admin')
                if isempty(opt.password)
                    disp('No password')
                    obj.user = 'student';
                elseif strcmp(opt.password,'KPivuKlobaskaKVeceruProchazka')
                    obj.password = opt.password;
                    obj.user = 'admin';
                else
                    disp('Incorrect password');
                    obj.user = 'student';
                end
            else
                obj.user = 'student';
            end


            if strcmp(mode,'robot') || strcmp(mode,'sim')
                obj.mode = mode;
            else
                disp('Invalid mode, choose robot/sim');
            end
        end
        
        function movel(obj,pose)
            if strcmp(obj.mode,'sim')  
                if sqrt(pose(1)^2+pose(2)^2+pose(3)^2)>0.9
                    disp('Target is out of bounds');
                    return;
                end
                obj.sim_config = movel_sim(obj,pose);
                obj.sim_pose = pose;
            elseif strcmp(obj.mode,'robot')
                obj.set_pose(pose,'cart');
            end
        end
        
        function movej(obj,pose) 
            if strcmp(obj.mode,'sim')  
                if sqrt(pose(1)^2+pose(2)^2+pose(3)^2)>0.9
                    disp('Target is out of bounds');
                    return;
                end
                obj.sim_config = movej_sim(obj,pose);
                obj.sim_pose = pose;
            elseif strcmp(obj.mode,'robot')
                obj.set_pose(pose,'joint');
            end
        end

        function movep(obj,pose)
            if strcmp(obj.mode,'sim')  
                if sqrt(pose(1)^2+pose(2)^2+pose(3)^2)>0.9
                    disp('Target is out of bounds');
                    return;
                end
                obj.sim_config = movel_sim(obj,pose);
                obj.sim_pose = pose;
            elseif strcmp(obj.mode,'robot')
                obj.set_pose(pose,'cart_p');
            end
        end
        
        function movel_tcp(obj,offset)
            if strcmp(obj.mode,'sim')
                pose = obj.sim_pose + offset;
                obj.sim_config = movel_sim(obj,pose);
                obj.sim_pose = pose;
            elseif strcmp(obj.mode,'robot')
                obj.move_tcp(offset,'cart');
            end
        end
        
        function movej_tcp(obj,offset)
            if strcmp(obj.mode,'sim')
                pose = obj.sim_pose + offset;
                obj.sim_config = movej_sim(obj,pose);
                obj.sim_pose = pose;
            elseif strcmp(obj.mode,'robot')
                obj.move_tcp(offset,'joint');
            end
        end
        
        function move_joint(obj,joint_num,angle)
            if strcmp(obj.mode,'sim')
                config = obj.sim_config;
                config(joint_num) = config(joint_num)+(angle/180*pi);
                if any(config<-2*pi)||any(config>2*pi)
                    disp('Incorrect angle, range is <-2pi;¨2pi>');
                    return;
                end
                obj.sim_pose = set_config_sim(obj,config);
                obj.sim_config = config;
            elseif strcmp(obj.mode,'robot')
                obj.move_joint_r(joint_num,angle);
            end
        end
        
        function set_config(obj,config)
            if any(config<-2*pi)|any(config>2*pi)
               disp('Incorrect angle, range is <-2pi;2pi>');
               return;
            end
            if strcmp(obj.mode,'sim')
                obj.sim_pose = set_config_sim(obj,config);
                obj.sim_config = config;
            elseif strcmp(obj.mode,'robot')
                obj.set_joint(config);
            end
        end
        
        function freedrive_on(obj)
            if strcmp(obj.mode,'sim')
                obj.sim_freedrive = interactiveRigidBodyTree(obj.model,'Frames','off');
            elseif strcmp(obj.mode,'robot')
                obj.freedrive_on_r();
            end
        end
        
        function freedrive_off(obj)
            if strcmp(obj.mode,'sim')
                obj.sim_config = obj.sim_freedrive.Configuration;
                transform = obj.sim_freedrive.MarkerBodyPose;
                R = transform(1:3,1:3);
                eulXYZ = rotm2eul(R,'XYZ');
                obj.sim_pose = [transform(1,4);transform(2,4);transform(3,4);eulXYZ(1);eulXYZ(2);eulXYZ(3)];
                clear obj.sim_freedrive;
            elseif strcmp(obj.mode,'robot')
                obj.freedrive_off_r();
            end
        end
        
        function add_gripper(obj)
           if strcmp(obj.mode,'sim')
                model = load('ur5e_robot_6dof_gripper_closed');
                obj.model = model.robot;
           elseif strcmp(obj.mode,'robot')
               model = load('ur5e_robot_6dof_gripper_closed');
               obj.model = model.robot;
           end
           disp('Gripper added to model');
        end
        
        function clear_gripper(obj)
           if strcmp(obj.mode,'sim')
               model = load('ur5e_robot_6dof');
               obj.model = model.robot;
           elseif strcmp(obj.mode,'robot')
               model = load('ur5e_robot_6dof');
               obj.model = model.robot;
           end
           disp('Gripper removed from model');
        end
        
        function open_gripper(obj)
           if strcmp(obj.mode,'sim')
               model = load('ur5e_robot_6dof_gripper_open');
               obj.model = model.robot;
           elseif strcmp(obj.mode,'robot')
               model = load('ur5e_robot_6dof_gripper_open');
               obj.model = model.robot;
               if strcmp(obj.s1.status,'closed') % if the port is not open, then open it
                    fopen(obj.s1);
                    disp(fscanf(obj.s1));
               end
               disp(query(obj.s1,'load gripper_open_matlab.urp'));     
               disp(query(obj.s1,'play')); 
               pause(0.5);
           end
           disp('Gripper model open');
        end
        
        function close_gripper(obj)
           if strcmp(obj.mode,'sim')
                model = load('ur5e_robot_6dof_gripper_closed');
                obj.model = model.robot;
           elseif strcmp(obj.mode,'robot')
               model = load('ur5e_robot_6dof_gripper_closed');
               obj.model = model.robot;
               if strcmp(obj.s1.status,'closed') % if the port is not open, then open it
                    fopen(obj.s1);
                    disp(fscanf(obj.s1));
               end
               disp(query(obj.s1,'load gripper_closed_matlab.urp'));     
               disp(query(obj.s1,'play')); 
               pause(0.5);
           end
           disp('Gripper model closed');
        end
        
        function half_gripper(obj)
           if strcmp(obj.mode,'sim')
                model = load('ur5e_robot_6dof_gripper');
                obj.model = model.robot;
           elseif strcmp(obj.mode,'robot')
               model = load('ur5e_robot_6dof_gripper');
               obj.model = model.robot;
               if strcmp(obj.s1.status,'closed') % if the port is not open, then open it
                    fopen(obj.s1);
                    disp(fscanf(obj.s1));
               end
               disp(query(obj.s1,'load gripper_half_matlab.urp'));     
               disp(query(obj.s1,'play')); 
               pause(0.5);
           end
           disp('Gripper model half opened');
        end
        
        function RG2(obj,wide,force,payload,setpayload,depthcomp)
           if strcmp(obj.mode,'sim')
                disp("No simulation mode")
           elseif strcmp(obj.mode,'robot')
               RG2_control(obj,wide,force,payload,setpayload,depthcomp);
               pause(2.5);
           end
        end

        function zero_force(obj)
            if strcmp(obj.mode,'sim')
                disp("No simulation mode")
            elseif strcmp(obj.mode,'robot')
                fprintf(obj.s2,'zero_ftsensor()\n');
            end
        end
        
        function apply_force(obj,selection_vector,wrench,limits)
           if strcmp(obj.mode,'sim')
                disp("No simulation mode")
           elseif strcmp(obj.mode,'robot')
               fprintf(obj.s2,'def myprog():\n');
                force_mode(obj,selection_vector,wrench,limits)
                fprintf(obj.s2,'sleep(600)\n');
               fprintf(obj.s2,'end\n');
               pause(0.2)
           end
        end
        
        function force_movel(obj,selection_vector,wrench,limits,tgt_pose)
           if strcmp(obj.mode,'sim')
                disp("No simulation mode")
           elseif strcmp(obj.mode,'robot')
               current_wait_mode = obj.wait_mode;
               obj.wait_mode = 'off';
               fprintf(obj.s2,'def force_movel():\n');
               force_mode(obj,selection_vector,wrench,limits);
               obj.set_pose(tgt_pose,'cart');
               fprintf(obj.s2,'end\n');
               obj.wait_mode = current_wait_mode;
               if strcmp(obj.wait_mode,'on')
                   program_waiting(obj)
               end
               pause(0.2)
           end
        end
               
    end  
    
%% robot function definitions
    methods
        
        LAN_init(obj,varargin);  %Initialize the LAN connection

        tcp_data_init(obj);  % Initialize and upload the tcp data to the robot

        set_active_tcp(obj); % Upload the active tcp data to the robot

        power_on(obj);  % power on the robot and release the brake

        power_off(obj);   % power of the robot
        
        unlock(obj);    % unlock protective stop
        
        freedrive_on_r(obj); % activate teachmode
        
        freedrive_off_r(obj); % end teachmode
        
        [pose,config,force] = refresh_status(obj);  % read current tcp pose, joint position and tcp force
        
        freedrive_status = refresh_freedrive_status(obj); % query the robot status: is in teachmode(1) or not(0)

        cmd = move_tcp(obj,offset,path_type) % move tcp in tcp coordinate frame

        cmd = set_pose(obj,tgt_pose,path_type,axis_no_rotate); % move tcp to a pose defined in base coordinate frame
        
        cmd = set_joint(obj,tgt_q); % move the joint to a give joint position

        cmd = stop(obj); % stop moving
        
        cmd = move_joint_r(obj, joint_num, angle); % rotate desired joint
                
        resume(obj,varargin)% resume the stopped movement 
        
        Rxyz = R2Rxyz(obj,R);  % convert rotation matrix to rotation vector 
        
        R = Rxyz2R(obj,Rxyz);  % convert rotation vector to rotation matrix
        
        %simulation animation
        [sim_pose] = set_config_sim(obj,config);
        
        [waypoint_config] = movej_sim(obj, waypoint);
        
        [waypoint_config] = movel_sim(obj, pose);
        
        
        
        program_running = refresh_program_status(obj);
    end
    
%% set & get
    methods
         % when pose,q,force and freedrive_status are queried, they are updated from robot
         
        function pose = get.pose(obj)
        	[pose,~,~] = obj.refresh_status;
        end
        
        function config = get.config(obj)  
            [~,config,~] = obj.refresh_status;
        end
        
        function force = get.force(obj)    
            [~,~,force] = obj.refresh_status;
        end

        function freedrive_status = get.freedrive_status(obj)  
           if strcmp(obj.mode,'robot')
                freedrive_status = obj.refresh_freedrive_status;
           elseif strcmp(obj.mode,'sim')
               	disp('Freedrive_status only available in robot mode');
           end

        end
        
        function gripper_pose = get.gripper_pose(obj)
            if strcmp(obj.mode,'robot')
                if strcmp(obj.gripper,'on')
                    gripper_pose = obj.refresh_gripper_status;
                else
                    gripper_pose = [];
                end
            elseif strcmp(obj.mode,'sim')
                disp('Gripper_pose only available in robot mode');
            end
        end
        
        function set.gripper(obj,str)
           if strcmp(obj.mode,'robot')
               if strcmp(str,'on')
                obj.n_tcp_data = 2;

               else
                obj.n_tcp_data = 1;

               end
                set_active_tcp(obj);
                obj.gripper = str;
           elseif strcmp(obj.mode,'sim')
               	disp('only available in robot mode');
           end
        end
        
        % when n_tcp_data is assigned, the correponding tcp data is uploaded to the robot
        function set.n_tcp_data(obj,n_tcp_data)      
           if strcmp(obj.mode,'robot')
                obj.n_tcp_data = n_tcp_data;
                set_active_tcp(obj);
           elseif strcmp(obj.mode,'sim')
               	disp('n_tcp_data only available in robot mode');
           end
        end
        
        % when active_tcp is queried, it will be refreshed according to current active tcp number
        function active_tcp = get.active_tcp(obj)
           if strcmp(obj.mode,'robot')
                active_tcp = obj.tcp_data(obj.n_tcp_data);
           elseif strcmp(obj.mode,'sim')
               	disp('active_tcp only available in robot mode');
           end
        end
        
        % when tcp_data is changed, the change will be also sent to the robot
        function set.tcp_data(obj,tcp_data) 
           if strcmp(obj.mode,'robot')
                obj.tcp_data = tcp_data;
                set_active_tcp(obj);
           elseif strcmp(obj.mode,'sim')
               	disp('tcp_data only available in robot mode');
           end
        end
        
        function set.v_tool(obj,v_tool)         % speed limit for students
            if strcmp(obj.user,'student')
                if v_tool > 0.05
                   v_tool = 0.05; 
                end
            end
            obj.v_tool = v_tool;
        end
        
        function set.a_tool(obj,a_tool)         % acceleration limit for students
            if strcmp(obj.user,'student')
                if a_tool > 0.05
                   a_tool = 0.05; 
                end
            end
            obj.a_tool = a_tool;
        end
        
        function set.v_joint(obj,v_joint)       % speed limit for students
            if strcmp(obj.user,'student')
                if v_joint > 10/180*pi
                   v_joint = 10/180*pi; 
                end
            end
            obj.v_joint = v_joint;
        end
        
        function set.a_joint(obj,a_joint)       % acceleration limit for students
            if strcmp(obj.user,'student')
                if a_joint > 0.35
                   a_joint = 0.35; 
                end
            end
            obj.a_joint = a_joint;
        end
        
        function set.user(obj, user)            % user verification
            if strcmp(user, 'admin')
                if strcmp(obj.password,'KPivuKlobaskaKVeceruProchazka')
                   obj.user = 'admin';
                   disp('User set to admin');
                else
                    disp('Incorrect password');
                    disp('User set to student');
                    obj.user = 'student';
                end
            else
                obj.user = 'student';
                disp('User set to student');
            end
        end
        
        function set.mode(obj, mode)
            if strcmp(mode,'sim')
               obj.mode = 'sim'; 
               disp('Mode set to simulation');
            elseif strcmp(mode,'robot')
                obj.mode = 'robot';
                LAN_init(obj);
                tcp_data_init(obj);
                obj.target_pose = obj.pose;
                disp('Mode set to robot');
            else
                disp('Invalid mode');
            end
        end
        
        function set.collision(obj,collision)
            if strcmp(collision,'off')
               if strcmp(obj.user,'admin')
                   obj.collision = 'off';
                   disp('Collision detection turned off');
               else
                   disp('Switch to admin mode to turn off collision detection');
               end
            else
                obj.collision = 'on';
                disp('Collision detection turned on');
            end
        end
        
    end
end