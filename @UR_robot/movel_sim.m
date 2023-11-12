function [waypoint_config] = movel_sim(obj, pose)
    robot = obj.model;
    current_config = obj.sim_config;
    if obj.model.NumBodies == 7
    start = getTransform(robot,current_config,'wrist3_link');
    elseif obj.model.NumBodies == 8
    start = getTransform(robot,current_config,'gripper');    
    end
    waypoints = [start(1:3,4),pose(1:3)];
    R = start(1:3,1:3);
    eulXYZ = rotm2eul(R,'XYZ');
    orientations = [eulXYZ(1),pose(4);eulXYZ(2),pose(5);eulXYZ(3),pose(6)];
    waypointTimes = [0 4];
    dist = sqrt((pose(1)-start(1,4))^2+(pose(2)-start(2,4))^2+(pose(3)-start(3,4))^2);
    if dist < 0.1
        ts = 1;
    else
        ts = 0.2;
    end
    trajTimes = 0:ts:waypointTimes(end);
    waypointAccelTimes = 1;
    ik = inverseKinematics('RigidBodyTree',robot);
    ikWeights = [1 1 1 1 1 1];
    ikInitGuess = current_config;
    R0 = eul2quat(orientations(:,1)','XYZ');
    Rf = eul2quat(orientations(:,2)','XYZ');
    timeInterval = waypointTimes;
    trajTimes = timeInterval(1):ts:timeInterval(2);
    [q,qd,qdd] = trapveltraj(waypoints,numel(trajTimes), ...
                'AccelTime',waypointAccelTimes(1), ... 
                'EndTime',diff(waypointTimes));
    [R, omega, alpha] = rottraj(R0, Rf, timeInterval, trajTimes);
    for idx = 1:numel(trajTimes) 
        tgtPose = trvec2tform(q(:,idx)') * quat2tform(R(:,idx)');
        if obj.model.NumBodies == 7
    	[config,info] = ik('wrist3_link',tgtPose,ikWeights,ikInitGuess);
        elseif obj.model.NumBodies == 8
        [config,info] = ik('gripper',tgtPose,ikWeights,ikInitGuess);    
        end
        show(robot,config,'Frames','off','PreservePlot',false);
        drawnow    
        if any((abs(abs(config)-2*pi))<0.05)
            disp('Joint near limit!');
            break;
        end
        ikInitGuess=config;
    end
    waypoint_config = config;
end