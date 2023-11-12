function [sim_pose] = set_config_sim(obj,config)
    robot = obj.model;
    current_config = obj.sim_config;
    waypointTimes = [0 4];
    if all(abs(config-current_config)<(30/180*pi))
        ts = 1;
    else
        ts = 0.2;
    end
    trajTimes = 0:ts:waypointTimes(end);
    waypointAccelTimes = diff(waypointTimes)/4;
    numJoints = 6;
    jointWaypoints = [current_config,config];
    [q,qd,qdd] = trapveltraj(jointWaypoints,numel(trajTimes), ...
            'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
            'EndTime',repmat(diff(waypointTimes),[numJoints 1]));
    for idx = 1:numel(trajTimes)  
        config = q(:,idx);
        show(robot,config,'Frames','off','PreservePlot',false);
        drawnow   
    end
    if obj.model.NumBodies == 7
    transform = getTransform(robot,config,'wrist3_link');
    elseif obj.model.NumBodies == 8
    transform = getTransform(robot,config,'gripper');   
    end
    
    R = transform(1:3,1:3);
    eulXYZ = rotm2eul(R,'XYZ');
    sim_pose = [transform(1:3,4);eulXYZ(1);eulXYZ(2);eulXYZ(3)];
end