function [waypoint_config] = movej_sim(obj, waypoint)
    robot = obj.model;
    current_config = obj.sim_config;
    current_pose = obj.sim_pose;
    dist = sqrt((waypoint(1)-current_pose(1))^2+(waypoint(2)-current_pose(2))^2+(waypoint(3)-current_pose(3))^2);
    if dist < 0.1
        ts = 1;
    else
        ts = 0.2;
    end
    waypointTimes = [0 4];
    trajTimes = 0:ts:waypointTimes(end);
    waypointAccelTimes = 1;
    ik = inverseKinematics('RigidBodyTree',robot);
    ikWeights = [1 1 1 1 1 1];
    ikInitGuess = current_config;
    jointWaypoints = zeros(6,2);
    jointWaypoints(:,1) = current_config;
    tgtPose = trvec2tform(waypoint(1:3)') * eul2tform(waypoint(4:6)','XYZ');
    if obj.model.NumBodies == 7
    [config,info] = ik('wrist3_link',tgtPose,ikWeights,ikInitGuess);
    elseif obj.model.NumBodies == 8
    [config,info] = ik('gripper',tgtPose,ikWeights,ikInitGuess);   
    end
    
    for i = 1:6
       if (config(i)-current_config(i))<(-pi)
          config(i) = config(i)+2*pi;
          if config(i) > (2*pi)
             config(i) = config(i)-2*pi;
          end
       elseif (config(i)-current_config(i))>(pi)
           config(i) = config(i)-2*pi;
          if config(i) < (-2*pi)
             config(i) = config(i)+2*pi;
          end
       end
    end
    jointWaypoints(:,2) = config';
    [q,qd,qdd] = trapveltraj(jointWaypoints,numel(trajTimes), ...
            'AccelTime',repmat(waypointAccelTimes,[6 1]), ... 
            'EndTime',repmat(diff(waypointTimes),[6 1]));
    for idx = 1:numel(trajTimes)  
        config = q(:,idx);
        show(robot,config,'Frames','off','PreservePlot',false);
        drawnow   
    end
    waypoint_config = config;
end