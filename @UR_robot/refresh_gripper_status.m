function zwidth = refresh_gripper_status(obj)

    if strcmp(obj.s3.status,'closed')  % if the port is not open, then open it
        fopen(obj.s3);
        for i=1:1  %the fisrt frame is robot_message instead of robot_state_message, which should be discarded.
            readasync(obj.s3);
            fread(obj.s3);
        end
    end


%% read data from realtime interface 30001

    readasync(obj.s3);
    fread(obj.s3,539,'int8');               % useless
    toolAnalog1 = fread(obj.s3,1,'double');
    fread(obj.s3,169,'int8');               % useless

%% Crazy caltulation
    zscale = (toolAnalog1-0.026)/2.5900013;
    zangle = zscale*1.57079633-0.08726646;
    zwidth = 5.0+110*sin(zangle) - 2 * obj.gripper_fingertips;

