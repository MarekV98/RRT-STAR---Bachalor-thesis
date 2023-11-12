function program_running = refresh_program_status(obj)

    if strcmp(obj.s3.status,'closed')  % if the port is not open, then open it
        fopen(obj.s3);
        for i=1:1  %the fisrt frame is robot_message instead of robot_state_message, which should be discarded.
            readasync(obj.s3);
            fread(obj.s3);
        end
    end

    readasync(obj.s3);
    msg = fread(obj.s3);
    
    if  size(msg,1)<4
        warning('s3 connection error, reconnecting');
        fclose(obj.s3);
        LAN_init(obj,obj.ip_UR);
        program_running = obj.refresh_program_status;  
        return;
    end
    len = msg(3)*256+msg(4);
    if len ~= length(msg)
        warning('s3 data error, rereading');
        program_running = obj.refresh_program_status;
        return;
    end    
    
    program_running = msg(24);    
end