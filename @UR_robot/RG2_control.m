function RG2_control(obj,wide,force,payload,setpayload,depthcomp)
%Todo : zkontrolovat nastavení TCP
%TODO : fingertips


if strcmp(obj.s2.status,'closed')  % if the port is not open, then open it
    fopen(obj.s2);
end

            
%% Start program
  fprintf(obj.s2,'def myprog():\n');
  
%% Gripper functions initialization
            fprintf(obj.s2,' set_tool_output_mode(0)\n');
            fprintf(obj.s2,' set_tool_digital_output_mode(0,1)\n');
            fprintf(obj.s2,' set_tool_digital_output_mode(1,1)\n');
            fprintf(obj.s2,'global measure_width=0\n');
            fprintf(obj.s2,'global grip_detected=False\n');
            fprintf(obj.s2,'global lost_grip=False\n');
            fprintf(obj.s2,'global zsysx=0\n');
            fprintf(obj.s2,'global zsysy=0\n');
            fprintf(obj.s2,'global zsysz=0.06935\n');
            fprintf(obj.s2,'global zsysm=0.7415\n');
            fprintf(obj.s2,'global zmasx=0\n');
            fprintf(obj.s2,'global zmasy=-0\n');
            fprintf(obj.s2,'global zmasz=0.18659\n');
            fprintf(obj.s2,'global zslax=0\n');
            fprintf(obj.s2,'global zslay=0\n');
            fprintf(obj.s2,'global zslaz=0\n');
            fprintf(obj.s2,'global zmasm=0\n');
            fprintf(obj.s2,'global zslam=0\n');
            fprintf(obj.s2,'global zslatcp=p[0,0,0,0,0,0]\n');
            fprintf(obj.s2,'global zmastcp=p[0,0,0.18659,0,-0,-3.14159]\n');
            fprintf(obj.s2,' thread lost_grip_thread():\n');
            fprintf(obj.s2,' while True:\n');
            fprintf(obj.s2,' set_tool_voltage(24)\n');
            fprintf(obj.s2,' if True ==get_digital_in(9):\n');
            fprintf(obj.s2,' sleep(0.024)\n');
            fprintf(obj.s2,' if True == grip_detected:\n');
            fprintf(obj.s2,' if False == get_digital_in(8):\n');
            fprintf(obj.s2,' grip_detected=False\n');
            fprintf(obj.s2,' lost_grip=True\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' set_tool_analog_input_domain(0, 1)\n');
            fprintf(obj.s2,' set_tool_analog_input_domain(1, 1)\n');
            fprintf(obj.s2,' zscale = (get_analog_in(2)-0.026)/2.5900013\n');
            fprintf(obj.s2,' zangle = zscale*1.57079633+-0.08726646\n');
            fprintf(obj.s2,' zwidth = 5.0+110*sin(zangle)\n');
            fprintf(obj.s2,' global measure_width = (floor(zwidth*10))/10-9.2\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' sync()\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,'lg_thr = run lost_grip_thread()\n');
            fprintf(obj.s2,' def RG2(target_width=110, target_force=40, payload=0.0, set_payload=False, depth_compensation=False, slave=False):\n');
            fprintf(obj.s2,' set_tcp(p[0,0,0.18659,0,-0,-3.14159])\n');
            fprintf(obj.s2,' grip_detected=False\n');
            fprintf(obj.s2,' if slave:\n');
            fprintf(obj.s2,' slave_grip_detected=False\n');
            fprintf(obj.s2,' else:\n');
            fprintf(obj.s2,' master_grip_detected=False\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' timeout = 0\n');
            fprintf(obj.s2,' timeout_limit = 3000000\n');
            fprintf(obj.s2,' while get_digital_in(9) == False:\n');
            fprintf(obj.s2,' if timeout > timeout_limit:\n');
            fprintf(obj.s2,' break\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' timeout = timeout+1\n');
            fprintf(obj.s2,' sync()\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' def bit(input):\n');
            fprintf(obj.s2,' msb=65536\n');
            fprintf(obj.s2,' local i=0\n');
            fprintf(obj.s2,' local output=0\n');
            fprintf(obj.s2,' while i<17:\n');
            fprintf(obj.s2,' set_digital_out(8,True)\n');
            fprintf(obj.s2,' if input>=msb:\n');
            fprintf(obj.s2,' input=input-msb\n');
            fprintf(obj.s2,' set_digital_out(9,False)\n');
            fprintf(obj.s2,' else:\n');
            fprintf(obj.s2,' set_digital_out(9,True)\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' if get_digital_in(8):\n');
            fprintf(obj.s2,' out=1\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' sync()\n');
            fprintf(obj.s2,' set_digital_out(8,False)\n');
            fprintf(obj.s2,' sync()\n');
            fprintf(obj.s2,' input=input*2\n');
            fprintf(obj.s2,' output=output*2\n');
            fprintf(obj.s2,' i=i+1\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' return output\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' target_width=target_width+9.2\n');
            fprintf(obj.s2,' if target_force>40:\n');
            fprintf(obj.s2,' target_force=40\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' if target_force<3:\n');
            fprintf(obj.s2,' target_force=3\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' if target_width>110:\n');
            fprintf(obj.s2,' target_width=110\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' if target_width<0:\n');
            fprintf(obj.s2,' target_width=0\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' rg_data=floor(target_width)*4\n');
            fprintf(obj.s2,' rg_data=rg_data+floor(target_force/2)*4*111\n');
            fprintf(obj.s2,' rg_data=rg_data+32768\n');
            fprintf(obj.s2,' if slave:\n');
            fprintf(obj.s2,' rg_data=rg_data+16384\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' bit(rg_data)\n');
            fprintf(obj.s2,' if slave==False:\n');
            fprintf(obj.s2,' t_w_rg=pose_trans(get_actual_tool_flange_pose(), zmastcp)\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' if slave:\n');
            fprintf(obj.s2,' t_w_rg=pose_trans(get_actual_tool_flange_pose(), zslatcp)\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' t_rg_w=pose_inv(t_w_rg)\n');
            fprintf(obj.s2,' if depth_compensation:\n');
            fprintf(obj.s2,' finger_length = 55.0/1000\n');
            fprintf(obj.s2,' finger_heigth_disp = 5.0/1000\n');
            fprintf(obj.s2,' center_displacement = 7.5/1000\n');
            fprintf(obj.s2,' start_pose = get_forward_kin()\n');
            fprintf(obj.s2,' set_analog_inputrange(2, 1)\n');
            fprintf(obj.s2,' zscale = (get_analog_in(2)-0.026)/2.5900013\n');
            fprintf(obj.s2,' zangle = zscale*1.57079633+-0.08726646\n');
            fprintf(obj.s2,' zwidth = 5.0+110*sin(zangle)\n');
            fprintf(obj.s2,' start_depth = cos(zangle)*finger_length\n');
            fprintf(obj.s2,' sleep(0.016)\n');
            fprintf(obj.s2,' timeout = 0\n');
            fprintf(obj.s2,' while get_digital_in(9) == True:\n');
            fprintf(obj.s2,' timeout=timeout+1\n');
            fprintf(obj.s2,' sleep(0.008)\n');
            fprintf(obj.s2,' if timeout > 20:\n');
            fprintf(obj.s2,' break\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' timeout = 0\n');
            fprintf(obj.s2,' timeout_limit = 750000\n');
            fprintf(obj.s2,' compensation_depth = 0\n');
            fprintf(obj.s2,' while get_digital_in(9) == False:\n');
            fprintf(obj.s2,' zscale = (get_analog_in(2)-0.026)/2.5900013\n');
            fprintf(obj.s2,' zangle = zscale*1.57079633+-0.08726646\n');
            fprintf(obj.s2,' zwidth = 5.0+110*sin(zangle)\n');
            fprintf(obj.s2,' measure_depth = cos(zangle)*finger_length\n');
            fprintf(obj.s2,' compensation_depth = (measure_depth - start_depth)\n');
            fprintf(obj.s2,' target_pose =pose_add(start_pose,pose_trans(pose_trans(t_w_rg, p[0,0,-compensation_depth,0,0,0]),t_rg_w))\n');
            fprintf(obj.s2,' if timeout > timeout_limit:\n');
            fprintf(obj.s2,' break\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' timeout=timeout+1\n');
            fprintf(obj.s2,' servoj(get_inverse_kin(target_pose),0,0,0.008,0.01,2000)\n');
            fprintf(obj.s2,' if point_dist(target_pose, get_forward_kin()) > 0.005:\n');
            fprintf(obj.s2,' popup("Lower grasping force or max width",title="RG-lag threshold exceeded", warning=False, error=False, blocking=False)\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' act_comp_pose = p[0,0,0,0,0,0]\n');
            fprintf(obj.s2,' while norm(act_comp_pose) < norm(compensation_depth)-0.0002:\n');
            fprintf(obj.s2,' servoj(get_inverse_kin(target_pose),0,0,0.008,0.01,2000)\n');
            fprintf(obj.s2,' act_comp_pose = pose_trans(pose_inv(start_pose),get_forward_kin())\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' stopj(2)\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' if depth_compensation==False:\n');
            fprintf(obj.s2,' timeout = 0\n');
            fprintf(obj.s2,' timeout_count=20*0.008/0.002\n');
            fprintf(obj.s2,' while get_digital_in(9) == True:\n');
            fprintf(obj.s2,' timeout = timeout+1\n');
            fprintf(obj.s2,' sync()\n');
            fprintf(obj.s2,' if timeout > timeout_count:\n');
            fprintf(obj.s2,' break\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' timeout = 0\n');
            fprintf(obj.s2,' timeout_limit = 3000000\n');
            fprintf(obj.s2,' while get_digital_in(9) == False:\n');
            fprintf(obj.s2,' timeout = timeout+1\n');
            fprintf(obj.s2,' sync()\n');
            fprintf(obj.s2,' if timeout > timeout_limit:\n');
            fprintf(obj.s2,' break\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' sleep(0.024)\n');
            fprintf(obj.s2,' if set_payload:\n');
            fprintf(obj.s2,' if slave:\n');
            fprintf(obj.s2,' if get_analog_in(3)/0.5180003 < 1.42:\n');
            fprintf(obj.s2,' zslam=0\n');
            fprintf(obj.s2,' else:\n');
            fprintf(obj.s2,' zslam=payload\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' else:\n');
            fprintf(obj.s2,' if get_digital_in(8) == False:\n');
            fprintf(obj.s2,' zmasm=0\n');
            fprintf(obj.s2,' else:\n');
            fprintf(obj.s2,' zmasm=payload\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' zload=zmasm+zslam+zsysm\n');
            fprintf(obj.s2,' set_payload(zload,[(zsysx*zsysm+zmasx*zmasm+zslax*zslam)/zload,(zsysy*zsysm+zmasy*zmasm+zslay*zslam)/zload,(zsysz*zsysm+zmasz*zmasm+zslaz*zslam)/zload])\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' master_grip_detected=False\n');
            fprintf(obj.s2,' master_lost_grip=False\n');
            fprintf(obj.s2,' slave_grip_detected=False\n');
            fprintf(obj.s2,' slave_lost_grip=False\n');
            fprintf(obj.s2,' if True == get_digital_in(8):\n');
            fprintf(obj.s2,' master_grip_detected=True\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' if get_analog_in(3)/0.5180003>1.97:\n');
            fprintf(obj.s2,' slave_grip_detected=True\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' grip_detected=False\n');
            fprintf(obj.s2,' lost_grip=False\n');
            fprintf(obj.s2,' if True == get_digital_in(8):\n');
            fprintf(obj.s2,' grip_detected=True\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' zscale = (get_analog_in(2)-0.026)/2.5900013\n');
            fprintf(obj.s2,' zangle = zscale*1.57079633+-0.08726646\n');
            fprintf(obj.s2,' zwidth = 5.0+110*sin(zangle)\n');
            fprintf(obj.s2,' global measure_width = (floor(zwidth*10))/10-9.2\n');
            fprintf(obj.s2,' if slave:\n');
            fprintf(obj.s2,' slave_measure_width=measure_width\n');
            fprintf(obj.s2,' else:\n');
            fprintf(obj.s2,' master_measure_width=measure_width\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,' return grip_detected\n');
            fprintf(obj.s2,' end\n');
            fprintf(obj.s2,'set_tool_voltage(24)\n');
    
%% Gripper command
            if setpayload==true
                setpayload ="True";
            elseif setpayload==false
                setpayload="False";
            else
                error('Not a valid argument')
            end
            if depthcomp==true
                depthcomp ="True";
            elseif depthcomp==false
                depthcomp="False";
            else
                error('Not a valid argument')
            end

            
            
            cmd = sprintf("RG2(" + '%f,%f,%f,%s,%s,False)\n',...
                     wide,force,payload,setpayload,depthcomp);
            
            fprintf(obj.s2,cmd);


%% End of the program
fprintf(obj.s2,'end\n');

end

