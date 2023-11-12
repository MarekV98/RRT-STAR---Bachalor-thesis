function move_joint_r(obj,joint_num,angle)
    [~,tgt_q,~] = refresh_status(obj);
    tgt_q(joint_num) = tgt_q(joint_num)+angle;
    obj.set_joint(tgt_q);
end