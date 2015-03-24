function [Rot, tr] = TransKinectToBody(meta)

persistent motor_offsets
persistent yang_offset
persistent Cb_init
persistent P_kinect_m2
persistent P_m2_b

if isempty(motor_offsets)
    motor_offsets = [0 0.02];
    yang_offset = 0;
    Cb_init = eulr2dcm([ yang_offset 0 0]');
    P_kinect_m2 = Cb_init*[0.015, 0.015, 0.08]';    
    P_m2_b = [0.0 0.0 1.25]';
end

% transformation to the body (foot) coordinate     
Cb_m1 = eulr2dcm([ 0 0 (meta.head_angles(1)-motor_offsets(1))]');
Cm1_m2 = eulr2dcm([0 (meta.head_angles(2)-motor_offsets(2)) 0]');

Cn_b = eulr2dcm([meta.imu_rpy(1:2)'; 0]);
Rot = Cb_init*Cm1_m2*Cb_m1*Cn_b;
tr = Cb_m1'*P_kinect_m2 + P_m2_b;  

Rot = Rot';
end