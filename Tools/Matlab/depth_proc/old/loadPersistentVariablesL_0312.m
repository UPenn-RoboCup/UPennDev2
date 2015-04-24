ONESCAN_ = resetParam.dims(2);    % single scan resolution 
NUMSCAN_ = resetParam.dims(1);     % number of scans (in horizontal direction)

Ccb_prev = eye(3);
Tcb_prev = zeros(3,1);

v_angles = resetParam.rfov(1):(0.25*pi/180):resetParam.rfov(2);   
if length(v_angles) > ONESCAN_
    v_angles = v_angles(1:ONESCAN_);
end

s_angles = resetParam.a;

