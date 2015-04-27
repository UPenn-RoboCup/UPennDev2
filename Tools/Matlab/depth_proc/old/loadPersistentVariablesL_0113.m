ONESCAN_ = resetParam.dims(2);    % single scan resolution 
NUMSCAN_ = resetParam.dims(1);     % number of scans (in horizontal direction)

Ccb_prev = eye(3);
Tcb_prev = zeros(3,1);

v_angles = resetParam.rfov(1):(0.25*pi/180):resetParam.rfov(2);   
if length(v_angles) > ONESCAN_
    v_angles = v_angles(1:ONESCAN_);
end

s_angles = resetParam.a;

normalComp_param = [5 1]; %  (w^2 + 1) half-window size
thre_svalue = 0.01; % The smaller it is, the flatter the plane fit is 
thre_clusterSize = 500; % number of clusters
thre_memberSize = 300; % number of connected members (in the image domain)
ms_resol = 0.70;% 0.6;         % mean shift resolution
ms_weights = [0 1]; %[0.2 1];   % mean shift weights (1:image distance, 2:angular distance in normal space) 


