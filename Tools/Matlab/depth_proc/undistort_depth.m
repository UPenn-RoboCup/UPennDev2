%%% INPUT THE IMAGE FILE NAME:
function I2 = undistort_depth(I)
load('./CalibInfo/KK_depth2.mat');

[I2] = rectify(I,eye(3),fc,cc,kc,KK_new);
end
            
           