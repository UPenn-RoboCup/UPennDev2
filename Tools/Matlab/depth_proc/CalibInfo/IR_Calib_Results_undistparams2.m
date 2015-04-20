% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/
% IMCX = 255.8;%422487561914693 ;% DEPTH_W/2;
% IMCY = 203.7 %.487139940005989;% DEPTH_H/2;
% fx =  364.7% 457362485643273; % DEPTH_W/2/tan(70.6/2*pi/180);
% fy =  366.1% 4.542810626989194; % DEPTH_H/2/tan(60/2*pi/180);

%-- Focal length:
fc = [ 364.7 ; 366.1 ];

%-- Principal point:
cc = [ 255.8 ; 203.7 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
% kc = [ 0.098069182739161 ; -0.249308515140031 ; 0.000500420465085 ; 0.000529487524259 ; 0.000000000000000 ];
kc = [ 0.08708 ; -0.16515 ; -0.00321 ; -0.00345; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.569282671152671 ; 1.461154863082004 ];

%-- Principal point uncertainty:
cc_error = [ 2.286222691982841 ; 1.902443125481905 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.012730833002324 ; 0.038827084194026 ; 0.001933599829770 ; 0.002380503971426 ; 0.000000000000000 ];

%-- Image size:
nx = 512;
ny = 424;

KK = [fc(1) alpha_c*fc(1) cc(1);0 fc(2) cc(2) ; 0 0 1];

%%% Compute the new KK matrix to fit as much data in the image (in order to
%%% accomodate large distortions:
r2_extreme = (nx^2/(4*fc(1)^2) + ny^2/(4*fc(2)^2));
dist_amount = 1; %(1+kc(1)*r2_extreme + kc(2)*r2_extreme^2);
fc_new = dist_amount * fc;

KK_new = [fc_new(1) alpha_c*fc_new(1) cc(1);0 fc_new(2) cc(2) ; 0 0 1];

save KK_depth2.mat fc cc kc alpha_c KK_new