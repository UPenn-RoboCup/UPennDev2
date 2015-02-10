DEPTH_W = 512;
DEPTH_H = 424;
DEPTH_MAX = 4500; %8000;
DEPTH_MIN = 400;
IMCX = DEPTH_W/2;
IMCY = DEPTH_H/2;
fx =  DEPTH_W/2/tan(70.6/2*pi/180);
fy =  DEPTH_H/2/tan(60/2*pi/180);
Sx = IMCX/fx;
Sy = IMCY/fy;

Xind = kron(1:DEPTH_H,ones(1,DEPTH_W)); 
Yind = repmat(1:DEPTH_W,1,DEPTH_H); % index in 1D array 

Xind_ = (Xind/IMCX-1); % index in 2D array representation
Yind_ = (Yind/IMCY-1); 

Xind_c = (reshape(Xind,DEPTH_W,DEPTH_H)-IMCX)/IMCX; % index in 2D array representation
Yind_c = (reshape(Yind,DEPTH_W,DEPTH_H)-IMCY)/IMCY; 

% kinect default pose info
%angles__ = [10 0 0];
%T_kh = eye(4);% kinect to head joint 
tr_kinect2head = zeros(3,1); % [-0.03; 0.06; -0.08];

% parameters 
param_normalComputation = [5 5]; % 1: (odd nember)^2+1 <- window sizw,  2: increment  
thre_sValue = 0.02; % The smaller it is, the flatter the plane fit is 
thre_clusterSize = 50; % number of clusters
thre_memberSize = 30; % number of connected members (in the image domain)
param_meanShiftResol = 0.6;% 0.6;         % mean shift resolution
param_meanShiftWeights = [0 1]; %[0.2 1];   % mean shift weights (1:image distance, 2:angular distance in normal space) 


