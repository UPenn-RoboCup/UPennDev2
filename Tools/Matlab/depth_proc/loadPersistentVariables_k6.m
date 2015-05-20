%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% parameters 
param_normalComputation = [13 3]; % 1: (odd nember)^2+1 <- window sizw,  2: increment  
thre_sValue = 0.07; % The smaller it is, the flatter the plane fit is 
thre_clusterSize = 1000; % number of clusters
thre_memberSize = 1000; % number of connected members (in the image domain)
param_meanShiftResol = 0.55;% 0.6;         % mean shift resolution
param_meanShiftWeights = [0 1]; %[0.2 1];   % mean shift weights (1:image distance, 2:angular distance in normal space) 

prevNormals = [];
manualModel = [];


