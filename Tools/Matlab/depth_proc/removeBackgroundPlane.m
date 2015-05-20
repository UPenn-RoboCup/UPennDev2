function  [Planes, metadata, OutOfPlanePoints] = removeBackgroundPlane(data, meta, ui)
% v6: corner of walls 

persistent Xind_c
persistent Yind_c
persistent MASK

DEPTH_MAX = 2000; %8000;
DEPTH_MIN = 400;

if isempty(Xind_c)
    load XYind_c_depth.mat
    load MASK2.mat
end

if ~isempty(meta) && isfield(meta,'tr')    
    [Rot, tr] = TransKinectToBody(meta);
   % T = reshape(meta.tr,4,4)';
   % Rot = T(1:3,1:3);
   % tr = T(1:3,4);
else
    Rot = eye(3);
    tr = zeros(3,1);
    meta.name = 'depth';
  
end

if ui.undistortDepth == 1
    data = undistort_depth(data); 
end

% if strcmp(char(meta.name),'depth')
data = flip(double(data)',2);
data(data(:) <= DEPTH_MIN) = 0;
data(data(:) >= DEPTH_MAX) = 0;   
data = MASK.*data;
[ Planes, nPlanes, PlaneOfInterest ] = detectPlaneInstances_kinect_v6(data,Rot,tr,ui);
metadata = struct('PlaneOfInterest',PlaneOfInterest,'numPlanes',nPlanes);

OutOfPlanePoints = [];
if numel(Planes) > 0
   
    valid = find(data > 0)';
    
    X = data(valid)*0.001;
    Y = -0.7091*Xind_c(valid).*X; % Sx = 0.7091
    Z = -0.5555*Yind_c(valid).*X; % Sy = 0.5555
   
    All = Rot*[X; Y; Z] + repmat(tr,1,length(X(:)));
    
    % check the distance from the first (largest) wall
    
    a = Planes{1}.Normal'*Planes{1}.Center;
    d = Planes{1}.Normal'*All -a;
    
    OutOfPlane = abs(d) > 0.05;
    OutOfPlanePoints = valid(OutOfPlane);
    temp = zeros(size(data));
    temp(OutOfPlanePoints) = 1;
    figure(9),
  %  showPointCloud(All(1,OutOfPlane), All(2,OutOfPlane), All(3,OutOfPlane));
    imagesc(temp);
     
    horAcc = sum(temp,2);
    figure(10), plot(horAcc,1:424,'.-');
    set(gca,'YDir','reverse');
    
    % for groups near peaks
    % weighted ransac for shelf detection 
    % findShelf(w,points);
    % wait, would it be useful? because the robot would be too close..
    
end
    
% elseif strcmp(char(meta.name),'lidar') % handles lidar here?? 
% Planes = detectPlaneInstances_lidar(data,param,3);
% end

end

function [est, inliers] = findShelf(w,points)

    N = size(points,2);
    Nite = 100; 
    thre = 0.03;
    
    inliers = [];
    Ninlier = 0;
    est = [];
    
    w_ = find(w > 100);    
    if numel(w_) < 0.05*N % 5 percent 
        return;
    end
    
    for k=1:Nite
        idx = randperm(length(w_),3)
        H = points(:,w_(idx));
        
        [u,~] = svd([H; ones(1,3)]);
        a = u(1:3,4);
        a = a/norm(a);
        c = mean(H,2);
        
        d = abs(a*points - c);
        in = find(d < thre);
        in_ = sum(in,2);
        if in_ > Ninlier
           Ninlier = in_;
           inliers = in;
           est = [a; c];
        end
    end

end