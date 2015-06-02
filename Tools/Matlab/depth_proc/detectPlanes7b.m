function  [Planes, outdata] = detectPlanes7b(data, meta, ui)
% v7: rough terrain (cinder blocks) 
persistent prevdata
persistent count
persistent MASK 
persistent list_planes

if isempty(MASK)
    load MASK2.mat
end

if isempty(list_planes)
    list_planes = struct('Center',[],'Normal',[],'var',0,'N',0);
end

Planes = [];
outdata = [];

DEPTH_MAX = 2500; %8000;
DEPTH_MIN = 400;
data = flip(double(data)',2);%-20;
data(data(:) <= DEPTH_MIN) = 0;
data(data(:) >= DEPTH_MAX) = 0;   

% moving average 
if isempty(prevdata) || count == 0 %|| ui.reset == 1
    prevdata = data;
    count = 1;
    list_planes = struct('Center',[],'Normal',[],'var',0,'N',0);
    return;
else
    % test difference 
    if ~isempty(prevdata)
        mad = mean(abs(data(:) - prevdata(:))) % mean absolute difference
        if mad < 100 %small enough % indoor setting
            
            count = count + 1;
               
            % moving average of the image
            prevdata = (1-1/count)*prevdata + 1/count*data;  
        else
            count = 0;
            return;
        end
    end
end

% transformation
if ~isempty(meta)   
     if isfield(meta,'tr')    
        [Rot, tr] = TransKinectToBody(meta);
    elseif isfield(meta,'tfL16')
       [Rot, tr] = TransKinectToBody_dale(meta);
    end
else
    Rot = eye(3);
    tr = zeros(3,1);
    meta.name = 'depth';  
end

if ui.undistortDepth == 1
    data = undistort_depth(data); 
end

% median filtering
data_ = medfilt2(prevdata,[7 7]);
data_ = MASK.*data_;

% detec planes
[ Planes, nPlanes, PlaneOfInterest ] = detectPlaneInstances_kinect_v7(data_,Rot,tr,ui);
outdata = [];

% track planes
if count > 5
    
    idx = 1:6;
    if nPlanes > 6
        D = zeros(1,nPlanes);
        for k=1:nPlanes
           D(k) = norm(Planes{k}.Center(1:2));         
        end
        [~,idx] = sort(D,'ascend'); % two closest ones
    end
    
    if isempty(list_planes)
        for k=1:min(nPlanes,6)
            k_ = idx(k);
            list_planes.Center(:,k) = Planes{k_}.Center;
            list_planes.Normal(:,k) = Planes{k_}.Normal;
            list_planes.var(k) = 0.04;
            list_planes.N = list_planes.N + 1;
        end
    else
        N_ = min(nPlanes,6)
        match = zeros(1,N_);
        for k=1:N_
            k_ = idx(k);
            
            dst = sum((repmat(Planes{k_}.Center,1,list_planes.N) - list_planes.Center).^2,1); % distance
            ang = abs(Planes{k_}.Normal'*list_planes.Normal); % angular distance
            
            [min_d, min_idx] = min(dst);
            % if the closest one is close enough in Euclidean & angular distance 
            if min_d < 0.05^2 && ang(min_idx) > 0.95       % should tune the thresholds         
                if match(k_) == 0
                    match(k_) = min_idx;
                else
                    match(k_) = 0; % not unnique match! something must be wrong, so set it 0
                end
            end           
        end
        
        for k=1:N_
            lidx = match(k);
            if lidx > 0
                list_planes.Center(:,lidx) 
            end
        end
    end   
    
end

% test if planes are within expected values
if 0% nPlanes > 0
    if nPlanes == 1
        if norm(Planes{1}.Center(1:2)) < 1 && abs(Planes{1}.Normal(3)) > 0.9 && Planes{1}.Size > 500 % ? 
            outdata = struct('cen',Planes{1}.Center,'bnd',Planes{1}.Points,'n',Planes{1}.Normal);
        end
    else
        D = zeros(1,nPlanes);
        for k=1:nPlanes
           D(k) = norm(Planes{k}.Center(1:2));         
        end
        [~,idx] = sort(D,'ascend'); % two closest ones
        if Planes{idx(1)}.Center(1) < Planes{idx(2)}.Center        
            outdata.cen = [Planes{idx(1)}.Center Planes{idx(2)}.Center]; 
            outdata.bnd = [Planes{idx(1)}.Points Planes{idx(2)}.Points]; 
            outdata.n = [Planes{idx(1)}.Normal Planes{idx(2)}.Normal];        
        else
            outdata.cen = [Planes{idx(2)}.Center Planes{idx(1)}.Center]; 
            outdata.bnd = [Planes{idx(2)}.Points Planes{idx(1)}.Points]; 
            outdata.n = [Planes{idx(2)}.Normal Planes{idx(1)}.Normal];   
        end
    end
    
end

end