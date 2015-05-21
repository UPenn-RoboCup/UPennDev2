function  [Planes, outdata] = detectPlanes7(data, meta, ui)
% v7: rough terrain (cinder blocks) 

if ~isempty(meta)   
     if isfield(meta,'tr')    
        [Rot, tr] = TransKinectToBody(meta);
    elseif isfield(meta,'tfL16')
       [Rot, tr] = TransKinectToBody_dale(meta);
    end
else
    Rot = eye(3);
    tr = zeros(3,1);
    meta.name = 'depth'
  
end

if ui.undistortDepth == 1
    data = undistort_depth(data); 
end

% if strcmp(char(meta.name),'depth')
[ Planes, nPlanes, PlaneOfInterest ] = detectPlaneInstances_kinect_v7(data,Rot,tr,ui);
outdata = [];

% test if planes are within expected values
if nPlanes > 0
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