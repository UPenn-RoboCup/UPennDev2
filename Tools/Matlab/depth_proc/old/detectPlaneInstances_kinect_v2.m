function [ Planes, nPlanes, PlaneOfInterest ] = detectPlaneInstances_kinect_v2( depthRaw, params, visflag )

persistent DEPTH_W       % Width
persistent DEPTH_H       % Height
persistent DEPTH_MAX     % Maximum valid depth value
persistent DEPTH_MIN     % Minimum valid depth value
persistent IMCX          % Image center x  
persistent IMCY          % Image center y  
persistent fx            % focal length x
persistent fy            % focal length y
persistent Sx            % IMCX/fx
persistent Sy            % IMCY/fy
persistent Xind
persistent Yind
persistent Xind_
persistent Yind_
persistent Xind_c
persistent Yind_c 
persistent Ccb_prev
persistent Tcb_prev
persistent tr_kinect2head

%if isempty(DEPTH_W),
    loadPersistentVariables_0112b;
%end

%depthRaw = (depthRaw );

% params{1} : Transformation Matrix
Ccb = eye(3);
Tcb = zeros(3,1);
if isempty(Ccb_prev)
    Ccb_prev = Ccb;
    Tcb_prev = Tcb;
end
if ~isempty(params)
    Ccb = params{1};
    if sum(size(Ccb) ~= [3 3])
        Ccb = eye(3);
    end
    Tcb = params{2};
    if length(Tcb) ~= 3
        Tcb = zeros(3,1);
    end
end

Tcb = Tcb + Ccb*tr_kinect2head;

% if params == 0
Planes =[]; 
Points3D = [];
Indices = [];
% Type: Ground, Wall, Table, Manual, Else 
                        
PlaneID = 0;
nPlanes = 0;
PlaneOfInterest = 0;

    
% parameters 
normalComp_param = [5 5]; % 1: (odd nember)^2+1 <- window sizw,  2: increment  
thre_svalue = 0.02; % The smaller it is, the flatter the plane fit is 
thre_clusterSize = 50; % number of clusters
thre_memberSize = 30; % number of connected members (in the image domain)
ms_resol = 0.6;% 0.6;         % mean shift resolution
ms_weights = [0 1]; %[0.2 1];   % mean shift weights (1:image distance, 2:angular distance in normal space) 


%% Filtering     
% Initialize mask
mask = ones(DEPTH_W, DEPTH_H);
mask(depthRaw(:) <= DEPTH_MIN) = 0;
mask(depthRaw(:) >= DEPTH_MAX) = 0;    
% Filter depth
depth = depthRaw.*mask;    
depth = medfilt2(depth,[7 7]);
validInd = find(mask>0);

 % inverse depth
ZZ = double(1000./depth);

%% Normal Computation
[A, S, V] = mexFitPlane_uv1d( Xind_c, Yind_c, ZZ, logical(mask),normalComp_param);
validNormal = find( V > 0); 
validNormal = validNormal(find(S(4,validNormal)< thre_svalue));

%% Clustering 
 
data = [  Xind_(validNormal) ; Yind_(validNormal)];

% works OK
% [finalMean,clusterXYcell,nMembers] = sphericalMeanShiftxy(data,A(1:3,validNormal),0.45,[0 1]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% generate initial mean information HERE for better starting 
[finalMean,clusterXYcell,nMembers] = sphericalMeanShiftxyB(data,A(1:3,validNormal),ms_resol,ms_weights);

% for each cluster
blankConnImg = zeros(floor(DEPTH_W/normalComp_param(2)),floor(DEPTH_H /normalComp_param(2)));
for tt = 1: size(finalMean,2)      
    if nMembers(tt) > thre_clusterSize  % if cluster size is big enough
        
        connImg = blankConnImg;
        index = validNormal(clusterXYcell{tt}); 
        [index_x, index_y] = ind2sub([DEPTH_W DEPTH_H],index);
        index_xsub = floor(index_x / normalComp_param(2));
        index_ysub = floor(index_y / normalComp_param(2));
        index_ = sub2ind(floor([DEPTH_W DEPTH_H]/normalComp_param(2)),index_xsub, index_ysub);
        connImg(index_) = 1;
       %% Connectivity Check 
        % Connected-component analysis 
        % check an error related to "eqlabel"
        [conn, ids, count_, indices ] = test4Connectivity(connImg);
        
        if ~isempty(ids)
            % offsets = []; % for merging?          
            % c0 = [];
            for t = 1: length(count_)                 
                if count_(t) > thre_memberSize % if the connected bloc is big enough 
                    [dummy,whichcell] = intersect(index_ , indices{ids(t)});    
                    if ~isempty(whichcell)   
                     %% Find center, bbox, boundary
                        [yind_s, xind_s] = ind2sub(size(conn),indices{ids(t)});
                        center_s = round(mean([xind_s;yind_s],2));
                       
                        Pts = [];
                        bbox = getBoundingBox(yind_s,xind_s);
                        Bbox = zeros(3,size(bbox,1));
                        [dummy,whichcell__] = intersect(index_ , sub2ind(size(conn), bbox(:,1), bbox(:,2)));   
                        Bbox(1,:) = depth(index(whichcell__))*0.001;
                        Bbox(2,:) = -Yind_c(index(whichcell__))*Sy.*Bbox(1,:);
                        Bbox(3,:) = -Xind_c(index(whichcell__))*Sx.*Bbox(1,:);
                        
                        % 8-directional extreme points 
                        pts = find8ExtremePoints(conn, center_s, ids(t));
                        if ~isempty(pts)                               
                            [dummy,whichcell_] = intersect(index_ , sub2ind(size(conn), pts(:,1), pts(:,2)));  
                          
                            Pts(1,:) = depth(index(whichcell_))*0.001;
                            Pts(2,:) = -Yind_c(index(whichcell_))*Sy.*Pts(1,:);
                            Pts(3,:) = -Xind_c(index(whichcell_))*Sx.*Pts(1,:);                                                                              
                        end       
                        
                     %% refinement 
                        % (could test using svd and find the principal axes?) 
                        [c, ins] = estimatePlane_useall( Xind_c(index(whichcell)), Yind_c(index(whichcell)), ZZ(index(whichcell)));
                         c(1) = c(1)/Sx;
                         c(2) = c(2)/Sy;
                        %% save output 
                        if ~isempty(c) && numel(ins) > 5
                            n_ = c([2 1 3]);
                            n_ = -n_/norm(n_);
                            n__ = [n_(3) -n_(1) -n_(2)]/norm(n_);
                           
                            z_ = depth(index(whichcell))*0.001;
                            z_mean = mean(z_);                                  
                            x_mean = mean((Xind(index(whichcell))-IMCX)/fx.*z_);
                            y_mean = mean((Yind(index(whichcell))-IMCY)/fy.*z_);
                                                                          
                            Center = [z_mean; -y_mean; -x_mean];
 
                            PlaneID = PlaneID + 1;
                            Planes{PlaneID} = struct('Center',Center,...
                                                     'Normal', n__',...
                                                     'Points',[Pts Bbox],...
                                                     'Size', numel(ins),...
                                                     'Type','Else');   
                                                 
                            if strcmp(params{3}.mode, 'manual_2d') 
                                Indices{PlaneID} = [Xind(index(whichcell)); Yind(index(whichcell))];
                            elseif strcmp(params{3}.mode, 'manual_3d')                                 
                                Points3D{PlaneID} = [z_;  -(Yind(index(whichcell))-IMCY)/fy.*z_; -(Xind(index(whichcell))-IMCX)/fx.*z_];                   
                            end
                        end
                    end
                end
            end
        end
        
    end % end of for each cluster
end
    
     % visualization
    if visflag
       % disp('vis');
       % Tcb
        Ztemp = depth(validInd(10:10:end))*0.001;
        Ytemp = -Yind_c(validInd(10:10:end))*Sy.*Ztemp;
        Xtemp = -Xind_c(validInd(10:10:end))*Sx.*Ztemp;
        Xvis = Ccb*[Ztemp(:)'; Ytemp(:)'; Xtemp(:)'] + repmat(Tcb,1,length(Ztemp(:)));
        figure(visflag), [az,el] = view; hold off; 
        scatter3( Xvis(1,:),Xvis(2,:), Xvis(3,:), 2 ,[0.5 0.5 0.5] ,'filled'); axis equal; hold on;
        set(gca,'XDir','reverse');
        xlabel('x');
        ylabel('y');
        zlabel('z','Rotation',0);        
        axis([0 1.5 -1 1 0 1.5]);
        view(az,el);

       % figure(101), hold off;
       % imagesc(depth'); axis equal;

    end
    
Ns = zeros(3,PlaneID);
Cs = zeros(3,PlaneID);
Szs = zeros(1,PlaneID);

 % Coordinate Transformation
if ~isempty(params)  
    for t = 1:PlaneID  
        Planes{t}.Center = Ccb*Planes{t}.Center + Tcb;
        Planes{t}.Points = Ccb*Planes{t}.Points + repmat(Tcb,1,size(Planes{t}.Points,2)) ;
        Planes{t}.Normal = Ccb*Planes{t}.Normal;
        if params{3}.mode > 0 
            Ns(:,t) = Planes{t}.Normal;
            Cs(:,t) = Planes{t}.Center;
        end
        if 0 % visflag
            randcolor = rand(1,3); % 0.5*(finalMean(3:5,tt)+1);   
            figure(visflag), 
            scatter3(Planes{t}.Points(1,:), Planes{t}.Points(2,:), Planes{t}.Points(3,:),15,randcolor,'filled');
            nvec = [Planes{t}.Center  Planes{t}.Center+Planes{t}.Normal*0.2];
            figure(visflag),
            plot3(nvec(1,:), nvec(2,:), nvec(3,:),'-', 'Color', randcolor, 'LineWidth',2);
        end
    end
end
  
if strcmp(params{3}.mode,'ground')
    % testGround();
    n_inner = 1-abs([0 0 1]*Ns);
    flag = (n_inner < 0.02);
    flag = flag & (Cs(3,:) < 0.03);
    % pick the largest
    candidates = find(flag>0);
    if ~isempty(candidates)
        [val, idx] = max(Szs(candidates));
        Planes{candidates(idx)}.Type = 'ground';
        PlaneOfInterest = idx;
    end
elseif strcmp(params{3}.mode, 'table')  
    ; % testTable();   
elseif strcmp(params{3}.mode, 'manual_2d')
    minval = 10000;
    min_idx = 0;
    for t=1:PlaneID 
        dsq =  (Indices{t} - repmat(params{3}.data',1,length(Indices{t}))).^2;
        [val] = min(sum(dsq,1));
        if val < minval 
            if min_idx > 0 
                Planes{min_idx}.Type = 'Else';
            end
            Planes{t}.Type = 'Manual';
            PlaneOfInterest = t;   
            min_idx = t;
            minval = val;
        end
    end
elseif strcmp(params{3}.mode, 'manual_3d')
    % Find closest plane center (Assuming not ground)
    minval = 10000;
    min_idx = 0;
    for t=1:PlaneID
        dsq = ( Ccb*Points3D{t} + repmat(Tcb,1,length(Points3D{t})) - repmat((params{3}.data'),1,size(Points3D{t},2))).^2;
        [val] = min(sum(dsq,1));
        if val < minval 
            if min_idx > 0 
                Planes{min_idx}.Type = 'Else';
            end
            Planes{t}.Type = 'Manual';
            PlaneOfInterest = t;   
            min_idx = t;
            minval = val;
        end
    end
else
    
for t = 1:PlaneID  

    if  visflag
        randcolor = rand(1,3); % 0.5*(finalMean(3:5,tt)+1);   
        figure(visflag), 
        scatter3(Planes{t}.Points(1,:), Planes{t}.Points(2,:), Planes{t}.Points(3,:),15,randcolor,'filled');
        nvec = [Planes{t}.Center  Planes{t}.Center+Planes{t}.Normal*0.2];
        figure(visflag),
        plot3(nvec(1,:), nvec(2,:), nvec(3,:),'-', 'Color', randcolor, 'LineWidth',2);
    end
end
    
    
end

if PlaneOfInterest > 0 
   if visflag
       t = PlaneOfInterest;
        randcolor = [1 0 0];%rand(1,3); % 0.5*(finalMean(3:5,tt)+1);   
        figure(visflag), 
        scatter3(Planes{t}.Points(1,:), Planes{t}.Points(2,:), Planes{t}.Points(3,:),15,randcolor,'filled');
        nvec = [Planes{t}.Center  Planes{t}.Center+Planes{t}.Normal*0.2];
        figure(visflag),
        plot3(nvec(1,:), nvec(2,:), nvec(3,:),'-', 'Color', randcolor, 'LineWidth',2);
    end
end
    
nPlanes = PlaneID;

end

