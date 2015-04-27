function [ Planes ] = detectPlaneInstances_lidar( meshRaw, params, visflag, resetParam )

persistent ONESCAN_         % single scan resolution 
persistent NUMSCAN_      % number of scans (in horizontal direction)
persistent v_angles;
persistent s_angles;
persistent Ccb_prev
persistent Tcb_prev


if resetParam.flag 
    loadPersistentVariablesL_0113;
end

if isempty(ONESCAN_),
    Planes = [];
    return;
end

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

%Tcb = Tcb + Ccb*tr_kinect2head;

% if params == 0
Planes = [];
PlaneID = 0;
    
% parameters 
normalComp_param = [3 1]; %  (w^2 + 1) half-window size
thre_svalue = 0.02; % The smaller it is, the flatter the plane fit is 
thre_clusterSize = 50; % number of clusters
thre_memberSize = 30; % number of connected members (in the image domain)
ms_resol = 0.5;% 0.6;         % mean shift resolution
ms_weights = [0 1]; %[0.2 1];   % mean shift weights (1:image distance, 2:angular distance in normal space) 

%%
meshRaw = reshape(typecast(meshRaw,'single'), [ONESCAN_ NUMSCAN_]);
meshRaw(meshRaw>5) = 0;             % clamp on ranges
meshRaw(meshRaw<0.1) = 0;
[mesh_, s_, v_] = scan2DepthImg_spherical( meshRaw, s_angles, v_angles); % remove repeated measure   
NUMS_ = size(mesh_,1);
NUMV_ = size(mesh_,2);
Xind = kron([1:NUMS_]',ones(1,NUMV_)); 
Yind = repmat(1:NUMV_,NUMS_,1); % index in 1D array    
validInd = find(mesh_>0);   
mask = zeros(size(mesh_));
mask(validInd) = 1;
% Convert to x, y, z 
cv_ = zeros(size(mesh_)); sv_ = cv_; cs_ = cv_; ss_ = cv_;
cv_(validInd) = cos(v_(validInd));
sv_(validInd) = sin(v_(validInd));
cs_(validInd) = cos(s_(validInd));
ss_(validInd) = sin(s_(validInd));
X0 = cs_.*cv_.*mesh_;
Y0 = ss_.*cv_.*mesh_; 
Z0  = -sv_.*mesh_ ;

%% Normal Computation
[N, S] = computeNormal_lidar(X0, Y0, Z0, mask, normalComp_param(1));
validNormal = (find( sum(S,1) > 0)); 
validNormal = validNormal(find(S(4,validNormal)<0.03));

% figure(5), scatter3(N(1,validNormal),N(2,validNormal),N(3,validNormal),5,[0.5 0.5 0.5], 'filled'); hold on;

%% Clustering  
data = [  Xind(validNormal) ; Yind(validNormal)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% generate initial mean information HERE for better starting 
[finalMean,clusterXYcell,nMembers] = sphericalMeanShiftxyB(data,N(1:3,validNormal),ms_resol,ms_weights);

% for each cluster
blankConnImg = zeros(floor(NUMS_/normalComp_param(2)),NUMV_);
for tt = 1: size(finalMean,2)      
    if nMembers(tt) > thre_clusterSize  % if cluster size is big enough
        
        connImg = blankConnImg;
        index = validNormal(clusterXYcell{tt}); 
        [index_x, index_y] = ind2sub([NUMS_ NUMV_],index);
        index_xsub = floor(index_x / normalComp_param(2));
        index_ysub = index_y;
        index_ = sub2ind(size(blankConnImg),index_xsub, index_ysub);
        connImg(index_) = 1;
       %% Connectivity Check         
        [conn, ids, count_, indices ] = test4Connectivity(connImg); % Connected-component analysis        
       %%%%%%%%%%%%%%%%%%%%%%%% 
       
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
                        Bbox(1,:) = X0(index(whichcell__));
                        Bbox(2,:) = Y0(index(whichcell__));
                        Bbox(3,:) = Z0(index(whichcell__));
                        
                        % 8-directional extreme points 
                        pts = find8ExtremePoints(conn, center_s, ids(t));
                        if ~isempty(pts)                               
                            [dummy,whichcell_] = intersect(index_ , sub2ind(size(conn), pts(:,1), pts(:,2)));  
                          
                            Pts(1,:) = X0(index(whichcell_));
                            Pts(2,:) = Y0(index(whichcell_));
                            Pts(3,:) = Z0(index(whichcell_));                                                                             
                        end       
                        
                     %% refinement 
                        % (could test using svd and find the principal axes?) 
                        [c, ins] = estimatePlaneL_useall( X0(index(whichcell)), Y0(index(whichcell)), Z0(index(whichcell)));
                     
                        %% save output 
                        if ~isempty(c) && numel(ins) > 5
                            n_ = c(1:3);
                            n_ = -n_/norm(n_);
                           
                            z_mean = mean(Z0(index(whichcell(ins))));                                  
                            x_mean = mean(X0(index(whichcell(ins))));
                            y_mean = mean(Y0(index(whichcell(ins))));
                                                                          
                            Center = [x_mean; y_mean; z_mean];
                            
                            if Center'*n_ > 0
                                n_ = -n_;
                            end
 
                            PlaneID = PlaneID + 1;
                            Planes{PlaneID} = struct('Center', Center, 'Normal', n_ , 'Points', [Pts Bbox],'Size',numel(ins));                             
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
        Ztemp = Z0(validInd);
        Ytemp = Y0(validInd);
        Xtemp = X0(validInd);
        Xvis = Ccb*[Xtemp(:)'; Ytemp(:)'; Ztemp(:)'] + repmat(Tcb,1,length(Ztemp(:)));
        figure(visflag), [az,el] = view; hold off; 
        scatter3( Xvis(1,:),Xvis(2,:), Xvis(3,:), 2 ,[0.5 0.5 0.5] ,'filled'); axis equal; hold on;
        set(gca,'XDir','reverse');
        xlabel('y');
        ylabel('x');
        zlabel('z','Rotation',0);        
        % axis([0 1.5 -1 1 0 1.5]);
        view(az,el);

        figure(visflag + 1), hold off;
        imagesc(mesh_); % axis equal;

    end
    
 % Coordinate Transformation
if ~isempty(params)
    for t = 1:PlaneID  
        
        Planes{t}.Center = Ccb*Planes{t}.Center + Tcb;
        Planes{t}.Points = Ccb*Planes{t}.Points + repmat(Tcb,1,size(Planes{t}.Points,2)) ;
        Planes{t}.Normal = Ccb*Planes{t}.Normal;

        if visflag
            randcolor = rand(1,3); % 0.5*(finalMean(3:5,tt)+1);   
            figure(visflag), 
            scatter3(Planes{t}.Points(1,:), Planes{t}.Points(2,:), Planes{t}.Points(3,:),15,randcolor,'filled');
            nvec = [Planes{t}.Center  Planes{t}.Center+Planes{t}.Normal*0.2];
            figure(visflag),
            plot3(nvec(1,:), nvec(2,:), nvec(3,:),'-', 'Color', randcolor, 'LineWidth',2);
        end
    end
end
    

end

