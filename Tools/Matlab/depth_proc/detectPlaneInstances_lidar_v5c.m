function [ Planes ] = detectPlaneInstances_lidar_v5c( meshRaw, visflag, resetParam )

persistent ONESCAN_         % single scan resolution 
persistent NUMSCAN_      % number of scans (in horizontal direction)
persistent v_angles
persistent s_angles
persistent Ccb_prev
persistent Tcb_prev

% parameters 
persistent normalComp_param 
persistent thre_svalue 
persistent thre_clusterSize 
persistent thre_memberSize 
persistent param_meanShiftResol 
persistent param_meanShiftWeights 

if isempty(ONESCAN_) || resetParam.flag 
    loadPersistentVariablesL_Terrain;
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

%Tcb = Tcb + Ccb*tr_kinect2head;

% if params == 0
Planes = [];
Points3D = [];
PlaneID = 0;
se = strel('disk',7,4);

% %%
% meshRaw = reshape(typecast(meshRaw,'single'), [ONESCAN_ NUMSCAN_]);
meshRaw(meshRaw>3) = 0;             % clamp on ranges
meshRaw(meshRaw<0.7) = 0;
[mesh_, s_, v_, nzcols] = scan2DepthImg_spherical0( meshRaw, s_angles, v_angles); % remove repeated measure   

if isempty(nzcols) 
    return;
end

% Let's check "motion"
T = reshape(resetParam.tfL16{nzcols(1)},4,4)';
Ccb = T(1:3,1:3);
Tcb = T(1:3,4) + [0; 0; 0.07];

mesh_ = medfilt2(mesh_,[5 5]);

NUMS_ = size(mesh_,1);
NUMV_ = size(mesh_,2);
Xind = repmat([1:NUMS_]',1,NUMV_); 
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

if visflag > 0
    P = Ccb*[ X0(:)'; Y0(:)'; Z0(:)' ] + repmat(Tcb,1,numel(X0));
    figure(visflag), hold off;
    showPointCloud(P(1,:),P(2,:),P(3,:),[0.5 0.5 0.5],'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up','MarkerSize',2);
    hold on;  
end
%% Normal Computation
[N, S] = computeNormal_lidarB(X0, Y0, Z0, mask, normalComp_param, 1);
validNormal = (find( sum(S,1) > 0)); 
validNormal = validNormal(find(S(4,validNormal)./(Z0(validNormal).^2)<thre_svalue));


%% Clustering  
data = [  Xind(validNormal) ; Yind(validNormal)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% generate initial mean information HERE for better starting 
stPoint = zeros(5,5);
stPoint(3:5,1) = Ccb*[0; 0; 1];   
stPoint(3:5,2) = Ccb*eulr2dcm([ 15;  0; 0]*pi/180)*[0; 0; 1];   
stPoint(3:5,3) = Ccb*eulr2dcm([-15;  0; 0]*pi/180)*[0; 0; 1];    
stPoint(3:5,4) = Ccb*eulr2dcm([ 0;  15; 0]*pi/180)*[0; 0; 1];  
stPoint(3:5,5) = Ccb*eulr2dcm([ 0; -15; 0]*pi/180)*[0; 0; 1];  
[finalMean,clusterXYcell,nMembers] = sphericalMeanShiftxyB(data,N(1:3,validNormal),param_meanShiftResol,param_meanShiftWeights,stPoint);

tags = zeros(NUMS_,NUMV_,'uint8');
devs = 1e6*ones(NUMS_,NUMV_);
% for each cluster
blankImg = zeros(NUMS_,NUMV_);
Indices = [];
for tt = 1: size(finalMean,2)      
    if nMembers(tt) > thre_clusterSize  % if cluster size is big enough
        
        connImg = blankImg;
        index = validNormal(clusterXYcell{tt}); 
        [index_xsub, index_ysub] = ind2sub([NUMS_ NUMV_],index);
        index_ = sub2ind(size(blankImg),index_xsub, index_ysub);
        connImg(index_) = 1;
        
       % Connectivity Check   
        L = bwlabel(connImg,8);
        NL = max(L(:));
        count_= 0;
        for t=1:NL
            indices{t} = find(L==t);
            count_(t) = length(indices{t});
            if length(indices{t}) < thre_memberSize
                L(indices{t}) = 0;
                count_(t) = 0;
            end
        end
               
       if NL > 0
            for t = 1: length(count_)                 
                if count_(t) > 0 % if the connected block is big enough 
                     
                     [~,whichcell] = intersect(index_ , indices{t});   
       
                    if ~isempty(whichcell)  
                        L_ = imdilate(im2bw(L),se); 
                        ttt = find(L_ == 1);
                      
                        % mean normal vector 
                        [ Center, n_, ins ] = estimatePlaneL(  X0(index(whichcell)), Y0(index(whichcell)), Z0(index(whichcell)), thre_memberSize );

                        if ~isempty(Center)
                            a0 = -n_'*Center;
                            % check points near the border 
                            del = abs(n_'*[X0(ttt)'; Y0(ttt)'; Z0(ttt)'] + a0*ones(1,length(ttt)));
                            sqdis = sum( ([X0(ttt)'; Y0(ttt)'; Z0(ttt)'] - repmat(Center,1,length(ttt))).^2 , 1) ;
                            withinPlane = (del < 0.01)';
                            closeToCenter = (devs(ttt) > sqdis');
                            
                            in = find(withinPlane & closeToCenter);
                            
                            PlaneID = PlaneID + 1;
                            % Indices{PlaneID} = ttt(in);
                            tags(ttt(in)) = PlaneID;
                            devs(ttt(in)) = sqdis(in);
                        end                     
                    end
                end
            end
       end
    end % end of for each cluster
end

NumPlane = PlaneID;
PlaneID = 0;
if NumPlane > 0 
     
    for t = 1:NumPlane  
        Indices{t} = find(tags==t);                                          
        % refinement 
        % (could test using svd and find the principal axes?) 
        [ Center, n_, ins ] = estimatePlaneL( X0(Indices{t})', Y0(Indices{t})', Z0(Indices{t})',thre_memberSize);

        if ~isempty(Center) && numel(ins) > thre_memberSize

            if Center'*n_ > 0
                n_ = -n_;
            end

            Indices{t} = Indices{t}(ins);

            % Find center, bbox, boundary
            [yind_s, xind_s] = ind2sub(size(connImg),Indices{t}');
            center_s = round(mean([xind_s;yind_s],2));
            % 8-directional extreme points 
            pts = find8ExtremePoints(tags, center_s, t);

            Pts = [];
            if ~isempty(pts)                               
                whichcell_ = sub2ind(size(connImg), pts(:,1), pts(:,2));  
                Pts(1,:) = X0(whichcell_);
                Pts(2,:) = Y0(whichcell_);
                Pts(3,:) = Z0(whichcell_);                                                                             
            end       

            % bbox 
            bbox = getBoundingBox(yind_s,xind_s);
            Bbox = zeros(3,size(bbox,1));
            whichcell__ = sub2ind(size(connImg), bbox(:,1), bbox(:,2));   
            Bbox(1,:) = X0(whichcell__);
            Bbox(2,:) = Y0(whichcell__);
            Bbox(3,:) = Z0(whichcell__);   
            
            % Coordinate Transformation
            Center = Ccb*Center + Tcb;
            Pts = Ccb*Pts + repmat(Tcb,1,size(Pts,2)) ;
            Bbox = Ccb*Bbox + repmat(Tcb,1,size(Bbox,2)) ;
            n_ = Ccb*n_;

            if n_(3) > 0.9                             
                PlaneID = PlaneID + 1;
                Planes{PlaneID} = struct('Center', Center,...
                                         'Normal', n_ ,...
                                         'Points', [Pts Bbox],...
                                         'Size',numel(Indices{t}));      

                if visflag 
                    Points3D{PlaneID} = [ X0(Indices{t})'; Y0(Indices{t})'; Z0(Indices{t})' ];
                    ALL = Ccb*Points3D{PlaneID} + repmat(Tcb,1,length(Points3D{PlaneID}));
                    randcolor = rand(1,3); % 0.5*(finalMean(3:5,tt)+1);   
                    figure(visflag), 
                    showPointCloud(ALL(1,:), ALL(2,:),ALL(3,:),...
                      randcolor,'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up','MarkerSize',5);
                    nvec = [Planes{PlaneID}.Center  Planes{PlaneID}.Center+Planes{PlaneID}.Normal*0.15];
                    figure(visflag),
                    plot3(nvec(1,:), nvec(2,:), nvec(3,:),'-', 'Color', [0 0 0], 'LineWidth',2);
                    plot3(Planes{PlaneID}.Points(1,:), Planes{PlaneID}.Points(2,:), Planes{PlaneID}.Points(3,:),'.', 'Color', [0 0 0],'MarkerSize',7);
                end
            end      

        end
    end
end

PlaneOfInterest = [];

% if~isempty(Planes)
%     idx = find(cellfun(@(x) x.Size, Planes(:)) > 100);    7
%     if ~isempty(idx)
%         for i=1:length(idx)
%             Planes{idx(i)}.Type = 'large';
%         end
%         PlaneOfInterest = idx;
%     end
% end
       
 % Coordinate Transformation
if 0 %~isempty(params)
    for t = 1:PlaneID  
        
        Planes{t}.Center = Ccb*Planes{t}.Center + Tcb;
        Planes{t}.Points = Ccb*Planes{t}.Points + repmat(Tcb,1,size(Planes{t}.Points,2)) ;
        Planes{t}.Normal = Ccb*Planes{t}.Normal;
%         ALL = Ccb*Points3D{t} + repmat(Tcb,1,length(Points3D{t}));
%         if visflag
%             
%             randcolor = rand(1,3); % 0.5*(finalMean(3:5,tt)+1);   
%             figure(visflag), 
%             showPointCloud(ALL(1,:), ALL(2,:),ALL(3,:),...
%                   randcolor,'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up','MarkerSize',5);
%             nvec = [Planes{t}.Center  Planes{t}.Center+Planes{t}.Normal*0.15];
%             figure(visflag),
%             plot3(nvec(1,:), nvec(2,:), nvec(3,:),'-', 'Color', [0 0 0], 'LineWidth',2);
%             plot3(Planes{t}.Points(1,:), Planes{t}.Points(2,:), Planes{t}.Points(3,:),'.', 'Color', [0 0 0]);
%         end
    end
end
    
Ccb_prev = Ccb;
Tcb_prev = Tcb;

end

