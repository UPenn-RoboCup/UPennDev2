function [ Planes, nPlanes, PlaneOfInterest ] = detectPlaneInstances_kinect_v3( depthRaw, Rot, tr, ui )

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
persistent prevNormals

% Important Parameters 
persistent param_normalComputation
persistent thre_sValue
persistent thre_clusterSize
persistent thre_memberSize
persistent param_meanShiftResol
persistent param_meanShiftWeights

if nargin < 4
    error('The number of input arguments must be 4');
end

if isempty(DEPTH_W) || ui.reset == 1
    loadPersistentVariables_0216;
end

M =[0.9996         0   -0.0287
    0.0017    0.9983    0.0576
    0.0287   -0.0576    0.9979]; % should consider the x-y alignment (it only fixes z)

Ccb = Rot'*M;
Ccb = Ccb*[0 -1 0; 1 0 0; 0 0 1];
Tcb = tr;
if isempty(Ccb_prev)
    Ccb_prev = Ccb;
    Tcb_prev = Tcb;
end
% Tcb = Tcb;%Ccb*tr_kinect2head;


Planes =[]; 
Points3D = [];
Indices = [];                                           
PlaneID = 0;
nPlanes = 0; %#ok<NASGU>
PlaneOfInterest = 0;    

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
[A, S, V] = mexFitPlane_uv1d( Xind_c, Yind_c, ZZ, logical(mask),param_normalComputation);
validNormal = find( V > 0); 
validNormal = validNormal(find(S(4,validNormal)< thre_sValue));

%% Clustering  
data = [  Xind_(validNormal) ; Yind_(validNormal)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ui.clickType == 0
    
    if ui.taskMode == 1 % ground
         stPoint = zeros(5,1);
         stPoint(3:5,:) = Ccb(3,:)';
    else
        if ~isempty(prevNormals)
            stPoint = zeros(5,size(prevNormals,2));
            stPoint(3:5,:) = Ccb'*prevNormals;
        else
            stPoint = [];
        end
    end
% elseif ui.clickType > 0 
% Display a figure for the user to make a click on. 
% stPoint = getClickPoint( ui, depth );
end

% compute Closest Point And Normal;

[finalMean,clusterXYcell,nMembers] = sphericalMeanShiftxyB(data,A(1:3,validNormal),param_meanShiftResol,param_meanShiftWeights,stPoint);

% for each cluster
blankConnImg = zeros(floor(DEPTH_W/param_normalComputation(2)),floor(DEPTH_H /param_normalComputation(2)));
for tt = 1: size(finalMean,2)      
    if nMembers(tt) > thre_clusterSize  % if cluster size is big enough
        
        connImg = blankConnImg;
        index = validNormal(clusterXYcell{tt}); 
        [index_x, index_y] = ind2sub([DEPTH_W DEPTH_H],index);
        index_xsub = floor(index_x / param_normalComputation(2));
        index_ysub = floor(index_y / param_normalComputation(2));
        index_ = sub2ind(floor([DEPTH_W DEPTH_H]/param_normalComputation(2)),index_xsub, index_ysub);
        connImg(index_) = 1;
       %% Connectivity Check 
        % Connected-component analysis 
        % check an error related to "eqlabel"
      
        %CC = bwconncomp(connImg,4);
        %numPixels = cellfun(@numel,CC.PixelIdxList);
        %tic,
        %[conn, ids, count_, indices ] = test4Connectivity(connImg);
        %toc,
        
        L = bwlabel(connImg,4);
        NL = max(L(:));
        for t=1:NL
            indices{t} = find(L==t);
            count_(t) = length(indices{t});
        end
       
        if NL >0
            for t = 1: length(count_)                 
                if count_(t) > thre_memberSize % if the connected bloc is big enough 
                    [dummy,whichcell] = intersect(index_ , indices{t});    
                    if ~isempty(whichcell)   
                     %% Find center, bbox, boundary
                        [yind_s, xind_s] = ind2sub(size(connImg),indices{t}');
                        center_s = round(mean([xind_s;yind_s],2));
                       
                        Pts = [];
                        bbox = getBoundingBox(yind_s,xind_s);
                        Bbox = zeros(3,size(bbox,1));
                        [dummy,whichcell__] = intersect(index_ , sub2ind(size(connImg), bbox(:,1), bbox(:,2)));   
                        Bbox(2,:) = depth(index(whichcell__))*0.001;
                        Bbox(1,:) = -Yind_c(index(whichcell__))*Sy.*Bbox(1,:);
                        Bbox(3,:) = -Xind_c(index(whichcell__))*Sx.*Bbox(1,:);
                        
                        % 8-directional extreme points 
                        pts = find8ExtremePoints(L, center_s, t);
                        if ~isempty(pts)                               
                            [dummy,whichcell_] = intersect(index_ , sub2ind(size(connImg), pts(:,1), pts(:,2)));  
                          
                            Pts(2,:) = depth(index(whichcell_))*0.001;
                            Pts(1,:) = -Yind_c(index(whichcell_))*Sy.*Pts(1,:);
                            Pts(3,:) = -Xind_c(index(whichcell_))*Sx.*Pts(1,:);                                                                              
                        end       
                        
                     %% refinement 
                        % (could test using svd and find the principal axes?)                       
                        [c, ins] = estimatePlane_useall( Xind_c(index(whichcell)), Yind_c(index(whichcell)), ZZ(index(whichcell)));
                       
                      %% save output 
                        if ~isempty(c) && numel(ins) > 5 
                            c(1) = c(1)/Sx;
                            c(2) = c(2)/Sy;
                            n_ = c([2 1 3]);
                            n_ = -n_/norm(n_);
                            n__ = [-n_(2) n_(3) -n_(1)]/norm(n_);
                           
                            z_ = depth(index(whichcell))*0.001;
                            z_mean = mean(z_);                                  
                            x_mean = mean((Xind(index(whichcell))-IMCX)/fx.*z_);
                            y_mean = mean((Yind(index(whichcell))-IMCY)/fy.*z_);
                                                                          
                            Center = [ -y_mean; z_mean; -x_mean];
 
                            PlaneID = PlaneID + 1;
                            Planes{PlaneID} = struct('Center',Center,...
                                                     'Normal', n__',...
                                                     'Points',[Pts Bbox],...
                                                     'Size', numel(ins),...
                                                     'Type','Else');   
                                                 
                           % if ui.clickType == 2 
                           %     Indices{PlaneID} = [Xind(index(whichcell)); Yind(index(whichcell))];
                            %elseif ui.clickType == 3                                 
                                Points3D{PlaneID} = [  -(Yind(index(whichcell))-IMCY)/fy.*z_; z_;-(Xind(index(whichcell))-IMCX)/fx.*z_];                   
                           % end
                        end
                    end
                end
            end
        end
        
    end % end of for each cluster
end
    
% visualization
if ui.figures(1) > 0
    figure(1), hold off;
    switch ui.figures(1)
        case 1,
            imagesc(depthRaw'); 
        case 2,
            imagesc(depth'); 
    end
    hold on; axis([1 DEPTH_W 1 DEPTH_H]); colormap('gray');
end

if ui.figures(3) 
     figure(3), subplot(2,1,2);  
    Ztemp = depth(validInd(10:10:end))*0.001;
    Ytemp = -Yind_c(validInd(10:10:end))*Sy.*Ztemp;
    Xtemp = -Xind_c(validInd(10:10:end))*Sx.*Ztemp;
    Xvis = Ccb*[ Ytemp(:)'; Ztemp(:)'; Xtemp(:)'] + repmat(Tcb,1,length(Ztemp(:)));
    figure(3), [az,el] = view; hold off; 
    scatter3(Xvis(2,:),  Xvis(1,:), Xvis(3,:), 2 ,[0.5 0.5 0.5] ,'filled'); axis equal; hold on;
    set(gca,'XDir','reverse');
    xlabel('y');
    ylabel('x');
    zlabel('z','Rotation',0);        
  %  axis([0 4.0 -2 2 -0.2 2.0]);
    view(az,el);

   % figure(101), hold off;
   % imagesc(depth'); axis equal
end
    
Ns = zeros(3,PlaneID);
Cs = zeros(3,PlaneID);
Szs = zeros(1,PlaneID);

 % Coordinate Transformation
if ~issame(Rot,eye(3))
    prevNormals = zeros(3,PlaneID);
    for t = 1:PlaneID  
        Planes{t}.Center = Ccb*Planes{t}.Center + Tcb;
        Planes{t}.Points = Ccb*Planes{t}.Points + repmat(Tcb,1,size(Planes{t}.Points,2)) ;
        Planes{t}.Normal = Ccb*Planes{t}.Normal;
        if ui.taskMode > 0 
            Ns(:,t) = Planes{t}.Normal;
            Cs(:,t) = Planes{t}.Center;
        end
        
        % save normals for clustering in the next frame
        prevNormals(:,t) = Planes{t}.Normal;
        
        if 0 % visflag
            randcolor = rand(1,3); % 0.5*(finalMean(3:5,tt)+1);   
            figure(visflag),    
            scatter3(Planes{t}.Points(2,:), Planes{t}.Points(1,:), Planes{t}.Points(3,:),15,randcolor,'filled');
            nvec = [Planes{t}.Center  Planes{t}.Center+Planes{t}.Normal*0.2];
            figure(visflag),
            plot3(nvec(2,:), nvec(1,:), nvec(3,:),'-', 'Color', randcolor, 'LineWidth',2);
        end
    end
else
    prevNormals = zeros(3,PlaneID);
    for t = 1:PlaneID  
         prevNormals(:,t) = Planes{t}.Normal;
    end
end
  
if ui.taskMode == 1 % ground
    % testGround();
    n_inner = 1-abs([0 0 1]*Ns);
    flag = (n_inner < 0.1);
    flag = flag & (Cs(3,:) < 0.2);
    % pick the largest
    candidates = find(flag>0);
    if ~isempty(candidates)
        [val, idx] = max(cellfun(@(x) x.Size, Planes(candidates)));
        Planes{candidates(idx)}.Type = 'ground';
        PlaneOfInterest = candidates(idx);
    end
end

% elseif strcmp(params{3}.mode, 'manual_2d')
%     minval = 10000;
%     min_idx = 0;
%     for t=1:PlaneID 
%         dsq =  (Indices{t} - repmat(params{3}.data',1,length(Indices{t}))).^2;
%         [val] = min(sum(dsq,1));
%         if val < minval 
%             if min_idx > 0 
%                 Planes{min_idx}.Type = 'Else';
%             end
%             Planes{t}.Type = 'Manual';
%             PlaneOfInterest = t;   
%             min_idx = t;
%             minval = val;
%         end
%     end
% elseif strcmp(params{3}.mode, 'manual_3d')
%     % Find closest plane center (Assuming not ground)
%     minval = 10000;
%     min_idx = 0;
%     for t=1:PlaneID
%         dsq = ( Ccb*Points3D{t} + repmat(Tcb,1,length(Points3D{t})) - repmat((params{3}.data'),1,size(Points3D{t},2))).^2;
%         [val] = min(sum(dsq,1));
%         if val < minval 
%             if min_idx > 0 
%                 Planes{min_idx}.Type = 'Else';
%             end
%             Planes{t}.Type = 'Manual';
%             PlaneOfInterest = t;   
%             min_idx = t;
%             minval = val;
%         end
%     end
% else
if  ui.figures(3) > 0    
%     for t = 1:PlaneID    
%         randcolor = rand(1,3); % 0.5*(finalMean(3:5,tt)+1);   
%         figure(3), 
%         scatter3(Planes{t}.Points(1,:), Planes{t}.Points(2,:), Planes{t}.Points(3,:),15,randcolor,'filled');
%         nvec = [Planes{t}.Center  Planes{t}.Center+Planes{t}.Normal*0.2];
%         figure(3),
%         plot3(nvec(1,:), nvec(2,:), nvec(3,:),'-', 'Color', randcolor, 'LineWidth',2);
%     end
    
    if PlaneOfInterest > 0 
        t = PlaneOfInterest;
        randcolor = [1 0 0];%rand(1,3); % 0.5*(finalMean(3:5,tt)+1);   
        figure(3), subplot(2,1,2);        
        ALL = Ccb*Points3D{t} + repmat(Tcb,1,length(Points3D{t}));
        scatter3(ALL(1,:), ALL(2,:), ALL(3,:),5,randcolor,'filled');
        scatter3(Planes{t}.Points(1,:), Planes{t}.Points(2,:), Planes{t}.Points(3,:),15,randcolor,'filled');
        nvec = [Planes{t}.Center  Planes{t}.Center+Planes{t}.Normal*0.2];
        plot3(nvec(1,:), nvec(2,:), nvec(3,:),'-', 'Color', 'k', 'LineWidth',2);
    end
end
    
% save previous pose
Ccb_prev = Ccb;
Tcb_prev = Tcb;

nPlanes = PlaneID;
    
end

