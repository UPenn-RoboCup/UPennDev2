function [ Planes, nPlanes, PlaneOfInterest ] = detectPlaneInstances_kinect_v4ese650( depthRaw, Rot, tr, ui )

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
persistent manualModel

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
    loadPersistentVariables_0310;
end

Ccb = Rot;
Tcb = tr;
if isempty(Ccb_prev)
    Ccb_prev = Ccb;
    Tcb_prev = Tcb;
end


Planes =[]; 
Points3D = [];
Indices = [];                                           
PlaneID = 0;
nPlanes = 0; %#ok<NASGU>
PlaneOfInterest = [];    
clickxy = [];

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
%stPoint = [];
stPoint = zeros(5,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ui.clickType == 0    
    if ui.taskMode == 1 % ground
         stPoint = zeros(5,1);
         stPoint(3:5,:) = Ccb(3,:)';
    else
        if 0  %~isempty(prevNormals)
            stPoint = zeros(5,size(prevNormals,2));
            stPoint(3:5,:) = Ccb'*prevNormals;    
        end
    end
    
elseif ui.clickType > 0 && isempty(manualModel)
% Display a figure for the user to make a click on. 
    ui.clickxy = getClickPoint( ui, depth );
end

% compute Closest Point And Normal;
[finalMean,clusterXYcell,nMembers] = sphericalMeanShiftxyB(data,A(1:3,validNormal),param_meanShiftResol,param_meanShiftWeights,stPoint);

[Xs,Ys,Zs] = sphere(20);
figure(5), h = mesh(Xs,Ys,Zs); hold on;
set(h,'FaceAlpha',0);
set(h,'EdgeColor',[0.7 0.7 0.7]);
axis equal;
set(gca,'XTick',[])
set(gca,'YTick',[])
set(gca,'ZTick',[])
figure(5), scatter3(A(1,validNormal),A(2,validNormal),A(3,validNormal),5,'filled');

figure(15), h = mesh(Xs,Ys,Zs); hold on;
set(h,'FaceAlpha',0);
set(h,'EdgeColor',[0.7 0.7 0.7]);
axis equal;
set(gca,'XTick',[])
set(gca,'YTick',[])
set(gca,'ZTick',[])



% for each cluster
blankConnImg = zeros(floor(DEPTH_W/param_normalComputation(2)),floor(DEPTH_H /param_normalComputation(2)));
for tt = 1: size(finalMean,2)      
    if nMembers(tt) > thre_clusterSize  % if cluster size is big enough 
       
        randcol = rand([1 3]);
        index = validNormal(clusterXYcell{tt}); 
        figure(15), scatter3(A(1,index),A(2,index),A(3,index),5,randcol,'filled');
        
        Pts = [];
        Pts(1,:) = depth(index)*0.001;
        Pts(2,:) = Yind_c(index)*Sy.*Pts(1,:);
        Pts(3,:) = Xind_c(index)*Sx.*Pts(1,:);  
        
        figure(4), showPointCloud(Pts(3,:),Pts(2,:),Pts(1,:),randcol,'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down'); hold on;
        
        if 0 %plane_dist(ui.taskMode,Ccb*finalMean(3:5,tt)) == true % if the normal is close to our models
            connImg = blankConnImg;
            index = validNormal(clusterXYcell{tt}); 
            [index_x, index_y] = ind2sub([DEPTH_W DEPTH_H],index);
            index_xsub = floor(index_x / param_normalComputation(2));
            index_ysub = floor(index_y / param_normalComputation(2));
            index_ = sub2ind(floor([DEPTH_W DEPTH_H]/param_normalComputation(2)),index_xsub, index_ysub);
            connImg(index_) = 1;
        
            %% Connectivity Check 
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
                            Bbox(1,:) = depth(index(whichcell__))*0.001;
                            Bbox(2,:) = Yind_c(index(whichcell__))*Sy.*Bbox(1,:);
                            Bbox(3,:) = -Xind_c(index(whichcell__))*Sx.*Bbox(1,:);

                            % 8-directional extreme points 
                            pts = find8ExtremePoints(L, center_s, t);
                            
                            
                            %pts = findVertSidePoints(L, center_s, t, bbox);
                            
                            
                            
                            if ~isempty(pts)                               
                                [dummy,whichcell_] = intersect(index_ , sub2ind(size(connImg), pts(:,1), pts(:,2)));  

                                Pts(1,:) = depth(index(whichcell_))*0.001;
                                Pts(2,:) = Yind_c(index(whichcell_))*Sy.*Pts(1,:);
                                Pts(3,:) = -Xind_c(index(whichcell_))*Sx.*Pts(1,:);                                                                              
                            end       

                         %% refinement 
                            % (could test using svd and find the principal axes?)                       
                            [c, ins] = estimatePlane_useall( Xind_c(index(whichcell)), Yind_c(index(whichcell)), ZZ(index(whichcell)));

                          %% save output 
                            if ~isempty(c) && numel(ins) > 5 
                                c(1) = c(1)/Sx;
                                c(2) = c(2)/Sy;
                                n_ = c;
                                n_ = -n_/norm(n_);
                                n__ = [n_(3) n_(2) -n_(1)];

                                z_ = depth(index(whichcell))*0.001;
                                z_mean = mean(z_);                                  
                                x_mean = mean((Xind(index(whichcell))-IMCX)/fx.*z_);
                                y_mean = mean((Yind(index(whichcell))-IMCY)/fy.*z_);

                                Center = [ z_mean; y_mean; -x_mean];

                                PlaneID = PlaneID + 1;
                                Planes{PlaneID} = struct('Center',Center,...
                                                         'Normal', n__',...
                                                         'Points',[Pts Bbox],...
                                                         'Size', numel(ins),...
                                                         'Type','Else');   

                                if ui.clickType == 2 
                                    Indices{PlaneID} = [Xind(index(whichcell)); Yind(index(whichcell))];
                                else %if ui.clickType == 3                                 
                                    Points3D{PlaneID} = [  z_; (Yind(index(whichcell))-IMCY)/fy.*z_; -(Xind(index(whichcell))-IMCX)/fx.*z_];                   
                                end
                            end
                        end
                    end
                end
            end
        end % process clusters of interest normals
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
    figure(3),% subplot(2,1,2);  
    Ztemp = depth(validInd(10:10:end))*0.001;
    Ytemp = Yind_c(validInd(10:10:end))*Sy.*Ztemp;
    Xtemp = -Xind_c(validInd(10:10:end))*Sx.*Ztemp;
    Xvis = Ccb*[  Ztemp(:)'; Ytemp(:)'; Xtemp(:)'] + repmat(Tcb,1,length(Ztemp(:)));
    figure(3), [az,el] = view; hold off; 
    scatter3(Xvis(1,:),  Xvis(2,:), Xvis(3,:), 2 ,[0.5 0.5 0.5] ,'filled'); axis equal; hold on;
    %set(gca,'XDir','reverse');
    xlabel('x');
    ylabel('y');
    zlabel('z','Rotation',0);        
  %  axis([0 4.0 -2 2 -0.2 2.0]);
    view(az,el);
end
    
Ns = zeros(3,PlaneID);
Cs = zeros(3,PlaneID);
Szs = zeros(1,PlaneID);

 % Coordinate Transformation
prevNormals = zeros(3,PlaneID);
if ~issame(Rot,eye(3))
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
    end
else
    for t = 1:PlaneID  
         prevNormals(:,t) = Planes{t}.Normal;
    end
end
  
% Choose if user task is specified
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
elseif ui.taskMode == 2
    n_inner = abs([0 0 1]*Ns);
    flag = (n_inner < 0.1);
     % pick the largest
    candidates = find(flag>0);
    if ~isempty(candidates)
        [val, idx] = max(cellfun(@(x) x.Size, Planes(candidates)));
        Planes{candidates(idx)}.Type = 'wall';
        PlaneOfInterest = candidates(idx);
    end
elseif ui.taskMode == 11
    idx = find(cellfun(@(x) x.Size, Planes(:)) > 1000);    
    if ~isempty(idx)
        for i=1:length(idx)
            Planes{idx(i)}.Type = 'large';
        end
        PlaneOfInterest = idx;
    end
elseif ui.taskMode == 101 
    if ~isempty(ui.clickxy) 
        minval = 10000;
        min_idx = 0;
        for t=1:PlaneID 
            dsq =  (Indices{t} - repmat(ui.clickxy',1,length(Indices{t}))).^2;
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

        if PlaneOfInterest > 0
            manualModel.Normal = Planes{PlaneOfInterest}.Normal;
            manualModel.Center = Planes{PlaneOfInterest}.Center;
            prevNormals = Planes{t}.Normal;
        end
    elseif ~isempty(manualModel)
        n_inner = 1-abs([0 0 1]*manualModel.Normal);
        flag = (n_inner < 0.1);        
        flag = flag & (abs(manualModel.Center(3)-Cs(3,:)) < 0.1);
         % pick the largest
        candidates = find(flag>0);
        if ~isempty(candidates)
            [val, idx] = max(cellfun(@(x) x.Size, Planes(candidates)));
            Planes{candidates(idx)}.Type = 'manual';
            PlaneOfInterest = candidates(idx);
        end
    end
else
    
    PlaneOfInterest = 1:PlaneID;
end
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

% Visualize
if  ui.figures(3) > 0    
   %  for t = 1:PlaneID    
   %      randcolor = rand(1,3); % 0.5*(finalMean(3:5,tt)+1);   
%         figure(3), 
%         scatter3(Planes{t}.Points(1,:), Planes{t}.Points(2,:), Planes{t}.Points(3,:),15,randcolor,'filled');
%         nvec = [Planes{t}.Center  Planes{t}.Center+Planes{t}.Normal*0.2];
%         figure(3),
%         plot3(nvec(1,:), nvec(2,:), nvec(3,:),'-', 'Color', randcolor, 'LineWidth',2);
%     end
    
   if 1 %~isempty(PlaneOfInterest)
       for t_=1:length(PlaneOfInterest);
            t = PlaneOfInterest(t_);
            randcolor = rand(1,3); % 0.5*(finalMean(3:5,tt)+1);   
            figure(3),% subplot(2,1,2);        
            ALL = Ccb*Points3D{t} + repmat(Tcb,1,length(Points3D{t}));
            scatter3(ALL(1,:), ALL(2,:), ALL(3,:),5,randcolor,'filled');
            scatter3(Planes{t}.Points(1,:), Planes{t}.Points(2,:), Planes{t}.Points(3,:),15,'k','filled');
            nvec = [Planes{t}.Center  Planes{t}.Center+Planes{t}.Normal*0.5];
            plot3(nvec(1,:), nvec(2,:), nvec(3,:),'-', 'Color', 'k', 'LineWidth',2);
       end
    end
end
    
% save previous pose
Ccb_prev = Ccb;
Tcb_prev = Tcb;

nPlanes = PlaneID;
    
end

