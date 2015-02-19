close all;
clear all;

DEPTH_W = 512;
DEPTH_H = 424;
DEPTH_MAX = 4000;%8000;
DEPTH_MIN = 200;
IMCX = DEPTH_W/2;
IMCY = DEPTH_H/2;

fx = 391.1;
fy = 463.1;

% Input: depth image, (depth)camera intrinsic parameters 
% Output: 
% (1) surface normals, 
% (2) some boundary points 
% All in camera centered coordinate

% parameters 
normalComp_param = [5 5]; % 1: (odd nember)^2+1 <- window sizw,  2: increment  
thre_svalue = 0.03; % The smaller it is, the flatter the plane fit is 
thre_clusterSize = 50; % number of clusters
thre_memberSize = 30; % number of connected members (in the image domain)
ms_resol = 0.52;         % mean shift resolution
ms_weights = [0.1 1];   % mean shift weights (1:image distance, 2:angular distance in normal space) 


RGB_W = 1920;
RGB_H = 1080;

Tb = [0 0 1; -1 0 0; 0 -1 0];

Yind = repmat(1:DEPTH_W,1,DEPTH_H); % index in 1D array 
Xind = kron(1:DEPTH_H,ones(1,DEPTH_W)); 

Xind_c = (reshape(Xind,DEPTH_W,DEPTH_H)-IMCX)/IMCX; % index in 2D array representation
Yind_c = (reshape(Yind,DEPTH_W,DEPTH_H)-IMCY)/IMCY; 

% foldername = 'Data';
foldername = 'kinect2_on_robot';
datestamp = '01.23.2015.14.48.29';
% -- datestamp = '12.04.2014.09.24.33';
% datestamp = '12.04.2014.09.25.21'; % kitchen, fast moving 
% datestamp = '12.04.2014.09.29.52'; % backyard, mostly sky  
% datestamp = '12.04.2014.09.30.05'; % backyard, plastic cloth
% datestamp = '12.04.2014.09.30.15'; % % backyard, gounrd + plastic cloth
% datestamp = '12.04.2014.09.30.29'; % backyard,
% datestamp = '12.04.2014.09.30.46'; % backyard, stairs
% -- datestamp = '12.04.2014.09.30.56';
% -- datestamp = '12.04.2014.09.31.22';
% datestamp = '12.04.2014.09.31.54'; % backyard, stairs
% datestamp = '12.04.2014.09.32.04'; % backyard, stairs
f_depth = fopen(sprintf('%s/k2_depth_r_%s.log', foldername,datestamp));
f_rgb = fopen(sprintf('%s/k2_rgb_r_%s.log', foldername,datestamp));
fid = fopen(sprintf('%s/k2_rgb_m_%s.log',foldername,datestamp));
rgbMeta = fread(fid,Inf,'*uint8');
fclose(fid);
fid = fopen(sprintf('%s/k2_depth_m_%s.log', foldername,datestamp));
depthMeta = fread(fid,Inf,'*uint8');
depthMeta = msgpack('unpacker', depthMeta, 'uint8');
fclose(fid);
rgbMeta = msgpack('unpacker',rgbMeta,'uint8');

saveflag = 0;
visflag = 1;

ilog = 0;
prevm = [];
L2 = [];
while ~feof(f_depth)
    
    depthRaw = fread(f_depth, [DEPTH_W, DEPTH_H], '*single');
    if isempty(depthRaw)
        break;
    end
    
    ilog = ilog + 1;
    Planes = cell(1);
    PlaneID = 0;
    if 1
    meta = rgbMeta{ilog};
    rgbJPEG = fread(f_rgb, meta.rsz, '*uint8');
    rgb_img0 = djpeg(rgbJPEG);
    rgb_img(:,:,1) = rgb_img0(:,:,1);
    rgb_img(:,:,2) = rgb_img0(:,:,2);
    rgb_img(:,:,3) = rgb_img0(:,:,3);   
    
        figure(90), imshow(rgb_img);
    end
    
    if 1% ilog > 10
        
    tic,
    % Initialize mask
    mask = ones(DEPTH_W, DEPTH_H);
    mask(depthRaw(:) <= DEPTH_MIN) = 0;
    mask(depthRaw(:) >= DEPTH_MAX) = 0;
        
    % Filter depth
    depth = depthRaw.*mask;
    
    depth = medfilt2(depth,[7 7]);
    
    validInd = find(depth(:)>DEPTH_MIN);
    mask = zeros(DEPTH_W, DEPTH_H);
    mask(validInd) = 1;
    depth = depth.*mask;
        
    % inverse depth
    ZZ = double(3000./depth);
   
    [A, S, V] = mexFitPlane_uv1d( Xind_c, Yind_c, ZZ, logical(mask),normalComp_param);


    validNormal = find( V > 0); 
    validNormal = validNormal(find(S(4,validNormal)< thre_svalue));

    if visflag
 
    figure(2), hold off; 
    scatter3( depth(validNormal)*0.001, -Yind_c(validNormal)*IMCY/fy.*depth(validNormal)*0.001,...
        -Xind_c(validNormal)*IMCX/fx.*depth(validNormal)*0.001, 5 ,'filled');
            axis equal; hold on;
    end
      
    u = randn(3,3);
    u(:,1) = u(:,1)/norm(u(:,1));
    u(:,2) = u(:,2)/norm(u(:,2));
    u(:,3) = u(:,3)/norm(u(:,3));    
    data = [  (Xind(validNormal)-IMCX)/IMCX; (Yind(validNormal)-IMCY)/IMCY];
   
   % [finalMean,clusterXYcell,nMembers]  = sphericalMeanShiftG(A(1:3,validNormal),0.3);
     % works OK
   % [finalMean,clusterXYcell,nMembers] = sphericalMeanShiftxy(data,A(1:3,validNormal),0.45,[0 1]);
   
    % 
    [finalMean,clusterXYcell,nMembers] = sphericalMeanShiftxy(data,A(1:3,validNormal),ms_resol,ms_weights);
 
    if visflag
    figure(99), hold off;
    scatter3(A(1,validNormal),A(2,validNormal),A(3,validNormal),3,0.5*ones(1,3)); axis equal; hold on;
    
    figure(1), hold off;
    imshow((depth'-DEPTH_MIN)/(DEPTH_MAX-DEPTH_MIN)); axis equal; hold on;
    figure(11), hold off;
    imshow((depth'-DEPTH_MIN)/(DEPTH_MAX-DEPTH_MIN)); axis equal; hold on;
    end
    
     for tt = 1: size(finalMean,2)  
         if visflag
       figure(2), hold off; 
        scatter3( depth(validNormal)*0.001, -Yind_c(validNormal)*IMCY/fy.*depth(validNormal)*0.001,...
        -Xind_c(validNormal)*IMCX/fx.*depth(validNormal)*0.001, 5 ,'filled');
            axis equal; hold on;
        xlabel('z');
        ylabel('-x');
        zlabel('-y','Rotation',0);
            axis equal; hold on;
         end
         connImg = zeros(floor(DEPTH_W/normalComp_param(2)),floor(DEPTH_H /normalComp_param(2)));
         
         if nMembers(tt) > thre_clusterSize
             randcolor = rand(3,1);%0.5*(finalMean(3:5,tt)+1);
            
             index = validNormal(clusterXYcell{tt}); 
             [index_x, index_y] = ind2sub([DEPTH_W DEPTH_H],index);
             index_xsub = floor(index_x / 5);
             index_ysub = floor(index_y / 5);
             index_ = sub2ind(floor([DEPTH_W DEPTH_H]/5),index_xsub, index_ysub);
             %randcolor = min(1,max(S(4,index))*10)*[1 0 0];
             
             connImg(index_) = 1;
             
             % Connected-component analysis 
             % check an error related to "eqlabel"
             [conn, ids, count_, indices ] = test4Connectivity(connImg);
             if visflag
             figure(1), 
             plot(Yind(index),Xind(index),'.','Color',randcolor,'MarkerSize',5);                
             figure(40), imshow(connImg');
             figure(41),imshow(conn'/max(conn(:))); colormap('colorcube'); axis equal;
             end
             
             flag = 0;
             
             if ~isempty(ids)
                offsets = [];             
                c0 = [];
                for t = 1: length(count_)  
                    if visflag
                    blobImg = zeros(size(depth));
                    end
                    if count_(t) > thre_memberSize
                        if visflag                               
                            figure(11), 
                            plot(Yind(index),Xind(index),'.','Color',randcolor,'MarkerSize',5); hold on;

                            figure(99), 
                            scatter3(A(1,index),A(2,index),A(3,index),7,randcolor','filled'); axis equal;

                            flag = 1;
                        end
                        
                        [dummy,whichcell] = intersect(index_ , indices{ids(t)});    
                        blobImg(index(whichcell)) = t;                      
                        if ~isempty(whichcell)   
                            center = round(mean([Xind(index(whichcell)); Yind(index(whichcell))],2));
                            if visflag
                            figure(100+t), hold off; 
                            imshow(blobImg'); hold on;
                            figure(1), 
                            plot(center(2), center(1),'o','LineWidth',2);
                            end
                            
                            [yind_s, xind_s] = ind2sub(size(conn),indices{ids(t)});
                            center_s = round(mean([xind_s;yind_s],2));
                            if visflag
                             figure(100+t), 
                             plot(center_s(2)*5, center_s(1)*5,'g+','LineWidth',2);
                            end
                            % extract min-max points & 8-directional boundary points
                            [minyind,minyloc] = min(yind_s);
                            [maxyind,maxyloc] = max(yind_s);
                            [maxxind,maxxloc] = max(xind_s);
                            [minxind,minxloc] = min(xind_s);
                            minmax_box = unique([xind_s(minyloc) yind_s(minyloc); 
                                          xind_s(maxyloc) yind_s(maxyloc);   
                                          xind_s(minxloc) yind_s(minxloc);   
                                          xind_s(maxxloc) yind_s(maxxloc);   ],'rows');
                            if visflag
                            figure(100+t), plot(minmax_box(:,2)*5,minmax_box(:,1)*5, 'b.','MarkerSize',7);
                            end
                            pts = zeros(8,2);
                            Pts = zeros(3,8);
                            Bbox = zeros(3,size(minmax_box,1));
                            
                            % direction 1-5
                            h_ = size(conn,1);
                            centerind = center_s(1)*h_ + center_s(2);
                            t_ind1 = find(conn(:,center_s(1))==ids(t));
                            if ~isempty(t_ind1)
                                % direction 3-7
                                t_ind3 = find(conn(center_s(2),:)==ids(t));     
                                if  ~isempty(t_ind3)
                                    idx_2 = centerind:(h_-1):numel(conn);
                                    idx_6 = centerind:(1-h_):h_*max((center_s(1)-(h_-center_s(2))),1);
                                    idx_4 = centerind:(h_+1):numel(conn);
                                    idx_8 = centerind:(-1-h_):h_*max((center_s(1)-(h_-center_s(2))),1);        
                                    t_ind2 = find(conn(idx_2)==ids(t),1,'last');                              
                                    t_ind6 = find(conn(idx_6)==ids(t),1,'last');  
                                    t_ind4 = find(conn(idx_4)==ids(t),1,'last');  
                                    t_ind8 = find(conn(idx_8)==ids(t),1,'last');
                                    
                                    if  ~isempty(t_ind2) && ~isempty(t_ind6) && ~isempty(t_ind4) && ~isempty(t_ind8)
                                        pts(1,:) = [min(t_ind1) center_s(1)];
                                        pts(5,:) = [max(t_ind1) center_s(1)];
                                        pts(3,:) = [center_s(2) min(t_ind3)];
                                        pts(7,:) = [center_s(2) max(t_ind3)];
                                        % direction 2-6 figure(2),
                                       
                                        [t_1, t_2] = ind2sub(size(conn),idx_2(t_ind2)); 
                                        pts(2,:) = [t_1, t_2] ;
                                        [t_1, t_2] = ind2sub(size(conn),idx_6(t_ind6)); 
                                        pts(6,:) =  [t_1, t_2];                          
                                        % direction 4-8
                                        [t_1, t_2] = ind2sub(size(conn),idx_4(t_ind4)); 
                                        pts(4,:) = [t_1, t_2] ;                         
                                        [t_1, t_2] = ind2sub(size(conn),idx_8(t_ind8)); 
                                        pts(8,:) =  [t_1, t_2];   
                                       
                                        [dummy,whichcell_] = intersect(index_ , sub2ind(size(conn), pts(:,1), pts(:,2)));   
                                        Pts(1,:) = depth(index(whichcell_))*0.001;
                                        Pts(2,:) = -Yind_c(index(whichcell_))*IMCY/fy.*Pts(1,:);
                                        Pts(3,:) = -Xind_c(index(whichcell_))*IMCX/fx.*Pts(1,:); 
                                        
                                        [dummy,whichcell__] = intersect(index_ , sub2ind(size(conn), minmax_box(:,2), minmax_box(:,1)));   
                                        Bbox(1,:) = depth(index(whichcell__))*0.001;
                                        Bbox(2,:) = -Yind_c(index(whichcell__))*IMCY/fy.*Bbox(1,:);
                                        Bbox(3,:) = -Xind_c(index(whichcell__))*IMCX/fx.*Bbox(1,:);
                                        
                                        if visflag
                                            figure(100+t),   plot(pts(:,1)*5,pts(:,2)*5,'r.'); hold on;

                                            figure(2),
                                            scatter3( depth(index(whichcell))*0.001, -Yind_c(index(whichcell))*IMCY/fy.*depth(index(whichcell))*0.001,...
                                                -Xind_c(index(whichcell))*IMCX/fx.*depth(index(whichcell))*0.001, 5 ,'r','filled');
                                            scatter3( Pts(1,:), Pts(2,:), Pts(3,:), 5 ,'k','filled');
                                            scatter3( Bbox(1,:), Bbox(2,:), Bbox(3,:), 10 ,'k','filled');
                                        end
                                    end
                                end
                            end
                            
                            z_mean = depth(center(2), center(1));
                            
                            % re-fit to a plane
                            % use svd and find the principal axes 
                            [c, ins] = estimatePlane_useall( Xind_c(index(whichcell)), Yind_c(index(whichcell)), ZZ(index(whichcell)));
                             
                            if ~isempty(c) && numel(ins) > 5
                                
                                if isempty(offsets)
                                    c0 = [c' 1].*[IMCX/fx IMCY/fy 1 1 ];
                                    offsets = [ids(t) 0 c0];
                                else
                                    c_ = [c' 1].*[IMCX/fx IMCY/fy 1 1 ];
                                    d = abs(c0(3)-c_(3))/sqrt(sum(c0(1:2).^2) + 1);
                                    offsets = [offsets; ids(t) d c_];
                                end
                                
                                % transformation to the body frame
                                n_ = c([2 1 3]);
                                n_ = -n_/norm(n_);
                                n__ = [n_(3) -n_(1) -n_(2)]/norm(n_);
                                
                                PlaneID = PlaneID + 1;
                                Planes{PlaneID} = struct('Normal', n__ , 'Points', [Pts Bbox]); 
                               
                                % draw the normal on 3D plot
                                yc = (center(1)-IMCX)/fx*z_mean*0.001;
                                xc = (center(2)-IMCY)/fy*z_mean*0.001;
                                zc = z_mean*0.001;
                                figure(2), plot3(zc, -xc, -yc,'ko');

                                offc = 0.1*n_ + [xc yc zc]';

                                figure(2), plot3([zc offc(3)],  -[xc offc(1)],-[yc offc(2)],'-','Color',[0 0.6 0],'LineWidth',3); 
                            end
                            
                        end
                    end
                end    
                
                
             end             
                         
      
         end
     end
        
     toc,
      
    end
   
end

fclose(f_depth);
fclose(f_rgb);


