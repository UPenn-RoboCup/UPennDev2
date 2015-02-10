close all;
clear all;

DEPTH_W = 512;
DEPTH_H = 424;
DEPTH_MAX = 4000;%8000;
DEPTH_MIN = 200;
IMCX = DEPTH_W/2;
IMCY = DEPTH_H/2;

RGB_W = 1920;
RGB_H = 1080;

Yind = repmat(1:DEPTH_W,1,DEPTH_H); % index in 1D array 
Xind = kron(1:DEPTH_H,ones(1,DEPTH_W)); 

Xind_c = (reshape(Xind,DEPTH_W,DEPTH_H)-IMCX)/IMCX; % index in 2D array representation
Yind_c = (reshape(Yind,DEPTH_W,DEPTH_H)-IMCY)/IMCY; 

% datestamp = '12.04.2014.09.30.46';
datestamp = '12.04.2014.09.31.54';
f_depth = fopen(sprintf('Data/k2_depth_r_%s.log', datestamp));
f_rgb = fopen(sprintf('Data/k2_rgb_r_%s.log', datestamp));
% f_ir = fopen(sprintf('Data/k2_ir_r_%s.log', datestamp));
fid = fopen(sprintf('Data/k2_rgb_m_%s.log',datestamp));
rgbMeta = fread(fid,Inf,'*uint8');
fclose(fid);
rgbMeta = msgpack('unpacker',rgbMeta,'uint8');

saveflag = 0;

ilog = 0;
prevm = [];
L2 = [];
while ~feof(f_depth)
    
    depthRaw = fread(f_depth, [DEPTH_W, DEPTH_H], '*single');
    if isempty(depthRaw)
        break;
    end
    
    ilog = ilog + 1;
    
    meta = rgbMeta{ilog};
    rgbJPEG = fread(f_rgb, meta.rsz, '*uint8');
    rgb_img0 = djpeg(rgbJPEG);
    rgb_img(:,:,1) = rgb_img0(:,:,3);
    rgb_img(:,:,2) = rgb_img0(:,:,2);
    rgb_img(:,:,3) = rgb_img0(:,:,1);
    
    figure(90), imshow(rgb_img);
    if saveflag 
        imwrite(rgb_img,strcat('Figures/rgb_',datestamp((end-7):end),'_',int2str(ilog),'.png'));
    end
    % Initialize mask
    mask = ones(DEPTH_W, DEPTH_H);
    mask(depthRaw(:) <= DEPTH_MIN) = 0;
    mask(depthRaw(:) >= DEPTH_MAX) = 0;
    
    
    if saveflag
        figure(92),  imshow((depthRaw'-DEPTH_MIN)/(DEPTH_MAX-DEPTH_MIN));
        F = getframe(gca);
        imwrite(F.cdata,strcat('Figures/rawdepthgca_',datestamp((end-7):end),int2str(ilog),'.png'));
    end
    % Filter depth
    depth = depthRaw.*mask;
    
    depth = medfilt2(depth,[7 7]);
    
    validInd = find(depth(:)>DEPTH_MIN);
    mask = zeros(DEPTH_W, DEPTH_H);
    mask(validInd) = 1;
    depth = depth.*mask;
    
    % resize?
    % reIm1 = imresize(depthRaw,0.5);
    
    validInd = validInd(20:20:end);% Subsample (for drawing)
    data = [  (Yind(validInd)-IMCY)'/IMCY (Xind(validInd)-IMCX)'/IMCX 3000./depth(validInd)];

    figure(1), imshow((depth'-DEPTH_MIN)/(DEPTH_MAX-DEPTH_MIN)); axis equal; hold on;
    %figure(2), scatter3(data(:,2),data(:,1),data(:,3),5,'k','filled'); hold on;
  
    
    
    % inverse depth
    ZZ = double(3000./depth);
    wndsz = 7^2; % (odd nember)^2 
    tic,
    [N, S] = mexComputeGeometry('normal', Xind_c, Yind_c, ZZ, logical(mask),wndsz);
    toc,

    %largestSVal = max(S,[],1);
    smallestSVal = min(S,[],1);
    validNormal = intersect(intersect(find( sum(N.^2,1) > 0), find(smallestSVal > 0)),find(smallestSVal < 0.015));
    

    figure(2), hold off; 
    scatter3((Xind(validNormal)-IMCX)/IMCX,(Yind(validNormal)-IMCY)/IMCY,...
                ZZ(validNormal),10,N(:,validNormal)','filled');
            axis equal;axis([-1 1 -1 1 1.5 3])
   
    
    %[model, inliers]  = findPlane_2paramRansac(data);
    %figure(2), scatter3(data(inliers,1),data(inliers,2),data(inliers,3),5,'k','filled'); hold on;
    
    %tic,
    %[L, L2, Me, Mo] = sphericalKMeansMode(N(:,validNormal),3, prevm);
    %toc,    
    % prevm = [];
    %[leng, idx] = max(I2length);
    %if  leng > 1000
    %    prevm = [prevm Mo(1:3,idx)]; 
    %end
    
    
    u = randn(3,3);
    u(:,1) = u(:,1)/norm(u(:,1));
    u(:,2) = u(:,2)/norm(u(:,2));
    u(:,3) = u(:,3)/norm(u(:,3));
    tic,
    [L, Me] = mexSKMeans(N(:,validNormal),int32(3),u);
    toc,
    
    % for the largest cluster, 
    % pick some of the points connected with the same cluster
    Ilength = zeros(3,1);
    for j=1:3
        I = L{j}(:,1)+1; 
        Ilength(j) = length(I); 
    end
    
    [nMaxClust, indMaxClust] = max(Ilength);
    tt = randperm(nMaxClust,15);
    tt_ = validNormal(L{indMaxClust}(tt,1)+1);
    
    [tt_y, tt_x] = ind2sub([DEPTH_W DEPTH_H], tt_);     
    
    
    figure(1), 
    color = {'c','m','y'};
    color2 = {'b','r','g'};
    I2length = zeros(3,1);
    for j=1:3
        I = L{j}(:,1)+1; 
        Ilength(j) = length(I); 
        scatter(Yind(validNormal(I)),Xind(validNormal(I)),3,color{j});
           
        if ~isempty(L2)
            I2 = L2{j}(:,1); 
            I2length(j) = length(I2); 
             if I2length(j) > 1000,
                scatter(Yind(validNormal(I2)),Xind(validNormal(I2)),5,color2{j},'filled');
            end       
        end
    end
   
   
    
    if saveflag
        figure(1), 
        F = getframe(gca);
        imwrite(F.cdata,strcat('Figures/normal_clustering_',datestamp((end-7):end),'_',int2str(ilog),'.png'));

        figure(92), 
        F = getframe(gca);
        imwrite(F.cdata,strcat('Figures/median_filtered_',datestamp((end-7):end),'_',int2str(ilog),'.png'));
    end
      

   
end

fclose(f_depth);
fclose(f_rgb);


