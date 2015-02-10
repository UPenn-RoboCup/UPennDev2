close all;
clear all;

DEPTH_W = 512;
DEPTH_H = 424;
DEPTH_MAX = 2000;%8000;
DEPTH_MIN = 200;
IMCX = DEPTH_W/2;
IMCY = DEPTH_H/2;

RGB_W = 1920;
RGB_H = 1080;

Yind = repmat(1:DEPTH_W,1,DEPTH_H); % index in 1D array 
Xind = kron(1:DEPTH_H,ones(1,DEPTH_W)); 

Xind_c = (reshape(Xind,DEPTH_W,DEPTH_H)-IMCX)/IMCX; % index in 2D array representation
Yind_c = (reshape(Yind,DEPTH_W,DEPTH_H)-IMCY)/IMCY; 

datestamp = '12.04.2014.09.30.46';
f_depth = fopen(sprintf('Data/k2_depth_r_%s.log', datestamp));
f_rgb = fopen(sprintf('Data/k2_rgb_r_%s.log', datestamp));
% f_ir = fopen(sprintf('Data/k2_ir_r_%s.log', datestamp));

ilog = 0;
while ~feof(f_depth)
    
    depthRaw = fread(f_depth, [DEPTH_W, DEPTH_H], '*single');
    if isempty(depthRaw)
        break;
    end
    
    ilog = ilog + 1;
    
    fid = fopen(sprintf('Data/k2_rgb_m_%s.log',datestamp));
    rgbMeta = fread(fid,Inf,'*uint8');
    fclose(fid);
    rgbMeta = msgpack('unpacker',rgbMeta,'uint8');
       % Grab the Depth logged information
    f_rgb = fopen(sprintf('Data/k2_rgb_r_%s.log', datestamp));
    
    meta = rgbMeta{ilog};
    rgbJPEG = fread(f_rgb, meta.rsz, '*uint8');
    rgb_img0 = djpeg(rgbJPEG);
    rgb_img(:,:,1) = rgb_img0(:,:,3);
    rgb_img(:,:,2) = rgb_img0(:,:,2);
    rgb_img(:,:,3) = rgb_img0(:,:,1);

    
 
    % Initialize mask
    mask = ones(DEPTH_W, DEPTH_H);
    mask(depthRaw(:) <= DEPTH_MIN) = 0;
    mask(depthRaw(:) >= DEPTH_MAX) = 0;

    % Filter depth
    depth = depthRaw.*mask;
    depth = medfilt2(depth,[7 7]);
    validInd = find(depth(:)>200);
    mask = zeros(DEPTH_W, DEPTH_H);
    mask(validInd) = 1;
    depth = depth.*mask;
    
    % resize?
    % reIm1 = imresize(depthRaw,0.5);
    
    validInd = validInd(20:20:end);% Subsample (for drawing)
    data = [  (Yind(validInd)-IMCY)'/IMCY (Xind(validInd)-IMCX)'/IMCX 3000./depth(validInd)];

    figure(1), imshow((depth'-200)/(2000-200)); axis equal; hold on;
    %figure(2), scatter3(data(:,2),data(:,1),data(:,3),5,'k','filled'); hold on;
    
    % inverse depth
    ZZ = double(3000./depth);
    wndsz = 5^2; % (odd nember)^2 
    tic,
    [N] = mexComputeGeometry('normal', Xind_c, Yind_c, ZZ, logical(mask),wndsz);
    toc,

    validNormal = find( sum(N.^2,1) > 0);

    figure(2), hold off; 
    scatter3((Xind(validNormal)-IMCX)/IMCX,(Yind(validNormal)-IMCY)/IMCY,...
                ZZ(validNormal),10,N(:,validNormal)','filled');
            axis equal;axis([-1 1 -1 1 1.5 3])
   
    
    %[model, inliers]  = findPlane_2paramRansac(data);
    %figure(2), scatter3(data(inliers,1),data(inliers,2),data(inliers,3),5,'k','filled'); hold on;

    L = sphericalKMeans(N(:,validNormal),3);
    
    figure(1), 
    i1 = L{1}(:,1);
    scatter(Yind(validNormal(i1)),Xind(validNormal(i1)),5,'r','filled');
    i2 = L{2}(:,1);
    scatter(Yind(validNormal(i2)),Xind(validNormal(i2)),5,'g','filled');
    i3 = L{3}(:,1);
    scatter(Yind(validNormal(i3)),Xind(validNormal(i3)),5,'b','filled');
    
    pause;

end

fclose(f_depth);



