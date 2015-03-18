function [pcx, pcy, pcz, r, g, b] = depthToCloud(depth, rgbImg)
% depthToPointCloud.m - Convert depth image into 3D point cloud
% Original Author: Liefeng Bo and Kevin Lai
% Modified by Bhoram Lee
% 
% Note: 
% 
% << Input >> 
% depth - the depth image
%
% << Output >>
% pcloud - the point cloud, where each channel is the x, y, and z euclidean coordinates respectively. 
%          Missing values are NaN.
topleft = [1 1];
depth= double(depth-30);
depth(depth == 0) = nan;
r = [];
g = [];
b = [];

% Refine the depth camera parameters using provided parameters
[imh, imw] = size(depth);
center = [ 258.422487561914693 ; 202.487139940005989 ];
fx = 364.457362485643273; 
fy = 364.542810626989194 ;

% convert depth image to 3d point clouds
xgrid = ones(imh,1)*(1:imw) + (topleft(1)-1) - center(1);
ygrid = (1:imh)'*ones(1,imw) + (topleft(2)-1) - center(2);
pcx = xgrid.*depth/fx;
pcy = ygrid.*depth/fy;
pcz = depth;

if nargin > 1

    dvalidInd = find(~isnan(depth));

    pcx = xgrid(dvalidInd).*depth(dvalidInd)/fx;
    pcy = ygrid(dvalidInd).*depth(dvalidInd)/fy;
    pcz = depth(dvalidInd);

    % Rotation
    load exParams.mat
    %-- Focal length:
    fc = [ 1049.331752604831308 ; 1051.318476285322504 ];
    %-- Principal point:
    cc = [ 956.910516428015740 ; 533.452032441484675 ];

    X_ = R*[pcx'; pcy'; pcz'] + T*ones(1,length(dvalidInd));

    x_ = round(fc(1)*X_(1,:)./X_(3,:) + cc(1));
    y_ = round(fc(2)*X_(2,:)./X_(3,:) + cc(2));
    
    rvalidInd = (x_>=1) & (x_<=size(rgbImg,2)) & (y_>=1) & (y_<=size(rgbImg,1));
  
    x_ = x_(rvalidInd);
    y_ = y_(rvalidInd);

    r_ = rgbImg(:,:,1);
    g_ = rgbImg(:,:,2);
    b_ = rgbImg(:,:,3);

    xgrid = ones(imh,1)*(1:imw);
    ygrid = (1:imh)'*ones(1,imw);
    ind = sub2ind(size(depth),ygrid(dvalidInd(rvalidInd)),xgrid(dvalidInd(rvalidInd)));
    ind_ = sub2ind(size(rgbImg),y_,x_);
    r = double(r_(ind_));
    g = double(g_(ind_));
    b = double(b_(ind_));
    
    pcx = pcx(rvalidInd);
    pcy = pcy(rvalidInd);
    pcz = pcz(rvalidInd);
    
end








