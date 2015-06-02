function [pcx, pcy, pcz, r, g, b] = meshToRGBCloud(meshRaw, meshmeta, rgbImg, rgbmeta)
% by Bhoram Lee
% 
% Note: 
% 
% << Input >> 
% meshRaw - the mesh raw data (2 dim)
pcx = [];
pcy = [];
pcz = [];
r = [];
g = [];
b = [];

ONESCAN_ = meshmeta.dims(2);    % single scan resolution 
v_angles = meshmeta.rfov(1):(0.25*pi/180):resetParam.rfov(2);   
if length(v_angles) > ONESCAN_
    v_angles = v_angles(1:ONESCAN_);
end
s_angles = meshmeta.a;


meshRaw(meshRaw>3) = 0;             % clamp on ranges
meshRaw(meshRaw<0.5) = 0;
[mesh_, s_, v_, nzcols] = scan2DepthImg_spherical0( meshRaw, s_angles, v_angles); % remove repeated measure   

if isempty(nzcols) 
    return;
end

% Let's check "motion"
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

TL = reshape(meshmeta.tfL16{nzcols(1)},4,4)';
Ccbl = TL(1:3,1:3);
Tcbl = TL(1:3,4);% + [0; 0; 0.07];
P = Ccbl*[ X0(:)'; Y0(:)'; Z0(:)' ] + repmat(Tcbl,1,numel(X0));

if visflag > 0    
    figure(visflag), hold off;
    showPointCloud(P(1,:),P(2,:),P(3,:),[0.5 0.5 0.5],'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up','MarkerSize',2);
    hold on;  
end

% 3d point clouds
pcx = P(1,:);
pcy = P(2,:);
pcz = P(3,:);

% Rotation
%if strcmp(rgbmeta.name,'c920')   
fc = [ 1687.77049851744; 1577.56403609168  ]; %-- Focal length:    
cc = [  1156.42684871619  ;  691.26801883923 ];  %-- Principal point:
%elseif
%end
TR= reshape(rgbmeta.tfL16{nzcols(1)},4,4)';
Ccbr = TR(1:3,1:3);
Tcbr = TR(1:3,4);% + [0; 0; 0.07];
X_ = Ccbr'*([pcx; pcy; pcz] - Tcbr*ones(1,length(dvalidInd)));

x_ = round(fc(1)*X_(1,:)./X_(3,:) + cc(1));
y_ = round(fc(2)*X_(2,:)./X_(3,:) + cc(2));

rvalidInd = (x_>=1) & (x_<=size(rgbImg,2)) & (y_>=1) & (y_<=size(rgbImg,1));

x_ = x_(rvalidInd);
y_ = y_(rvalidInd);

r_ = rgbImg(:,:,1);
g_ = rgbImg(:,:,2);
b_ = rgbImg(:,:,3);

xgrid = ones(NUMS_,1)*(1:NUMV_);
ygrid = (1:NUMS_)'*ones(1,NUMV_);
ind = sub2ind(size(mesh_),ygrid(dvalidInd(rvalidInd)),xgrid(dvalidInd(rvalidInd)));
ind_ = sub2ind(size(rgbImg),y_,x_);
r = double(r_(ind_));
g = double(g_(ind_));
b = double(b_(ind_));

pcx = pcx(rvalidInd);
pcy = pcy(rvalidInd);
pcz = pcz(rvalidInd);


    
end








