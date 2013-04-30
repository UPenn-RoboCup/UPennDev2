clear all;
SetMagicPaths;
DefineLidarMsgs;
DefineEncoderMsg;


ipcAPIConnect;
pointCloudTypeName = 'PointCloud3DColorDoubleRGBA';
pointCloudMsgName = ['pointCloud' VisMarshall('getMsgSuffix',pointCloudTypeName)];
pointCloudFormat  = VisMarshall('getMsgFormat',pointCloudTypeName);
ipcAPIDefine(pointCloudMsgName,pointCloudFormat);

lidarPointsTypeName = 'PointCloud3DColorDoubleRGBA';
lidarPointsMsgName = ['lidar1Points' VisMarshall('getMsgSuffix',lidarPointsTypeName)];
lidarPointsFormat  = VisMarshall('getMsgFormat',lidarPointsTypeName);
ipcAPIDefine(lidarPointsMsgName,lidarPointsFormat);


poseMsgName = ['Robot0' VisMarshall('getMsgSuffix','Pose3D')];
poseMsgFormat  = VisMarshall('getMsgFormat','Pose3D');
ipcAPIDefine(poseMsgName,poseMsgFormat);


mapMsgName = 'map2de_map2d';
mapMsgFormat = VisMap2DSerializer('getFormat');

ipcAPIConnect;
ipcAPIDefine(mapMsgName,mapMsgFormat);

lidarMsgName = 'Robot0/Lidar0';
ipcAPISubscribe(lidarMsgName);


nPointsUtm=1081;
resUtm = 0.25;
anglesUtm=((0:resUtm:(nPointsUtm-1)*resUtm)-135)'*pi/180;

cosines = cos(anglesUtm);
sines   = sin(anglesUtm);

res = 0.05;
invRes = 1/res;

xmin = -5;
ymin = -5;
xmax = 25;
ymax = 25;
zmin = 0;
zmax = 5;

sizex = (xmax - xmin) * invRes;
sizey = (ymax - ymin) * invRes;

mapp = [];
mapType = 'uint8';
map  = zeros(sizex,sizey,mapType);
emap = [];


mape.x   = xmin;
mape.y   = ymin;
mape.z   = 0;
mape.yaw = 0;
mape.res = 0.05;
mape.map.sizex = size(map,1);
mape.map.sizey = size(map,2);
mape.map.data  = 127*ones(size(map),'uint8');

content = VisMap2DSerializer('serialize',mape);
ipcAPIPublishVC(mapMsgName,content);

ScanMatch2D('setBoundaries',xmin,ymin,xmax,ymax);
ScanMatch2D('setResolution',res);




yaw =(0.5)/180*pi; %1.5
x=0;
y=0;
z=0;

positions = zeros(3,100000);

rotCntr = 1;





heightData.sizex=sizex;
heightData.sizey=sizey;
heightData.data = zeros(sizex,sizey,'single');

props.resx = res;
props.resy = res;
props.interpx = 0;
props.interpy = 0;
props.lineSepX = 20.0;
props.lineSepY = 20.0;

surface.heightData = heightData;
surface.props = props;

%content = VisSurface2DSerializer('serialize',surface);
%ipcAPIPublishVC(surfMsgName,content);

hHough = [];
hAccum = [];

hhPrev = [];


freshLidar = 0;
ranges = [];

i=0;
tic

%load the initial map
load ~/Desktop/MappingLogs/levine5th_04.26.10/levine5th_hall.mat

mapp=zeros(size(map),mapType);
%mapp=map2d;

while(1)
  msgs = ipcAPI('listen',5);
 
  nmsgs = length(msgs);
  
  for mi=1:nmsgs
    msg = msgs(mi);
    switch msg.name
      case lidarMsgName
        lidarScan = MagicLidarScanSerializer('deserialize',msg.data);
        ranges = double(lidarScan.ranges)';
        fprintf('.');
        freshLidar=1;
    end
  
    if (freshLidar == 0)
      continue;
    end

    freshLidar=0;

    i=i+1;
    pose = [0 0 0];

    indGood = ranges >0.25;

    xs = ranges.*cosines;
    ys = ranges.*sines;
    zs = zeros(size(xs));

    xsss=xs(indGood);
    ysss=ys(indGood);
    zsss=zs(indGood);
    onez=ones(size(xsss));





    xsh = xsss(1:5:end);
    ysh = ysss(1:5:end);

    %[h_trans angles rhos]

     a_center = 0; %pi/2;
    a_range  = pi/2;
    a_res = pi/200;
    r_center =0;
    r_range = 5;
    r_res   = 0.10;


    [h_trans] = HoughTransformAPI(xsh,ysh,a_center,a_range,a_res,r_center, r_range, r_res);

    h_trans(h_trans<10) = 0;
    hh = sum(h_trans,1);



    if isempty(hhPrev)
      hhPrev = hh;
    end

    cshift = -5:5;
    clen = length(cshift);
    cvals= zeros(clen,1);
    for s=1:clen;
      cvals(s) = sum(hh'.*(circshift(hhPrev',cshift(s))));
    end

    [cmax cimax] = max(cvals);
    hhPrev = hh;

    %toc


    if (isempty(mapp))
      T = rotz(yaw);
      X = [xsss'; ysss';zsss';onez'];
      Y=T*X;

      xss = Y(1,:)' +x;
      yss = Y(2,:)' +y;
      zss = Y(3,:)' +z;

      xis = ceil((xss - xmin) * invRes);
      yis = ceil((yss - ymin) * invRes);
      indGood = (xis > 1) & (yis > 1) & (xis < sizex) & (yis < sizey);

      inds = sub2ind(size(map),xis(indGood),yis(indGood));
      mapp(inds) = 100;

      %emap = zeros(size(map),'uint8');
      %emap(245:255,245:255) = 249;

      continue;
    end



    %fprintf(1,'------------------------');

    nyaw= 21;
    nxs = 11;
    nys = 11;

    dyaw = 0.25/180.0*pi;
    dx   = 0.02;
    dy   = 0.02;

    aCand = (-10:10)*dyaw+yaw ; %+ (-cshift(cimax))*a_res;
    xCand = (-5:5)*dx+x;
    yCand = (-5:5)*dy+y;

    hits = ScanMatch2D('match',mapp,xsss,ysss,xCand,yCand,aCand);


    [hmax imax] = max(hits(:));
    [kmax mmax jmax] = ind2sub([nxs,nys,nyaw],imax);

    yaw = aCand(jmax);
    x   = xCand(kmax);
    y   = yCand(mmax);
    positions(:,i) = [x;y;yaw];

    T = (trans([x y 0])*rotz(yaw))';
    X = [xsss ysss zsss onez];
    Y=X*T;


    xss = Y(:,1);
    yss = Y(:,2);

    xis = ceil((xss - xmin) * invRes);
    yis = ceil((yss - ymin) * invRes);

    xl = ceil((x-xmin) * invRes);
    yl = ceil((y-ymin) * invRes);


    %tic
    [eix eiy] = getMapCellsFromRay(xl,yl,xis,yis);
    %toc

    %plot(eix,eiy,'r.'), hold on
    %plot(xis,yis,'b.'), drawnow, hold off


    cis = sub2ind(size(mapp),eix,eiy);
    mape.map.data(cis) = mape.map.data(cis)+1;

    %emap(cis) = 249;

    ptemp.x = x;
    ptemp.y = y;
    ptemp.theta = yaw;

    if (mod(i,3)==0)
      %publishMaps(mapp,emap,ptemp);
      content = VisMap2DSerializer('serialize',mape);
      ipcAPIPublishVC(mapMsgName,content);
    end

    %{
    p.x =x;
    p.y =y;
    p.yaw = yaw;
    publishPose(p)
    %}
    %imagesc(emap); drawnow
    %imagesc(mapp); drawnow
    %}

    indGood = (xis > 1) & (yis > 1) & (xis < sizex) & (yis < sizey);
    inds = sub2ind(size(map),xis(indGood),yis(indGood));

    mapp(inds)= mapp(inds)+1;

    if (mod(i,50)==0)
      indd=mapp<50 & mapp > 0;
      mapp(indd) = mapp(indd)*0.95;
      mapp(mapp>100) = 100;
    end

    if (mod(i,1)==0)
      vpose = [x y 0.05 pose(1) -pose(2) yaw];
      content = VisMarshall('marshall','Pose3D',vpose);
      ipcAPIPublishVC(poseMsgName,content);
    end

    if (mod(i,10)==0)
      lxs = xss';
      lys = yss';
      lzs = zeros(size(lxs));
      lrs = zeros(size(lxs));
      lgs = ones(size(lxs));
      lbs = 0.5*ones(size(lxs));
      las = ones(size(lxs));
      data = [lxs; lys; lzs; lrs; lgs; lbs; las];

      content = VisMarshall('marshall', lidarPointsTypeName,data);
      ipcAPIPublishVC(lidarPointsMsgName,content);
    end

    %{
    if (mod(i,500)==0)
      inds = find((mapp(:) > 0));
      [subx suby] = ind2sub(size(map),inds);

      surfUpdate.heightData.size = length(inds);
      surfUpdate.xs.size = surfUpdate.heightData.size;
      surfUpdate.ys.size = surfUpdate.heightData.size;

      surfUpdate.heightData.data = single(mapp(inds))/100;
      surfUpdate.xs.data = uint32(subx);
      surfUpdate.ys.data = uint32(suby);

      content = VisSurface2DUpdateSerializer('serialize',surfUpdate);
      ipcAPIPublishVC(surfUpdateMsgName,content);
    end
    %}

    if (mod(i,10)==0)

      %set(hMap,'cdata',mapp');

      inds = find((mapp(:) > 50));
      mape.map.data(inds) = 0;
      [subx suby] = ind2sub(size(map),inds);

      %position
      vxs = subx'*res + xmin;
      vys = suby'*res + ymin;
      vzs = 0.01*ones(size(vxs));

      %color information
      vrs = double((mapp(inds)/100)');
      vgs = 1-vrs;
      vbs = 0.2*ones(size(vxs));
      vas = 0.5*ones(size(vxs));

      data = [vxs; vys; vzs; vrs; vgs; vbs; vas];

      content = VisMarshall('marshall', pointCloudTypeName,data);
      ipcAPIPublishVC(pointCloudMsgName,content);


      %{
      surface.heightData.data = single(mapp)/100;
      content = VisSurface2DSerializer('serialize',surface);
      ipcAPIPublishVC(surfMsgName,content);
      %}
    end

    %set(h,'xdata',xss,'ydata',yss);
    %drawnow;
  end
end


