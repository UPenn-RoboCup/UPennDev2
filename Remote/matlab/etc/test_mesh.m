warning off

global CHEST_LIDAR HEAD_LIDAR


CHEST_LIDAR.lidarrange = 1;
%}

%% Setup the figure
f = figure(1);
clf;
set(gcf,'doublebuffer','off');

subplot(2,2,1);
HEAD_LIDAR.p1 = gca;
HEAD_LIDAR.h1= imagesc( HEAD_LIDAR.ranges );
set(HEAD_LIDAR.h1, 'ButtonDownFcn', @select3D);
set(HEAD_LIDAR.p1,'xtick',[],'ytick',[])
colormap('HOT')

subplot(2,2,2);
CHEST_LIDAR.p1 = gca;
CHEST_LIDAR.h1= imagesc( CHEST_LIDAR.ranges );
set(CHEST_LIDAR.p1,'xtick',[],'ytick',[])

colormap('HOT')

subplot(2,2,3);
HEAD_LIDAR.p2 = gca;
HEAD_LIDAR.h2 = patch('FaceColor','flat','EdgeColor','none',...
     'AmbientStrength',0.4,'SpecularStrength',0.9 );
set(HEAD_LIDAR.p2,'xtick',[],'ytick',[], 'ztick',[])

light('Position',[0 3 3]);
lighting flat

subplot(2,2,4);
CHEST_LIDAR.p2 = gca;
CHEST_LIDAR.h2 = patch('FaceColor','flat','EdgeColor','none',...
     'AmbientStrength',0.4,'SpecularStrength',0.9 );
set(CHEST_LIDAR.p2,'xtick',[],'ytick',[],'ztick',[])

light('Position',[0 3 3]);
lighting flat



drawnow;
