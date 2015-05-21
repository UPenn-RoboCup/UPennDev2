function RGBD_lidar(kinect,lidar)
%-- Focal length:
RGB_fc = [ 1049.331752604831308 ; 1051.318476285322504 ];
%-- Principal point:
RGB_cc = [ 956.910516428015740 ; 533.452032441484675 ];
k=1;

mesh=lidar.meshRaw';
alpha=zeros(360,80);
beta=zeros(360,80);
alpha=repmat((0:90/359:90)',1,80);
beta=repmat(-40:80/79:40,360,1);
X=mesh.*cosd(alpha).*sind(beta);
Y=mesh.*sind(alpha);
Z=(mesh.^2-X.^2-Y.^2).^0.5;
new_plot=zeros(360,80,3);

H_K2W=reshape(kinect.metad.tr,4,4)'*[0 0 1 0;-1 0 0 0;0 -1 0 0;0 0 0 1];
H_L2W=[0 0 1 0;-1 0 0 0;0 -1 0 1;0 0 0 1];
H_W2K=zeros(4,4);
H_W2K(1:3,1:3)=H_K2W(1:3,1:3)';
H_W2K(1:3,4)=-H_K2W(1:3,1:3)'*H_K2W(1:3,4);
H_W2K(4,4)=1;
% tic
for i=1:1:360
    for j=1:1:80
        x=X(i,j);
        y=Y(i,j);
        z=Z(i,j);
        v=[x;y;z;1];
        nv=H_W2K*H_L2W*v;
        nx=nv(1);
        ny=nv(2);
        nz=nv(3);
        rgbx=nx/nz*RGB_fc(1)+RGB_cc(1);
        rgby=ny/nz*RGB_fc(2)+RGB_cc(2);
        if(1<rgbx && rgbx<1920 && rgby>1 && rgby<1080)
            new_plot(i,j,:)=kinect.rgb_img(round(rgby),round(rgbx),:);
        end
        
    end
end
% toc;
% figure(3)
 new_plot=new_plot/255;
% imagesc(new_plot);
% pause(1)


points=zeros(360*80,6);
k=1;

figure(4)
for i=1:5:360
%     disp(i);
    for j=1:2:80
        x=X(i,j);
        y=Y(i,j);
        z=Z(i,j);
        v=H_L2W*[x;y;z;1];
        color=[new_plot(i,j,1);new_plot(i,j,2);new_plot(i,j,3)];
        if(v(1)<4 && any(color))
%         plot3(v(1),v(2),v(3),'.','Color',color,'MarkerSize',9);
%         hold on
        points(k,:)=[v(1),v(2),v(3),color'];
        k=k+1;
        end
    end
end
 points=points(1:k,:);
%%
step=1;
scatter3(points(1:step:end,1),points(1:step:end,2),points(1:step:end,3),10,points(1:step:end,4:6));
axis equal
view(-90,30)
