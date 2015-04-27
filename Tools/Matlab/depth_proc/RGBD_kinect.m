function []=RGBD_kinect(rgb_img,depthRaw)
%Calibration result for IR camera
%-- Focal length:
IR_fc = [ 364.457362485643273 ; 364.542810626989194 ];
%-- Principal point:
IR_cc = [ 258.422487561914693 ; 202.487139940005989 ];
%-- Distortion coefficients:
IR_kc = [ 0.098069182739161 ; -0.249308515140031 ; 0.000500420465085 ; 0.000529487524259 ; 0.000000000000000 ];

%Calibration result for RGB camera
%-- Focal length:
RGB_fc = [ 1049.331752604831308 ; 1051.318476285322504 ];
%-- Principal point:
RGB_cc = [ 956.910516428015740 ; 533.452032441484675 ];
%-- Distortion coefficients:
RGB_kc = [ 0.026147836868708 ; -0.008281285819487 ; -0.000157005204226 ; 0.000147699131841 ; 0.000000000000000 ];


% IR_T=[-123.442251 ; -47.091687 ;460.089520];
% IR_R=[-0.003212 0.999999 0.003211; 0.998991 0.003353 -0.044785; -0.044795 0.003064 -0.998991];
% 
% RGB_T=[-68.620724 ; -44.328531 ;460.280951];
% RGB_R=[0.002452 0.999994 -0.002327; 0.999039 -0.002552 -0.043749;-0.043754 -0.002218 -0.999040];

%for img25
RGB_T=[-97.582353 	; -37.985984 ;	 362.524877 ];
RGB_R=[0.089245 	 0.819926 	 -0.565471;
                               0.989721 	 -0.136700 	 -0.042011;
                               -0.111746 	 -0.555909 	 -0.823698 ];
IR_T=[-151.806218 ;	 -40.235688 ;	 362.459755 ];
IR_R=[0.083127 	 0.825976 	 -0.557543;
                               0.990822 	 -0.128346 	 -0.042412;
                               -0.106590 	 -0.548900 	 -0.829064 ];


R=RGB_R*IR_R';
T=RGB_T-R*IR_T;
%T=[0;0;0];


RGB_img=rgb_img;
depth_img=depthRaw';


% X_list=zeros(190459,1);
% Y_list=zeros(190459,1);
% depth_list=zeros(190459,1);

theta=-10;

points=zeros(512*424,6);
k=1;



figure(3)
tic;
for i=1:5:512
%     disp(i);
    for j=1:5:424
Z=depth_img(j,i);
if(Z>0 && Z<7000)
X=(i-IR_cc(1))/IR_fc(1)*Z;
Y=(j-IR_cc(2))/IR_fc(2)*Z;
P_RGB=R*[X;Y;Z]+T;
nX=P_RGB(1);
nY=P_RGB(2);
nZ=P_RGB(3);
x=nX/nZ*RGB_fc(1)+RGB_cc(1);
y=nY/nZ*RGB_fc(2)+RGB_cc(2);
if(x>1 && x<1920 && y>1  &&y<1080)
color=[rgb_img(round(y),round(x),1);rgb_img(round(y),round(x),2);rgb_img(round(y),round(x),3);];
color=double(color)/255;
Plot_R=[0 0 1 0;-1 0 0 0;0 -1 0 0;0 0 0 1];
v=Plot_R*[X;Y;Z;1];
% plot3(v(1),v(2),v(3),'.','Color',color,'MarkerSize',9);
% hold on
points(k,:)=[v(1) v(2) v(3) color'];
k=k+1;
end

end
    end
end
toc;
points=points(1:k,:);
%%
scatter3(points(10:10:end,1),points(10:10:end,2),points(10:10:end,3),6,points(10:10:end,4:6));
xlabel('X');
ylabel('Y');
zlabel('Z');
% axis([-2000 2000 -3000 2000 -2000 6000])
axis equal
view([-90 0])











