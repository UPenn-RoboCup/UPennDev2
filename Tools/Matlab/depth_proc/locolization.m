function [x,y,head_theta,body_theta,flag]=locolization(depthRaw,metad,rgb_img)
persistent x_old
persistent y_old
persistent head_theta_old
persistent body_theta_old
center=[0,0];
vis=1;

if isempty(x_old)
    x_old=0;
end
if isempty(y_old)
    y_old=0;
end
if isempty(head_theta_old)
    head_theta_old=0;
end
if isempty(body_theta_old)
    body_theta_old=0;
end
meta=[];
uisetting;
[plane_data, ~] = detectPlanes6(depthRaw, meta, ui);
if(length(plane_data)<1)
    
    flag=0;
    x=x_old;
    y=y_old;
    head_theta=head_theta_old;
    body_theta=body_theta_old;
else if(length(plane_data)<2)
        
        flag=1;
        x=-plane_data{1}.Center'*plane_data{1}.Normal;
        y=y_old;
%         disp(plane_data{1}.Center);
        head_theta=acos(-[1,0,0]*plane_data{1}.Normal);
        if(plane_data{1}.Normal(2)<0)
            head_theta=-head_theta;
        end
%         disp(head_theta/pi*180);
        body_theta=head_theta-metad.head_angles(1);
%         disp(metad.head_angles(1));
    else
        
        flag=2;
        if(plane_data{1}.Center(2)>plane_data{2}.Center(2))
            left_plane=plane_data{1};
            right_plane=plane_data{2};
        else
            left_plane=plane_data{2};
            right_plane=plane_data{1};
        end
            x=-left_plane.Center'*left_plane.Normal;
            y=-right_plane.Center'*right_plane.Normal;
            head_theta=acos(-[1,0,0]*right_plane.Normal);
            body_theta=head_theta-metad.head_angles(1);
    end
end
if vis
%     switch flag
%         case 0
%             disp('No plane detected! Updating locolization with odometry.');
%         case 1
%             disp('Only seeing one plane.');
%         case 2
%             disp('Seeing two planes!');
%     end
    figure(12)
    subplot(1,2,1)
    imagesc(fliplr(depthRaw'))
    if vis
    subplot(1,2,2)
    if(flag==1)
        p1=[center(1)-1.5;center(1)+1.5];
        p2=[center(2)+x;center(2)+x];
        plot(p1,p2,'b-','LineWidth',4);
        hold on  
        p1=[0;-0.5*sin(head_theta)];
        p2=[0;0.5*cos(head_theta)];
        plot(p1,p2,'r-','LineWidth',2);
        hold on
        p1=[0;-0.5*sin(body_theta)];
        p2=[0;0.5*cos(body_theta)];
        plot(p1,p2,'g-','LineWidth',2);
        legend('wall','head direction','body direction')
        hold off
        
    end
    if(flag==2)
        p1=[center(1)-1.5;0];
        p2=[0;0];
        plot(p1,p2,'b-','LineWidth',4);
        hold on
        p1=[0;0];
        p2=[0;center(2)-1.5];
        plot(p1,p2,'b-','LineWidth',4);
        hold on
        p1=[-y;-y+0.5*cos(head_theta)];
        p2=[-x;-x+0.5*sin(head_theta)];
        plot(p1,p2,'r-','LineWidth',2);
        hold on
        p1=[-y;-y+0.5*cos(body_theta)];
        p2=[-x;-x+0.5*sin(body_theta)];
        plot(p1,p2,'g-','LineWidth',2);
        legend('','wall','head direction','body direction')
        hold off
    end
    end
    sprintf('x= %f,y=%f,body theta= %f,head theta= %f',x,y,body_theta,head_theta)
end
            