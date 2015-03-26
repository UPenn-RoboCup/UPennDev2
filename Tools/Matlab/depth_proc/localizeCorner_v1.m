function pose = localizeCorner_v1(Planes,metad,reset)

persistent p1
persistent p2
persistent a1
persistent b1
persistent a2
persistent b2
persistent theta_x
persistent meas_x
persistent meas_y
persistent inters
persistent orig
persistent prev_odo
persistent Fx  % filter for x location (distance)
persistent Fy  % filter for y location (distance)

vis=1;
visinit = 0;
pose = struct('x',0,'y',0,'isValidx',false,'isValidy',false,...
              'theta_body',0,'theta_head',0);

          
if isempty(p1) || (nargin == 3 && reset == 1)
    p1.init = false;     p1.sign = 0;     
    p2.init = false;     p2.sign = 0;    
    a1 = [0;0]; b1 = 0;
    a2 = [0;0]; b2 = 0;
    theta_x = 0;
    inters = [0 0]';
    prev_odo = metad.odom;
    orig = prev_odo;
    Fx = BayesianFilter1D; %Fx = Fx.initialize(0, 4);
    Fy = BayesianFilter1D; %Fy = Fy.initialize(0, 4);
    visinit = 1;
    meas_x = 0;
    meas_y = 0;
end

% no plane observed yet
if p1.init == false
    
    if ~isempty(Planes) % first observation        
        
        % the largest plane normal becomes the x 
        p1.init = true;
        % Let the normal of this plane be the x-axis
        theta_x = atan2(Planes{1}.Normal(2),Planes{1}.Normal(1));  
        a1 = Planes{1}.Normal(1:2);
        p1.sign = sign(theta_x); 
        b1 = Planes{1}.Normal'*Planes{1}.Center;
        meas_y = -b1;
        pose.y = -b1;
        Fx = Fx.initialize(pose.y, 1);
        % Compute "theta"s here 
        
        if numel(Planes) > 1 % two planes if lucky 
            p2.init = true;
            p2.sign = sign(Planes{1}.Normal(1).*Planes{2}.Normal(2) - Planes{1}.Normal(2).*Planes{2}.Normal(1)); 
            a2 = Rot2d(p2.sign*pi/2)*a1;
            b2 = Planes{2}.Normal'*Planes{2}.Center;
            pose.x = -b2;
            meas_x = -b2;
            Fy = Fy.initialize(pose.x, 1);
            
            % compute intersect point
            inters = [a1'; a2']\[b1; b2];
        end       
    end
    
else % p1 initialized 
    
    update_p1 = 0;
    update_p2 = 0;
    cr = [];
    
    % consider yaw
    u = metad.odom - prev_odo;
    ang = -theta_x - u(3);
        
    % update according to motion
    ux = [cos(ang) -sin(ang)]*u(1:2)';
    [Fx, y, ~] = Fx.propagate(ux); 
    pose.y = y;
    if p2.init == true
        uy = [sin(ang) cos(ang)]*u(1:2)';
        [Fy, x, ~] = Fy.propagate(uy);
        pose.x = x;
    end
    
    % if new measurements available, identify them first 
    if ~isempty(Planes)         
        % is the first plane p1 or not? 
        cr(1) = cos(ang)*Planes{1}.Normal(2) - sin(ang)*Planes{1}.Normal(1);
        if abs(cr(1)) > 0.9 % cross product with x-axis small enough?
           % p1
           update_p1 = 1;
        else
           % p2
           update_p2 = 1;
        end                  
        
        % if another measurement available, identify it too 
        if numel(Planes) > 1 
            % is this p1 or not? 
            cr(2) =  cos(ang)*Planes{2}.Normal(2) - sin(ang)*Planes{2}.Normal(1);
            if abs(cr(2)) > 0.9
               update_p1 = 2;
            else
               update_p2 = 2;
            end            
        end
        
    end
    
    % update the filter   
    if update_p1 > 0
        x_meas = -Planes{update_p1}.Normal'*Planes{update_p1}.Center;
        meas.value = x_meas;
        meas.param = 0;
        [Fx, x, Px] = Fx.update(meas); 
        pose.y = x;
        meas_y = x_meas;
    end
    
     if update_p2 > 0
         
        if p2.init == false % if first time the plane #2 is observed 
            p2.init = true;
            p2.sign = sign(cr(update_p2)); 
            a2 = Rot2d(p2.sign*pi/2)*a1;
            pose.x = -Planes{2}.Normal'*Planes{2}.Center;
            Fy = Fy.initialize(pose.x, 1);
             % compute intersect point            
            inters = [a1'; a2']\[b1; b2];
            meas_x = pose.x;
        else
            % update the filter 
            y_meas = -Planes{update_p2}.Normal'*Planes{update_p2}.Center;
            meas.value = y_meas;
            meas.param = 0;
            [Fy, y, Py] = Fy.update(meas);    
            pose.x = y;
            meas_x = y_meas;
        end
     end    
end

pose.isValidx = p1.init;
pose.isValidy = p2.init;

prev_odo = metad.odom;

if vis
%     switch flag
%         case 0
%             disp('No plane detected! Updating locolization with odometry.');
%         case 1
%             disp('Only seeing one plane.');
%         case 2
%             disp('Seeing two planes!');
%     end
    if visinit == 1
        figure(13),  hold off; 
        plot(0,0,'k+');axis equal; hold on;
        axis([-2 2 -0.5 2]);  
        set(gca,'XDir','reverse');
        xlabel('y_b_0');
        ylabel('x_b_0');

        if p1.init == true              
            if abs(a1(2)) > abs(a1(1))
                px = [-0.5; 2];
                py =  (b1 - a1(1)*px)/a1(2) ;
            else
                py = [-2; 2];
                px =  (b1 - a1(2)*py)/a1(1) ;
            end        
            plot(py,px,'Color',[0.7 1 0.7], 'LineWidth',4);        
        end

        if p2.init == true
            if abs(a2(2)) > abs(a2(1))
                px = [-0.5; 2];
                py =  (b2- a2(1)*px)/a2(2) ;
            else
                py = [-2; 2];
                px =  (b2 - a2(2)*py)/a2(1) ;
            end    
            plot(py,px,'Color',[0.7 0.7 1], 'LineWidth',4);
            plot(inters(2), inters(1),'ko','MarkerSize',7,'MarkerFaceColor','k');
        end
    end
    
    figure(13),
    curpos = [a2';a1']'*[pose.x;pose.y]+inters;    
    curmeas = [a2';a1']'*[meas_x;meas_y]+inters;
    plot(curpos(2), curpos(1), 'bo');    
    plot(metad.odom(2)-orig(2), metad.odom(1)-orig(1), '.','Color',0.5*ones(1,3));
    plot(curmeas(2), curmeas(1), 'r.');    
    
end
   
end
            