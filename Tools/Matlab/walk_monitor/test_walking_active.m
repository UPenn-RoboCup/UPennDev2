function test_walking(enable_push)
    clf;
    h = 0.6; %body height
    g = 9.8;
    t_zmp = sqrt(h/g);

    supportY = 0.5;  %Y ZMP position
    torsoY = 0.3;    %the max COM movement allowed 
       
    %For transforming to global coordinate

    step_x = 0.50;
    step_y = 0.0;
    
    y0 = torsoY-supportY;
 
    step_params=[];
    step_params.stepX=0;
    step_params.stepY=0; 
    step_params.stepA=0;
        
    step_params.t_start = 0;    
    step_params.y0=y0;
    step_params.y0_next=0;
   
    step_params.x_start = 0;
    step_params.vx_start = 0;
    step_params.rx = 0;
    step_params.rx_next = 0;
      
    step_params.support_foot = 0;
    
    step_params.t_offset = 0;
    step_params.uSupport=[0 supportY 0];

    %Initial DS state
    
    step_params.y0=-0.04;    
    step_params.uSupport=[0 0 0];
    step_params.support_foot = 2;
    step_count = 0;
    
    stop_request = 0;
    walk_ph = 0;
    
    function global_pose = pose_global(rel,pose)
        global_pose=[pose(1)+cos(pose(3))*rel(1)-sin(pose(3))*rel(2),...
                     pose(2)+sin(pose(3))*rel(1)+cos(pose(3))*rel(2),...
                     pose(3)+rel(3)];        
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    %Calculate the step switch time using newton solver
    function stepparams = calculate_switch_time(stepparams)
        t_end = 0.37;    %initial guess
        for i=1:3
          t_start_next = t_zmp * asinh(stepparams.y0/stepparams.y0_next * sinh(t_end/t_zmp));           
          est =  stepparams.y0*cosh(t_end/t_zmp)-stepparams.y0_next*cosh(t_start_next/t_zmp);         
          y0dot = 1/t_zmp * stepparams.y0 * sinh(t_end/t_zmp); 
          err = stepparams.supportMovementY-est;
          t_end_delta = err/y0dot/2;
          t_end = t_end + t_end_delta;
        end      
        t_start_next = t_zmp * asinh(stepparams.y0/stepparams.y0_next * sinh(t_end/t_zmp));        
        stepparams.t_end = t_end;
        stepparams.t_start_next = t_start_next;        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
      
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Solve X axis parameters
    function stepparams = calculate_x_parameters(stepparams)
        
        rx_next = 0; %GIVEN PARAM
        
        t_start = stepparams.t_start;
        t_end = stepparams.t_end;
        t_start_next = stepparams.t_start_next;
        x_start = stepparams.x_start;
        vx_start =stepparams.vx_start;
        
        xparams=linsolve(...
                [1 cosh(t_start/t_zmp)      t_zmp*sinh(t_start/t_zmp) 0;
                 0 sinh(t_start/t_zmp)/t_zmp  cosh(t_start/t_zmp)  0;
                 0 sinh(t_end/t_zmp)/t_zmp    cosh(t_start/t_zmp) -cosh(t_start_next/t_zmp);
                 1 cosh(t_end/t_zmp)          t_zmp*sinh(t_end/t_zmp) -t_zmp*sinh(t_start_next/t_zmp)],...
                 [x_start;vx_start;0;rx_next+step_params.supportMovementX]);
        
        stepparams.rx = xparams(1);
        stepparams.x0 = xparams(2);
        stepparams.vx0 = xparams(3);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Solve X axis parameters after phase shift
    function stepparams = recalculate_x_parameters(stepparams,t_local)
        
        rx_next = 0; %GIVEN PARAM
                
        t_end = stepparams.t_end;
        t_start_next = stepparams.t_start_next;
        t_local_old = stepparams.t_local_old;
        
        x_current = stepparams.rx + stepparams.x0*cosh(t_local_old/t_zmp) +stepparams.vx0 *t_zmp*sinh(t_local_old/t_zmp);  
        vx_current = stepparams.x0/t_zmp*sinh(t_local_old/t_zmp)+stepparams.vx0*cosh(t_local_old/t_zmp);
        
        rx_old = stepparams.rx;
                       
        disp('recalculating param:');
        xparams=linsolve(...
                [1 cosh(t_local/t_zmp)      t_zmp*sinh(t_local/t_zmp) 0;
                 0 sinh(t_local/t_zmp)/t_zmp  cosh(t_local/t_zmp)  0;
                 0 sinh(t_end/t_zmp)/t_zmp    cosh(t_local/t_zmp) -cosh(t_start_next/t_zmp);
                 1 cosh(t_end/t_zmp)          t_zmp*sinh(t_end/t_zmp) -t_zmp*sinh(t_start_next/t_zmp)],...
                 [x_current;vx_current;0;rx_next+step_params.supportMovementX - (x_current-rx_old)]);
             
        stepparams.rx = xparams(1);
        stepparams.x0 = xparams(2);
        stepparams.vx0 = xparams(3);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [cx,cy,zmpx,zmpy] = calculate_com_position(stepparams,t_local)        

        %TODO: correct transform (for rotation)
        cx = stepparams.uSupport(1) + stepparams.rx + stepparams.x0*cosh(t_local/t_zmp) +stepparams.vx0 *t_zmp*sinh(t_local/t_zmp);  
        cy = stepparams.uSupport(2) + stepparams.y0*cosh((t_local)/t_zmp);        
        
        zmpx = cx - stepparams.x0*cosh(t_local/t_zmp) - stepparams.vx0 *t_zmp*sinh(t_local/t_zmp);
        zmpy = cy - stepparams.y0*cosh(t_local/t_zmp);
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Advance the step state
    function stepparams = advance_step(stepparams)
      disp('step');
      stepparams.t_offset = stepparams.t_offset + stepparams.t_end - stepparams.t_start_next;
      stepparams.t_start = stepparams.t_start_next;

      if stepparams.support_foot==2  stepparams.support_foot = 1;   %ds to right stance, starting state
      elseif stepparams.support_foot==4  stepparams.support_foot = 6; %double support, ending state          
      elseif stepparams.support_foot==5  stepparams.support_foot = 6; %double support, ending state
      else stepparams.support_foot = 1-stepparams.support_foot;
      end
      
      if stop_request>0
          if stepparams.support_foot==0 stepparams.support_foot=4;
          elseif stepparams.support_foot==1 stepparams.support_foot=5;
          end
      end
      
      stepparams.y0=stepparams.y0_next;       

      xfin = stepparams.rx + stepparams.x0*cosh(stepparams.t_end/t_zmp)+stepparams.vx0*t_zmp*sinh(stepparams.t_end/t_zmp);
      stepparams.x_start = xfin - step_x;      
      stepparams.vx_start = stepparams.x0/t_zmp*sinh(stepparams.t_end/t_zmp) + stepparams.vx0*cosh(stepparams.t_end/t_zmp);
      walk_ph = 0;
      
      
      %TODO: transform (for rotation)
      stepparams.uSupport(1)=stepparams.uSupport(1)+ step_x;      
      stepparams.uSupport(2)=stepparams.uSupport(2)+stepparams.supportMovementY;
      
      step_count = step_count + 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function stepparams=set_support_target(stepparams)
        %Setup next support position and target Y COM position
        if stepparams.support_foot==0 %left support
          stepparams.y0_next = - (torsoY - supportY);                      
          stepparams.supportMovementX = step_x;
          stepparams.supportMovementY = step_y-2*supportY;    
        elseif stepparams.support_foot==1 %right support                  
          stepparams.y0_next =  torsoY - supportY;          
          stepparams.supportMovementX = step_x;          
          stepparams.supportMovementY = step_y+2*supportY;    
        elseif stepparams.support_foot==2 %double support to right support          
          stepparams.y0_next = - (torsoY - supportY);                      
          stepparams.supportMovementX = 0;
          stepparams.supportMovementY = -supportY;
        elseif step_params.support_foot==4 %left support to double support
          stepparams.y0_next =  0.03;          
          stepparams.supportMovementX = 0;          
          stepparams.supportMovementY = -supportY;                        
          
        elseif step_params.support_foot==5 %right support to double support
          stepparams.y0_next =  -0.03;          
          stepparams.supportMovementX = 0;          
          stepparams.supportMovementY = supportY;                                          
          
        else            
          stepparams.supportMovementX = 0;            
        end
        if step_count==1
          stepparams.supportMovementX = stepparams.supportMovementX/2;
        end
                    
            
    end    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function draw_com(stepparams,t_local,walk_ph)
        [cx,cy,zmpx,zmpy] = calculate_com_position(stepparams,t_local);        
        marker='r.';        
        if stepparams.support_foot==1 marker= 'b.';
        elseif stepparams.support_foot==2 marker= 'g.';                  
        elseif stepparams.support_foot==4 marker= 'y.'; 
        elseif stepparams.support_foot==5 marker= 'y.'; 
        elseif stepparams.support_foot==6 marker= 'k.';                  
        end                    
        subplot(3,1,1);    
        hold on;
        plot(t,cy,marker,t,zmpy,'k') 
        subplot(3,1,2);
        hold on;
        plot(t,cx,marker,t,zmpx,'k');        
        subplot(3,1,3);
        hold on;
        plot(t,walk_ph);        
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function stepparams=estimate_current_state(stepparams,t_local,t)
       y_t = stepparams.y0*cosh(t_local/t_zmp);
       vy_t = stepparams.y0/t_zmp*sinh(t_local/t_zmp);
       
       if enable_push==1
           if abs(t-2.5)<0.005 vy_t = vy_t-1; end
           if abs(t-2.6)<0.005 vy_t = vy_t-0.5; end
       elseif enable_push==2
           if abs(t-2.2)<0.005 vy_t = vy_t+1; end           
       end
       
       y0_measured = sqrt(y_t*y_t - (vy_t*t_zmp)^2)*sign(y_t);
       t_local_measured = asinh(vy_t*t_zmp/y0_measured)*t_zmp;        
       
       if abs(stepparams.y0-y0_measured)>0.0001 || abs(t_local-t_local_measured>0.001)
          y0_dff=[stepparams.y0 y0_measured]
          t_diff=[t_local t_local_measured]          
          disp('shock');          
          stepparams.pushed=1;
          stepparams.y0_measured = y0_measured;
          stepparams.t_local_measured = t_local_measured;
       else
          stepparams.pushed = 0 ;
       end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    t=0;
    running = 1;
    t_local = 0;
    step_params = set_support_target(step_params);
    step_params = calculate_switch_time(step_params);        
    step_params = calculate_x_parameters(step_params);        
        
    
    while running>0 %&& step_count<5
        t=t+0.01;
        t_local = t-step_params.t_offset;
        
        walk_ph_old = walk_ph;
        walk_ph =(t_local-step_params.t_start) / (step_params.t_end-step_params.t_start);
        walk_ph = max(walk_ph_old,walk_ph);
        
        if step_count>5 stop_request=1;end
          
          
  
        draw_com(step_params,t_local,walk_ph);
        if step_params.support_foot==6 && t_local>0 running = 0;  end        

        step_params = estimate_current_state(step_params,t_local,t);
        if step_params.pushed            
            step_params.t_local_old = t_local;            

            step_params.y0 = step_params.y0_measured;
            step_params.t_local_old = t_local;
            t_local = step_params.t_local_measured;

            step_params.t_offset = t-t_local;
            step_params = calculate_switch_time(step_params);        

            step_params = recalculate_x_parameters(step_params,t_local); 
        end        

        if t_local>step_params.t_end-0.01 
          step_params = advance_step(step_params);        
          step_params = set_support_target(step_params);    
          step_params = calculate_switch_time(step_params);        
          step_params = calculate_x_parameters(step_params);     
        end
                                    
    end

    curaxis=axis;
    curaxis([3 4])=[-1 1];
    subplot(3,1,1);
    axis(curaxis);
    
end