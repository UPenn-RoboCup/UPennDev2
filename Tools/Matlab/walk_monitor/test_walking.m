function test_walking
    clf;
    h = 0.6; %body height
    g = 9.8;
    t_zmp = sqrt(h/g);

    supportY = 0.5;  %Y ZMP position
    torsoY = 0.3;    %the max COM movement allowed 
       
    %For transforming to global coordinate

    step_x = 0.30;
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
        rx=xparams(1);
        x0=xparams(2);
        vx0=xparams(3);
        
        stepparams.rx = rx;
        stepparams.x0 = x0;
        stepparams.vx0 = vx0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [cx,cy] = calculate_com_position(stepparams,t_local)        

        %TODO: correct transform (for rotation)
        cx = stepparams.uSupport(1) + stepparams.rx + stepparams.x0*cosh(t_local/t_zmp) +stepparams.vx0 *t_zmp*sinh(t_local/t_zmp);  
        cy = stepparams.uSupport(2) + stepparams.y0*cosh((t_local)/t_zmp);        
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
      else stepparams.support_foot = 1-stepparams.support_foot;
      end

      if step_count>5 && stepparams.support_foot==0 stepparams.support_foot = 4;   end


      stepparams.y0=stepparams.y0_next;       

      xfin = stepparams.rx + stepparams.x0*cosh(stepparams.t_end/t_zmp)+stepparams.vx0*t_zmp*sinh(stepparams.t_end/t_zmp);
      stepparams.x_start = xfin + stepparams.rx - step_x;
      stepparams.vx_start = stepparams.x0/t_zmp*sinh(stepparams.t_end/t_zmp) + stepparams.vx0*cosh(stepparams.t_end/t_zmp);

      %TODO: transform (for rotation)
      stepparams.uSupport(1)=stepparams.uSupport(1)+ step_x -stepparams.rx;
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
        else            
          stepparams.supportMovementX = 0;            
        end
    end    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function draw_com(stepparams,t_local)
        [cx,cy] = calculate_com_position(stepparams,t_local);        
        marker='r.';        
        if stepparams.support_foot==1 marker= 'b.';
        elseif stepparams.support_foot==2 marker= 'g.';                  
        elseif stepparams.support_foot==4 marker= 'y.';                  
        elseif stepparams.support_foot==6 marker= 'k.';                  
        end                    
        subplot(2,1,1);    
        hold on;
        plot(t,cy,marker) 
        subplot(2,1,2);
        hold on;
        plot(t,cx,marker);        
    end
    
    t=0;
    running = 1;
    t_local = 0;
    step_params = set_support_target(step_params);
    step_params = calculate_switch_time(step_params);        
    step_params = calculate_x_parameters(step_params);        
        
    
    while running>0 && t<20
        t=t+0.01;
        t_local = t-step_params.t_offset;

        draw_com(step_params,t_local);
        if step_params.support_foot==6 && t_local>0 running = 0;  end        
        
        if t_local>step_params.t_end-0.01 
          step_params = advance_step(step_params);        
          step_params = set_support_target(step_params);    
          step_params = calculate_switch_time(step_params);        
          step_params = calculate_x_parameters(step_params);     
        end
                                    
    end

end