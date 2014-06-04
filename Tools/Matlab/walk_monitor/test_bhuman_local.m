h = 0.6;
g = 9.8;
p = 0.5;
t_zmp = sqrt(h/g);



% algorithm 2: given x0, x0', zmp
% step end time variable

close all;
hold on;


t_offset = 0;
t_start = 0;



support_foot = 0;
supportY = 0.5;
torsoY = 0.3;


vy = 0;
y = torsoY-supportY;
y0 = torsoY-supportY;
vy0 = 0;


x_start=0;
vx_start=0;

t_start = 0;

%For transforming to global coordinate

uSupportY = supportY;
uSupportX = 0;

for j=1:8
    
    %Next support position
    if support_foot==0 %left support
      supportMovementY = -2*supportY;     
    else        
      supportMovementY = 2*supportY;
    end
    y1=-y0;
    
    if j==5 
        supportMovementY =   supportMovementY + 0.4;
    end 
    
    
    %Calculate the switch time using newton solver
    t_end = 0.37;
    
    for i=1:4
      t_start_next = t_zmp * asinh(y0/y1 * sinh(t_end/t_zmp));           
      est =  y0*cosh(t_end/t_zmp)-y1*cosh(t_start_next/t_zmp);         
      y0dot = 1/t_zmp * y0 * sinh(t_end/t_zmp); 
      err = supportMovementY-est;
      t_end_delta = err/y0dot/2;
      t_end = t_end + t_end_delta;
    end      
    t_start_next = t_zmp * asinh(y0/y1 * sinh(t_end/t_zmp));
    
    %Current pendumum: t_start to t_end
    %Next pendulum: t_start_next 
    
    % X solving
    % Given: x and v at the start  
    % Assumption: nx = 0
    % ZMP compensation for next step: 0.015 (b-human value)    
    
    %x(t) = (x0-px0)* cosh(t/tzmp)+ vx0*tzmp*sinh(t/tzmp) + px0
    %vx(t) = (x0-px0)/tzmp * sinh(t/tzmp)

    
    
    
    
    r_next = 0.0;
    step_x = 0.03;
    
    if j==3 step_x = 0.12;end
    
    xparams=linsolve(...
        [1 cosh(t_start/t_zmp)      t_zmp*sinh(t_start/t_zmp) 0;
         0 sinh(t_start/t_zmp)/t_zmp  cosh(t_start/t_zmp)  0;
         0 sinh(t_end/t_zmp)/t_zmp    cosh(t_start/t_zmp) -cosh(t_start_next/t_zmp);
         1 cosh(t_end/t_zmp)          t_zmp*sinh(t_end/t_zmp) -t_zmp*sinh(t_start_next/t_zmp)],...
         [x_start;vx_start;0;r_next+step_x]);
     
    %xparams: r x0 v0 v0_next
    
    r=xparams(1);
    x0=xparams(2);
    vx0=xparams(3);
    
    
    
    
    t0=[t_start:0.01:t_end];
    
    
    cx0 = uSupportX + xparams(1) + xparams(2)*cosh(t0/t_zmp) + xparams(3) *t_zmp*sinh(t0/t_zmp);    
    cy0 = uSupportY + y0*cosh(t0/t_zmp);    
    
    subplot(2,1,1);
    hold on;
    if mod(j,2)==1  plot(t0+t_offset,cy0,'r')
    else     plot(t0+t_offset,cy0,'b')
    end

    subplot(2,1,2);
    hold on;
    if mod(j,2)==1  plot(t0+t_offset,cx0,'r')
    else     plot(t0+t_offset,cx0,'b')
    end
    
    
    t_offset = t_offset + t_end - t_start_next;
    uSupportY = uSupportY + supportMovementY;        
    
    t_start = t_start_next;
    support_foot = 1-support_foot;
    y0=y1;
    

    xfin = xparams(1) + xparams(2)*cosh(t_end/t_zmp)+xparams(3)*t_zmp*sinh(t_end/t_zmp);
    x_start = xfin + xparams(1) - r_next - step_x;
    vx_start = xparams(2)/t_zmp*sinh(t_end/t_zmp) + xparams(3)*cosh(t_end/t_zmp);
    uSupportX = uSupportX + step_x + r_next-xparams(1);    
    
end


%{
x = (x0-p)*cosh(t/t_zmp) + v0*t_zmp*sinh(t/t_zmp) + p;
plot(t,x);
%}