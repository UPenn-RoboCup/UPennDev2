h = 0.6;
g = 9.8;
p = 0.5;
t_zmp = sqrt(h/g);



% algorithm 2: given x0, x0', zmp
% step end time variable

clf;
hold on;


t_offset = 0;
t_start = 0;



support_foot = 0;
footY = 1.0;
torsoSupportY = 0.2;
py0 = 0.5;
y0 = py0 - torsoSupportY;

velY = 0.5;


for j=1:7
    if support_foot==0 %left support
      py1 = py0 - footY;
      
      if j==3 
          py1 = py1 + velY; %movement
      end
      y1 = py1 + torsoSupportY;      
    else %right support
      py1 = py0 + footY;
      y1 = py1 - torsoSupportY;
    end
   
    t_a = 0.2;
    for i=1:4
      t_b = t_zmp * asinh((y0-py0)/(y1-py1) * sinh(t_a/t_zmp));
      y0dot = 1/t_zmp * (y0-py0) * sinh(t_a/t_zmp);
      err = (y1-py1)*cosh(t_b/t_zmp) - (y0-py0)*cosh(t_a/t_zmp) + py1-py0;
      t_a_delta = err/y0dot/2;
      t_a = t_a + t_a_delta;
    end  
    
    %Current pendumum: t_start to t_a
    %Next pendulum: starts at t_b 
    
        
    
    
    
    
    t0=[t_start:0.01:t_a];
    cy0 = (y0-py0)*cosh(t0/t_zmp) + py0;
    if mod(j,2)==1  plot(t0+t_offset,cy0,'r')
    else     plot(t0+t_offset,cy0,'b')
    end
    t_offset = t_offset + t_a - t_b;
    
    
    t_dur = t_a-t_b
    
    t_start = t_b;
    support_foot = 1-support_foot;
    py0 = py1; %support movement
    y0 = y1;
end


%{
x = (x0-p)*cosh(t/t_zmp) + v0*t_zmp*sinh(t/t_zmp) + p;
plot(t,x);
%}