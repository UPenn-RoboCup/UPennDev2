h = 0.6;
g = 9.8;
p = 0.5;
t_zmp = sqrt(h/g);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Algorithm 1:fixed t_step, fixed zmp
%{
figure(1);

t_step = 0.7;
a=-p;
b = p* (cosh(t_step/t_zmp)-1)/sinh(t_step/t_zmp);
t=[0:0.01:t_step];
x = a*cosh(t/t_zmp) + b*sinh(t/t_zmp) + p;
max(x)
plot(t,x);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(2);
%}
% algorithm 2: given x0, x0', zmp
% step end time variable

clf;
hold on;


t_offset = 0;
t_start = 0;



support_foot = 0;
footY = 1.0;
torsoSupportY = 0.2;
p0 = 0.5;
x0 = p0 - torsoSupportY;

velY = 0.5;


for j=1:7
    if support_foot==0 %left support
      p1 = p0 - footY;
      
      if j==3 
          p1 = p1 + velY; %movement
      end
      x1 = p1 + torsoSupportY;      
    else %right support
      p1 = p0 + footY;
      x1 = p1 - torsoSupportY;
    end
   
    t_a = 0.2;
    for i=1:4
      t_b = t_zmp * asinh((x0-p0)/(x1-p1) * sinh(t_a/t_zmp));
      x0dot = 1/t_zmp * (x0-p0) * sinh(t_a/t_zmp);
      err = (x1-p1)*cosh(t_b/t_zmp) - (x0-p0)*cosh(t_a/t_zmp) + p1-p0;
      t_a_delta = err/x0dot/2;
      t_a = t_a + t_a_delta;
    end  
    
    t0=[t_start:0.01:t_a];
    c0 = (x0-p0)*cosh(t0/t_zmp) + p0;
    if mod(j,2)==1  plot(t0+t_offset,c0,'r')
    else     plot(t0+t_offset,c0,'b')
    end
    t_offset = t_offset + t_a - t_b;
    
    
    t_dur = t_a-t_b
    
    t_start = t_b;
    support_foot = 1-support_foot;
    p0 = p1; %support movement
    x0 = x1;
end


%{
x = (x0-p)*cosh(t/t_zmp) + v0*t_zmp*sinh(t/t_zmp) + p;
plot(t,x);
%}