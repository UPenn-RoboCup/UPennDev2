tStep = 1.0;
tZMP = 0.30;

phase1=0.2;
phase2=0.8;

t1= tStep*phase1;
t2 = tStep*phase2;

z0 = 0;
z1 = 0.5;
z2 = 0;


z11 = 0.4;
z12 = 0.6;


m1 = (z1-z0)/t1;
m2 = -(z1-z2)/(tStep-t2);
c1 =  tZMP*m1*sinh(-t1/tZMP);
c2 =  tZMP*m2*sinh((tStep-t2)/tZMP);
expTstep = exp(tStep/tZMP);
aP = (c2-c1/expTstep)/(expTstep-1/expTstep);
aN = (c1*expTstep-c2)/(expTstep-1/expTstep);


m1n = (z11-z0)/t1;
m2n = -(z12-z2)/(tStep-t2);
m15 = (z12-z11)/(t2-t1);


com=[];
zmp=[];
zmp2=[];
com2=[];
for ph=0:0.01:1 
    com_cur = z1+ aP*exp(ph*tStep/tZMP)+aN/exp(ph*tStep/tZMP);
    if ph<phase1
        t_start = tStep*(ph-phase1);
        zmp=[zmp z1+m1*t_start];
        com=[com com_cur+m1*t_start-tZMP*m1*sinh(t_start/tZMP)];
        
        zmp2=[zmp2 z11+m1n*t_start];
        com2=[com2 com_cur+m1*t_start-tZMP*m1*sinh(t_start/tZMP)];
    elseif ph<phase2
        t_mid = tStep*(ph-phase1);
        zmp=[zmp z1];        
        com=[com com_cur];
        
        zmp2=[zmp2 z11+m15*t_mid];
        com2=[com2 com_cur];
    else
        t_finish = tStep*(ph-phase2);
        zmp=[zmp z1+m2*t_finish];        
        com=[com com_cur+m2*t_finish-tZMP*m2*sinh(t_finish/tZMP)];
        
        zmp2=[zmp2 z12+m2n*t_finish];
        com2=[com2 com_cur+m2*t_finish-tZMP*m2*sinh(t_finish/tZMP)];
    end        
end
ph=[0:0.01:1];
subplot(2,1,1);
plot(ph,zmp,'--',ph,com,'r');

subplot(2,1,2);
plot(ph,zmp2,'--',ph,com,'r');