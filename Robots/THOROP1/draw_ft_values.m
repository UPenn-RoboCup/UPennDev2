close all;
t_max=max(debug(:,1));
subplot(2,2,1);
hold on;
plot(debug(:,1),debug(:,2),'r-');
plot(debug(:,1),debug(:,3),'b-');
plot(debug(:,1),debug(:,4),'k-');
legend('X Force','Y Force','Z Force');
axis([0 t_max -300 300]);
subplot(2,2,3);
hold on;
plot(debug(:,1),debug(:,5),'r-');
plot(debug(:,1),debug(:,6),'b-');
plot(debug(:,1),debug(:,7),'k-');
legend('X Torque','Y Torque','Z Torque');
axis([0 t_max -50 50]);


subplot(2,2,2);
hold on;
plot(debug(:,1),debug(:,8),'r-');
plot(debug(:,1),debug(:,9),'b-');
plot(debug(:,1),debug(:,10),'k-');
legend('X Force','Y Force','Z Force');
axis([0 t_max -300 300]);

subplot(2,2,4);
hold on;
plot(debug(:,1),debug(:,11),'r-');
plot(debug(:,1),debug(:,12),'b-');
plot(debug(:,1),debug(:,13),'k-');
legend('X Torque','Y Torque','Z Torque');
axis([0 t_max -50 50]);

