global  IMU_TS


figure(1); clf(gcf);

cntr = IMU_TS.cntr-1;

plot(IMU_TS.dts(1:cntr))