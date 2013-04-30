global  LIDAR0_TS


figure(1); clf(gcf);

cntr = LIDAR0_TS.cntr-1;

plot(LIDAR0_TS.dts(1:cntr))