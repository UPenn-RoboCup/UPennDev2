function test_lidar1(addr,id)

SetMagicPaths;
ipcInit('localhost');

lidar1Init;
servo1Init;

ipcReceiveSetFcn(GetMsgName('Lidar1'),      @slamProcessLidar1Test);
ipcReceiveSetFcn(GetMsgName('ImuFiltered'), @ipcRecvImuFcn);
ipcReceiveSetFcn(GetMsgName('Servo1'),      @slamProcessServo1);

loop = 1;
while loop,
  ipcReceiveMessages;
end
