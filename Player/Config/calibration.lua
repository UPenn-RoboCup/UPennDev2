module(..., package.seeall);

---------------------------------------------
-- Automatically generated calibration data
---------------------------------------------
cal={}

--Initial values for each robots

cal["betty"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchComp = 0,
  armBias={0,0,0,0,0,0},
  pid = 0,
};

cal["linus"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchComp = 0,
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};

cal["lucy"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchComp = 0,
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};

cal["scarface"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchComp = 0,
  armBias={0,0,0,0,0,0},
  pid = 1,
};

cal["felix"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchBiasComp = 4*math.pi/180;
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};

cal["hokie"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchBiasComp = 0*math.pi/180;
  armBias={0,12*math.pi/180,0,0,-6*math.pi/180,0},
  pid = 1, --NEW FIRMWARE
};

cal["darwin1"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchBiasComp = 0*math.pi/180;
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};

cal["darwin2"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchBiasComp = 0*math.pi/180;
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};

cal["darwin3"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchBiasComp = 0*math.pi/180;
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};

cal["darwin4"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchBiasComp = 0*math.pi/180;
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};

cal["darwin5"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchBiasComp = 0*math.pi/180;
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};

cal["darwin6"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchBiasComp = 0*math.pi/180;
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};

cal["darwin7"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchBiasComp = 4*math.pi/180;
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};

cal["darwin8"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchBiasComp = 0*math.pi/180;
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};

cal["darwin9"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchBiasComp = 0*math.pi/180;
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};

cal["darwin10"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchBiasComp = 0*math.pi/180;
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};









------------------------------------------------------------
--Auto-appended calibration settings
------------------------------------------------------------

-- Updated date: Mon Apr  9 07:28:15 2012
cal["betty"].servoBias={0,0,2,-6,-1,0,0,0,-3,-1,-3,0,};

-- Updated date: Sun Apr 15 20:46:52 2012
cal["linus"].servoBias={3,1,2,1,1,-3,-8,3,-13,-4,1,-5,};
cal["linus"].footXComp=-0.003;
cal["linus"].kickXComp=0.000;

-- Updated date: Tue Apr 17 01:23:40 2012
cal["lucy"].servoBias={1,10,-18,20,24,7,-43,-4,10,0,6,0,};
cal["lucy"].footXComp=-0.003;
cal["lucy"].kickXComp=0.005;

-- Updated date: Mon Apr 16 18:28:20 2012
cal["betty"].servoBias={0,0,2,-6,-1,0,0,3,-3,-1,-3,-2,};
cal["betty"].footXComp=0.006;
cal["betty"].kickXComp=0.005;

-- Updated date: Mon Apr 16 23:36:32 2012
cal["scarface"].servoBias={0,0,7,0,0,0,0,0,-7,-9,-4,0,};
cal["scarface"].footXComp=0.002;
cal["scarface"].kickXComp=0.000;

-- Updated date: Wed Apr 18 12:26:58 2012
cal["scarface"].servoBias={0,0,7,0,0,0,0,0,-7,-9,-4,0,};
cal["scarface"].footXComp=0.002;
cal["scarface"].kickXComp=0.005;

-- Updated date: Wed Apr 18 23:55:59 2012
cal["lucy"].servoBias={19,3,4,20,19,10,-15,11,-20,0,6,6,};
cal["lucy"].footXComp=0.006;
cal["lucy"].kickXComp=0.005;

-- Updated date: Thu Apr 19 21:39:44 2012
cal["lucy"].servoBias={19,3,4,20,19,10,-15,11,-20,0,6,6,};
cal["lucy"].footXComp=0.006;
cal["lucy"].kickXComp=0.005;

-- Updated date: Fri Apr 20 00:00:29 2012
cal["linus"].servoBias={3,1,2,1,1,-3,-8,3,-13,-4,1,-5,};
cal["linus"].footXComp=-0.003;
cal["linus"].kickXComp=0.000;

-- Updated date: Fri Apr 20 21:12:24 2012
cal["lucy"].servoBias={19,3,10,20,19,22,-15,11,-20,0,6,6,};
cal["lucy"].footXComp=0.006;
cal["lucy"].kickXComp=0.005;

-- Updated date: Fri Apr 20 21:14:31 2012
cal["lucy"].servoBias={19,-22,10,20,19,-1,-15,11,-20,0,6,6,};
cal["lucy"].footXComp=0.006;
cal["lucy"].kickXComp=0.005;

-- Updated date: Fri Apr 20 21:16:10 2012
cal["lucy"].servoBias={19,-22,10,20,19,-1,-15,11,-20,0,6,6,};
cal["lucy"].footXComp=0.006;
cal["lucy"].kickXComp=0.005;

--AFTER LEG SWAP WITH LUCY
-- Updated date: Mon Apr 23 22:03:46 2012
cal["linus"].servoBias={3,1,43,1,-12,-17,-8,-5,-13,-4,1,-5,};
cal["linus"].footXComp=0.012;
cal["linus"].kickXComp=0.005;

-- Updated date: Sat May  5 09:56:25 2012
cal["linus"].servoBias={3,1,43,1,-14,-31,-8,-5,-13,-4,1,0,};
cal["linus"].footXComp=0.006;
cal["linus"].kickXComp=0.005;

-- Updated date: Sat May  5 21:53:24 2012
cal["scarface"].servoBias={0,0,7,0,0,0,0,0,-7,-9,-4,0,};
cal["scarface"].footXComp=-0.003;
cal["scarface"].kickXComp=0.010;

-- Updated date: Sat May  5 22:07:10 2012
cal["scarface"].servoBias={0,0,7,0,0,0,0,0,-7,-9,-4,2,};
cal["scarface"].footXComp=-0.003;
cal["scarface"].kickXComp=0.010;

-- Updated date: Sat May  5 22:28:03 2012
cal["felix"].servoBias={11,-11,0,0,0,0,-9,4,0,0,-6,12,};
cal["felix"].footXComp=-0.002;
cal["felix"].kickXComp=0.000;

-- Updated date: Sat May  5 22:35:53 2012
cal["felix"].servoBias={11,-11,0,0,0,0,-9,4,0,0,-6,12,};
cal["felix"].footXComp=0.001;
cal["felix"].kickXComp=0.005;

-- Updated date: Sat May  5 22:50:50 2012
cal["felix"].servoBias={11,1,0,0,0,0,-9,10,0,0,-6,15,};
cal["felix"].footXComp=0.001;
cal["felix"].kickXComp=0.005;


-- Updated date: Sun May  6 12:03:01 2012
cal["hokie"].servoBias={0,0,0,0,0,0,0,0,0,0,0,0,};
cal["hokie"].footXComp=0.000;
cal["hokie"].kickXComp=0.000;

-- Updated date: Sun May  6 14:20:12 2012
cal["hokie"].servoBias={0,0,0,25,0,-26,0,0,0,0,0,2,};
cal["hokie"].footXComp=0.000;
cal["hokie"].kickXComp=0.000;

-- Updated date: Sun May  6 14:23:18 2012
cal["hokie"].servoBias={0,-14,0,25,0,-26,0,8,0,0,0,2,};
cal["hokie"].footXComp=0.010;
cal["hokie"].kickXComp=0.000;

-- Updated date: Sun May  6 14:26:23 2012
cal["hokie"].servoBias={0,-14,0,25,0,-26,0,10,0,0,0,2,};
cal["hokie"].footXComp=0.010;
cal["hokie"].kickXComp=0.000;

-- Updated date: Sun May  6 14:53:11 2012
cal["hokie"].servoBias={0,-13,-64,25,0,-26,0,-11,78,0,0,2,};
cal["hokie"].footXComp=0.006;
cal["hokie"].kickXComp=0.000;

-- Updated date: Sun May  6 16:55:53 2012
cal["hokie"].servoBias={0,-13,-48,26,0,-26,0,-3,66,-33,0,2,};
cal["hokie"].footXComp=0.006;
cal["hokie"].kickXComp=0.000;

-- Updated date: Sun May  6 17:42:47 2012
cal["hokie"].servoBias={0,-13,-48,26,23,-21,0,-3,66,-59,-47,-3,};
cal["hokie"].footXComp=0.001;
cal["hokie"].kickXComp=0.000;

-- Updated date: Sun May  6 18:35:12 2012
cal["hokie"].servoBias={0,-13,-48,26,23,-21,0,-3,66,-59,-47,-3,};
cal["hokie"].footXComp=0.004;
cal["hokie"].kickXComp=0.000;

-- Updated date: Thu May 10 01:10:47 2012
cal["darwin7"].servoBias={0,0,0,0,0,0,0,0,0,0,0,0,};
cal["darwin7"].footXComp=-0.010;
cal["darwin7"].kickXComp=0.010;

-- Updated date: Thu May 10 01:44:57 2012
cal["darwin7"].servoBias={0,0,0,0,0,0,0,0,0,0,21,0,};
cal["darwin7"].footXComp=-0.006;
cal["darwin7"].kickXComp=0.010;

-- Updated date: Thu May 10 01:47:24 2012
cal["darwin7"].servoBias={0,0,0,0,0,0,0,0,0,0,21,0,};
cal["darwin7"].footXComp=0.000;
cal["darwin7"].kickXComp=0.010;

-- Updated date: Thu May 10 02:27:33 2012
cal["darwin7"].servoBias={23,0,0,0,-1,0,0,0,0,-17,-15,0,};
cal["darwin7"].footXComp=-0.005;
cal["darwin7"].kickXComp=0.010;

-- Updated date: Thu May 10 03:34:25 2012
cal["darwin2"].servoBias={0,0,0,0,0,0,0,0,0,0,0,0,};
cal["darwin2"].footXComp=-0.004;
cal["darwin2"].kickXComp=0.000;

-- Updated date: Thu May 10 05:00:49 2012
cal["darwin7"].servoBias={23,0,0,0,-1,0,0,0,0,-17,-15,0,};
cal["darwin7"].footXComp=-0.011;
cal["darwin7"].kickXComp=0.010;

-- Updated date: Thu May 10 05:05:50 2012
cal["darwin7"].servoBias={23,0,0,0,-1,0,0,0,0,-17,-15,0,};
cal["darwin7"].footXComp=-0.004;
cal["darwin7"].kickXComp=0.010;

-- Updated date: Tue May 15 13:33:24 2012
cal["scarface"].servoBias={0,0,7,373,0,0,0,0,-7,-9,-4,2,};
cal["scarface"].footXComp=-0.003;
cal["scarface"].kickXComp=0.010;

-- Updated date: Tue May 15 18:47:35 2012
cal["scarface"].servoBias={0,0,6,50,55,0,0,0,-11,-32,-4,2,};
cal["scarface"].footXComp=-0.003;
cal["scarface"].kickXComp=0.010;

-- Updated date: Tue May 15 18:48:48 2012
cal["scarface"].servoBias={0,0,6,50,55,0,0,0,-11,-32,-4,2,};
cal["scarface"].footXComp=-0.001;
cal["scarface"].kickXComp=0.010;

-- Updated date: Tue May 15 18:49:17 2012
cal["scarface"].servoBias={0,0,6,50,55,0,0,0,-11,-32,-4,2,};
cal["scarface"].footXComp=-0.001;
cal["scarface"].kickXComp=0.010;

-- Updated date: Tue May 22 20:34:10 2012
cal["scarface"].servoBias={0,0,6,50,32,18,0,0,-20,-32,-4,1,};
cal["scarface"].footXComp=-0.001;
cal["scarface"].kickXComp=0.010;

-- Updated date: Tue May 22 20:35:53 2012
cal["scarface"].servoBias={0,0,6,50,32,18,0,0,-20,-32,-4,1,};
cal["scarface"].footXComp=-0.001;
cal["scarface"].kickXComp=0.010;

-- Updated date: Thu May 24 12:39:17 2012
cal["scarface"].servoBias={0,0,6,50,59,15,0,0,-20,-32,-3,1,};
cal["scarface"].footXComp=-0.001;
cal["scarface"].kickXComp=0.010;

-- Updated date: Fri May 28 06:33:46 2010
cal["betty"].servoBias={8,0,2,-6,-1,0,-5,6,-3,-1,-4,3,};
cal["betty"].footXComp=0.006;
cal["betty"].kickXComp=0.005;

-- Updated date: Thu May 31 20:30:23 2012
cal["betty"].servoBias={8,0,2,-6,-1,-6,-5,6,-3,-1,-4,7,};
cal["betty"].footXComp=0.006;
cal["betty"].kickXComp=0.005;

-- Updated date: Thu May 31 20:39:39 2012
cal["betty"].servoBias={8,0,2,-6,-1,0,-5,6,-3,-1,-4,7,};
cal["betty"].footXComp=0.006;
cal["betty"].kickXComp=0.005;

-- Updated date: Fri Jun  1 00:55:57 2012
cal["linus"].servoBias={21,1,43,1,-6,-22,-21,-5,-13,-4,-10,17,};
cal["linus"].footXComp=0.002;
cal["linus"].kickXComp=0.005;

-- Updated date: Fri Jun  1 02:39:22 2012
cal["linus"].servoBias={21,-3,43,-1,1,-11,-21,-5,-13,-12,-15,17,};
cal["linus"].footXComp=0.001;
cal["linus"].kickXComp=0.005;

-- Updated date: Fri Jun  1 02:42:37 2012
cal["linus"].servoBias={21,-3,43,-1,1,-11,-21,-5,-13,-12,-15,-3,};
cal["linus"].footXComp=0.001;
cal["linus"].kickXComp=0.005;

-- Updated date: Sat Jun  2 17:03:33 2012
cal["scarface"].servoBias={0,0,6,50,59,15,0,0,-20,-32,-3,1,};
cal["scarface"].footXComp=-0.001;
cal["scarface"].kickXComp=0.005;

-- Updated date: Sat Jun  2 17:12:10 2012
cal["scarface"].servoBias={0,0,6,50,59,15,0,0,-20,-32,-21,1,};
cal["scarface"].footXComp=-0.001;
cal["scarface"].kickXComp=0.005;

-- Updated date: Sat Jun  2 17:28:43 2012
cal["scarface"].servoBias={0,0,6,50,59,15,0,0,-20,-32,-21,1,};
cal["scarface"].footXComp=-0.004;
cal["scarface"].kickXComp=0.005;

-- Updated date: Sat Jun  2 17:35:13 2012
cal["scarface"].servoBias={0,0,6,50,59,15,0,0,-20,-32,-21,1,};
cal["scarface"].footXComp=-0.008;
cal["scarface"].kickXComp=0.005;

-- Updated date: Sat Jun  2 21:40:18 2012
cal["linus"].servoBias={21,-3,43,-1,1,-11,-21,-5,-13,-12,-15,-3,};
cal["linus"].footXComp=0.002;
cal["linus"].kickXComp=0.010;
