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
  pid = 1,
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
  armBias=vector.new({0,-10,10,0,0,26})*math.pi/180, 
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
  armBias={0,10*math.pi/180,0,0,-6*math.pi/180,0},
  pid = 1, --NEW FIRMWARE
};

cal["jiminy"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchComp = 4*math.pi/180;
  armBias={0,6*math.pi/180,0,
           0,-4*math.pi/180,0},
  pid = 1, --NEW FIRMWARE
};

cal["sally"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchComp = 0;
  armBias={0,0,0,0,0,0},
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


-- Updated date: Mon Jun 18 12:28:45 2012
cal["scarface"].servoBias={0,0,6,50,30,15,0,0,-20,-32,-21,1,};
cal["scarface"].footXComp=0.000;
cal["scarface"].kickXComp=0.005;

-- Updated date: Mon Jun 18 15:05:26 2012
cal["linus"].servoBias={21,-3,37,-10,-20,-6,-21,-5,-34,-13,-9,-6,};
cal["linus"].footXComp=0.005;
cal["linus"].kickXComp=0.010;

-- Updated date: Mon Jun 18 15:17:18 2012
cal["linus"].servoBias={21,-3,37,-13,-19,-11,-21,-5,-34,10,12,-15,};
cal["linus"].footXComp=0.005;
cal["linus"].kickXComp=0.010;

-- Updated date: Mon Jun 18 20:50:27 2012
cal["lucy"].servoBias={19,-7,10,20,19,-9,-15,-4,-20,0,6,-13,};
cal["lucy"].footXComp=0.006;
cal["lucy"].kickXComp=0.005;

-- Updated date: Mon Jun 18 20:51:17 2012
cal["lucy"].servoBias={19,-7,10,20,19,-9,-15,-4,-20,0,6,-13,};
cal["lucy"].footXComp=0.006;
cal["lucy"].kickXComp=0.005;

-- Updated date: Mon Jun 18 21:12:59 2012
cal["lucy"].servoBias={19,4,10,0,21,-13,-15,7,-20,2,-21,-4,};
cal["lucy"].footXComp=-0.003;
cal["lucy"].kickXComp=0.000;

-- Updated date: Tue Jun 19 09:42:59 2012
cal["sally"].servoBias={-33,-22,-6,0,29,-8,0,0,-4,0,0,0,};
cal["sally"].footXComp=0.005;
cal["sally"].kickXComp=0.005;

-- Updated date: Tue Jun 19 10:47:25 2012
cal["felix"].servoBias={12,-2,37,-31,7,0,-6,30,-33,7,3,18,};
cal["felix"].footXComp=0.005;
cal["felix"].kickXComp=0.005;

-- Updated date: Tue Jun 19 11:17:04 2012
cal["felix"].servoBias={12,-2,37,-31,7,0,-6,30,-33,7,3,18,};
cal["felix"].footXComp=0.003;
cal["felix"].kickXComp=0.005;
-- Updated date: Tue Jun 19 11:15:22 2012
cal["linus"].servoBias={21,-3,37,-13,-19,-11,-21,-5,-30,10,12,16,};
cal["linus"].footXComp=0.005;
cal["linus"].kickXComp=0.010;

-- Updated date: Tue Jun 19 11:34:21 2012
cal["linus"].servoBias={21,-16,37,-13,-11,7,-31,-4,-30,10,0,16,};
cal["linus"].footXComp=0.001;
cal["linus"].kickXComp=0.005;

-- Updated date: Tue Jun 19 11:39:30 2012
cal["linus"].servoBias={21,-16,37,-13,-14,-3,-31,-4,-30,10,0,16,};
cal["linus"].footXComp=0.003;
cal["linus"].kickXComp=0.005;

-- Updated date: Tue Jun 19 10:59:08 2012
cal["betty"].servoBias={8,-10,11,-6,27,-15,-5,6,-26,-1,-20,7,};
cal["betty"].footXComp=0.001;
cal["betty"].kickXComp=0.005;

-- Updated date: Tue Jun 19 11:01:00 2012
cal["betty"].servoBias={8,-10,11,-6,27,-15,-5,6,-26,-1,-20,7,};
cal["betty"].footXComp=0.003;
cal["betty"].kickXComp=0.005;

-- Updated date: Tue Jun 19 11:31:15 2012
cal["betty"].servoBias={8,-10,11,-6,20,-15,-5,6,-26,-1,-10,7,};
cal["betty"].footXComp=0.004;
cal["betty"].kickXComp=0.005;

-- Updated date: Wed Jun 20 17:21:03 2012
cal["hokie"].servoBias={0,-5,-17,-13,-1,-26,0,2,34,-17,-20,5,};
cal["hokie"].footXComp=0.002;
cal["hokie"].kickXComp=0.000;

-- Updated date: Wed Jun 20 17:54:19 2012
cal["felix"].servoBias={12,-2,37,-31,7,-7,-6,30,-33,7,3,20,};
cal["felix"].footXComp=0.003;
cal["felix"].kickXComp=0.005;

-- Updated date: Wed Jun 20 18:11:31 2012
cal["felix"].servoBias={12,-2,37,-31,7,-7,-6,30,-33,7,3,20,};
cal["felix"].footXComp=0.003;
cal["felix"].kickXComp=0.005;

-- Updated date: Thu Jun 21 06:31:14 2012
cal["betty"].servoBias={8,-10,21,-6,20,-15,-5,6,-26,-1,-10,7,};
cal["betty"].footXComp=0.004;
cal["betty"].kickXComp=0.005;

-- Updated date: Thu Jun 21 06:32:27 2012
cal["betty"].servoBias={8,-10,21,-6,20,-15,-5,6,-26,-1,-10,7,};
cal["betty"].footXComp=0.004;
cal["betty"].kickXComp=0.005;

-- Updated date: Thu Jun 21 07:03:29 2012
cal["betty"].servoBias={8,-10,21,-23,36,4,-5,6,-31,2,-40,-13,};
cal["betty"].footXComp=-0.003;
cal["betty"].kickXComp=0.005;

-- Updated date: Thu Jun 21 07:08:31 2012
cal["betty"].servoBias={8,-10,21,-23,36,-11,-5,6,-31,2,-40,14,};
cal["betty"].footXComp=-0.007;
cal["betty"].kickXComp=0.005;
-- Updated date: Thu Jun 21 12:36:09 2012
cal["scarface"].servoBias={0,0,6,50,30,15,0,0,-20,-32,-21,1,};
cal["scarface"].footXComp=-0.005;
cal["scarface"].kickXComp=0.005;

-- Updated date: Thu Jun 21 12:41:54 2012
cal["scarface"].servoBias={0,0,6,50,30,15,0,0,-20,-32,-21,1,};
cal["scarface"].footXComp=-0.001;
cal["scarface"].kickXComp=0.005;

-- Updated date: Thu Jun 21 12:50:06 2012
cal["scarface"].servoBias={0,0,6,50,30,15,0,0,-20,-32,-21,1,};
cal["scarface"].footXComp=-0.003;
cal["scarface"].kickXComp=0.005;

-- Updated date: Thu Jun 21 17:10:25 2012
cal["scarface"].servoBias={0,0,6,50,30,15,0,0,-20,-32,-21,1,};
cal["scarface"].footXComp=-0.005;
cal["scarface"].kickXComp=0.005;

-- Updated date: Thu Jun 21 13:53:36 2012
cal["betty"].servoBias={8,-10,21,-23,36,-11,-5,6,-31,2,-40,14,};
cal["betty"].footXComp=0.000;
cal["betty"].kickXComp=0.005;

-- Updated date: Thu Jun 21 15:18:28 2012
cal["betty"].servoBias={8,-10,21,-23,36,-11,-5,6,-31,-83,-28,6,};
cal["betty"].footXComp=0.016;
cal["betty"].kickXComp=0.005;

-- Updated date: Thu Jun 21 15:33:21 2012
cal["betty"].servoBias={8,-10,21,-77,29,-11,-5,6,-31,-16,-34,6,};
cal["betty"].footXComp=0.002;
cal["betty"].kickXComp=0.000;

-- Updated date: Thu Jun 21 16:57:53 2012
cal["betty"].servoBias={8,-10,21,-77,29,-11,-5,6,-31,-16,-34,6,};
cal["betty"].footXComp=0.000;
cal["betty"].kickXComp=0.000;

-- Updated date: Fri Jun 22 12:20:42 2012
cal["scarface"].servoBias={0,0,12,11,30,15,0,0,0,-46,-21,1,};
cal["scarface"].footXComp=-0.005;
cal["scarface"].kickXComp=0.005;

-- Updated date: Fri Jun 22 12:27:44 2012
cal["scarface"].servoBias={0,0,-1,20,-1,3,-14,0,0,-8,20,-24,};
cal["scarface"].footXComp=-0.005;
cal["scarface"].kickXComp=0.005;

-- Updated date: Fri Jun 22 12:33:40 2012
cal["scarface"].servoBias={0,0,-1,20,-1,3,-14,0,0,-8,20,-24,};
cal["scarface"].footXComp=0.003;
cal["scarface"].kickXComp=0.005;

-- Updated date: Fri Jun 22 23:36:53 2012
cal["jiminy"].servoBias={15,0,41,12,-22,4,-20,0,-23,-8,18,-8,};
cal["jiminy"].footXComp=0.009;
cal["jiminy"].kickXComp=0.000;

-- Updated date: Sat Jun 23 08:52:33 2012
cal["linus"].servoBias={21,-16,37,-13,-8,-3,-31,-4,-30,10,0,16,};
cal["linus"].footXComp=0.003;
cal["linus"].kickXComp=0.005;
