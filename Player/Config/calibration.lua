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

cal["annie"]={
  servoBias={0,0,0,0,0,0, 0,0,0,0,0,0},
  footXComp = 0,
  footYComp = 0,
  kickXComp = 0,
  headPitchComp = 0;
  armBias={0,0,0,0,0,0},
  pid = 1, --NEW FIRMWARE
};

cal["andy"]={
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
--Cleaned up on 06/28/2012 @ GRASP

-- Updated date: Mon Jun 18 21:12:59 2012
cal["lucy"].servoBias={19,4,10,0,21,-13,-15,7,-20,2,-21,-4,};
cal["lucy"].footXComp=-0.003;
cal["lucy"].kickXComp=0.000;

-- Updated date: Tue Jun 19 09:42:59 2012
cal["sally"].servoBias={-33,-22,-6,0,29,-8,0,0,-4,0,0,0,};
cal["sally"].footXComp=0.005;
cal["sally"].kickXComp=0.005;

-- Updated date: Tue Jun 19 11:39:30 2012
cal["linus"].servoBias={21,-16,37,-13,-14,-3,-31,-4,-30,10,0,16,};
cal["linus"].footXComp=0.003;
cal["linus"].kickXComp=0.005;

-- Updated date: Wed Jun 20 17:21:03 2012
cal["hokie"].servoBias={0,-5,-17,-13,-1,-26,0,2,34,-17,-20,5,};
cal["hokie"].footXComp=0.002;
cal["hokie"].kickXComp=0.000;

-- Updated date: Wed Jun 20 18:11:31 2012
cal["felix"].servoBias={12,-2,37,-31,7,-7,-6,30,-33,7,3,20,};
cal["felix"].footXComp=0.003;
cal["felix"].kickXComp=0.005;

-- Updated date: Thu Jun 21 16:57:53 2012
cal["betty"].servoBias={8,-10,21,-77,29,-11,-5,6,-31,-16,-34,6,};
cal["betty"].footXComp=0.000;
cal["betty"].kickXComp=0.000;

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

-- Updated date: Sat Jun 23 08:07:03 2012
cal["betty"].servoBias={8,-10,21,-67,29,-11,-5,6,-31,-24,-34,6,};
cal["betty"].footXComp=0.006;
cal["betty"].kickXComp=0.000;

-- Updated date: Fri Jun 22 11:51:21 2012
cal["sally"].servoBias={-33,-22,-6,0,29,-8,0,0,-4,0,0,0,};
cal["sally"].footXComp=0.005;
cal["sally"].kickXComp=0.000;

-- Updated date: Wed Jul 11 16:23:03 2012
cal["betty"].servoBias={8,-10,21,-67,39,11,-5,6,-31,-24,-43,10,};
cal["betty"].footXComp=0.001;
cal["betty"].kickXComp=0.000;

-- Updated date: Fri Apr 19 15:38:41 2013
cal["linus"].servoBias={21,-16,37,-13,-8,-3,-31,-4,-30,10,0,16,};
cal["linus"].footXComp=0.003;
cal["linus"].kickXComp=0.005;

-- Updated date: Fri Apr 19 15:39:26 2013
cal["linus"].servoBias={21,-16,37,-13,-8,-3,-31,-4,-30,10,0,16,};
cal["linus"].footXComp=-0.003;
cal["linus"].kickXComp=0.005;

-- Updated date: Sat Apr 20 14:06:32 2013
cal["lucy"].servoBias={19,4,40,0,21,-13,-15,7,-20,2,-7,-4,};
cal["lucy"].footXComp=-0.003;
cal["lucy"].kickXComp=0.000;

-- Updated date: Sat Apr 20 14:07:21 2013
cal["lucy"].servoBias={19,4,40,0,21,-13,-15,7,-20,2,-7,-4,};
cal["lucy"].footXComp=-0.003;
cal["lucy"].kickXComp=0.000;

-- Updated date: Sat Apr 20 14:08:31 2013
cal["lucy"].servoBias={19,4,40,0,21,-13,-15,7,-20,2,-7,-4,};
cal["lucy"].footXComp=-0.003;
cal["lucy"].kickXComp=0.000;

-- Updated date: Mon May 27 20:35:56 2013
cal["annie"].servoBias={0,0,0,0,0,0,0,0,0,0,0,0,};
cal["annie"].footXComp=0.000;
cal["annie"].kickXComp=0.000;

-- Updated date: Mon May 27 20:48:02 2013
cal["annie"].servoBias={0,0,0,0,0,0,0,0,0,0,0,0,};
cal["annie"].footXComp=-0.002;
cal["annie"].kickXComp=0.000;

-- Updated date: Wed May 29 00:01:29 2013
cal["annie"].servoBias={0,0,0,0,0,0,0,0,0,0,0,0,};
cal["annie"].footXComp=-0.002;
cal["annie"].kickXComp=0.000;

-- Updated date: Mon May 27 18:51:36 2013
cal["linus"].servoBias={21,-16,37,-13,-8,-3,-31,-4,-30,10,0,16,};
cal["linus"].footXComp=-0.001;
cal["linus"].kickXComp=0.005;

-- Updated date: Mon May 27 16:34:13 2013
cal["betty"].servoBias={8,-10,21,-67,39,11,-5,6,-31,-24,-43,10,};
cal["betty"].footXComp=0.001;
cal["betty"].kickXComp=0.000;

-- Updated date: Sun Jun  2 23:43:29 2013
cal["annie"].servoBias={0,0,0,0,0,0,0,0,0,0,0,0,};
cal["annie"].footXComp=-0.004;
cal["annie"].kickXComp=0.000;

-- Updated date: Sun Jun  2 23:45:18 2013
cal["annie"].servoBias={0,0,0,0,0,0,0,0,0,0,0,0,};
cal["annie"].footXComp=-0.013;
cal["annie"].kickXComp=-0.005;

-- Updated date: Sun Jun  2 23:45:51 2013
cal["annie"].servoBias={0,0,0,0,0,0,0,0,0,0,0,0,};
cal["annie"].footXComp=-0.005;
cal["annie"].kickXComp=-0.005;

-- Updated date: Mon Jun  3 23:07:44 2013
cal["lucy"].servoBias={19,4,40,0,21,-13,-15,7,-20,2,-7,-4,};
cal["lucy"].footXComp=-0.003;
cal["lucy"].kickXComp=0.000;

-- Updated date: Sun Jun 16 19:19:40 2013
cal["annie"].servoBias={0,0,0,0,0,0,-18,0,0,0,0,0,};
cal["annie"].footXComp=-0.005;
cal["annie"].kickXComp=-0.005;

-- Updated date: Sun Jun 16 14:34:44 2013
cal["betty"].servoBias={8,-10,18,-67,39,11,-5,6,-81,-24,-43,10,};
cal["betty"].footXComp=0.001;
cal["betty"].kickXComp=0.000;

-- Updated date: Sun Jun 16 14:35:37 2013
cal["betty"].servoBias={8,-10,18,-67,39,11,-5,6,-81,-24,-43,10,};
cal["betty"].footXComp=-0.008;
cal["betty"].kickXComp=0.000;

-- Updated date: Fri Jun  7 16:35:41 2013
cal["lucy"].servoBias={19,4,27,0,18,-13,3,0,-20,2,-7,-4,};
cal["lucy"].footXComp=-0.003;
cal["lucy"].kickXComp=0.000;

-- Updated date: Sun Jun 16 19:00:52 2013
cal["andy"].servoBias={0,0,0,0,0,0,0,0,-16,-14,0,0,};
cal["andy"].footXComp=-0.003;
cal["andy"].kickXComp=0.000;

-- Updated date: Fri Oct 22 16:14:29 2010
cal["sally"].servoBias={-33,-22,-6,0,29,-8,0,0,-4,0,0,0,};
cal["sally"].footXComp=-0.001;
cal["sally"].kickXComp=0.000;

-- Updated date: Wed Jun 19 20:43:50 2013
cal["linus"].servoBias={21,-16,37,-13,-8,-3,-31,-4,-30,10,0,16,};
cal["linus"].footXComp=-0.001;
cal["linus"].kickXComp=0.005;
-- Updated date: Mon Jun 17 17:27:48 2013

cal["betty"].servoBias={8,-10,18,-67,39,-8,-5,6,-81,-24,-43,10,};
cal["betty"].footXComp=0.004;
cal["betty"].kickXComp=0.000;

-- Updated date: Mon Jun 17 17:30:06 2013
cal["betty"].servoBias={8,-10,18,-67,39,-8,-5,6,-81,-24,-43,10,};
cal["betty"].footXComp=-0.007;
cal["betty"].kickXComp=0.000;

-- Updated date: Mon Jun 17 17:32:11 2013
cal["betty"].servoBias={8,-10,0,-67,39,-8,-5,6,-51,-24,-43,10,};
cal["betty"].footXComp=-0.007;
cal["betty"].kickXComp=0.000;

-- Updated date: Mon Jun 17 17:32:45 2013
cal["betty"].servoBias={8,-10,0,-67,39,-8,-5,6,-51,-24,-43,10,};
cal["betty"].footXComp=-0.001;
cal["betty"].kickXComp=0.000;

-- Updated date: Mon Jun 17 17:39:26 2013
cal["betty"].servoBias={8,-10,0,-67,39,-8,-5,6,-51,-24,-43,10,};
cal["betty"].footXComp=-0.001;
cal["betty"].kickXComp=0.000;

-- Updated date: Mon Jun 17 17:40:03 2013
cal["betty"].servoBias={8,-10,0,-67,39,-8,-5,6,-51,-24,-43,10,};
cal["betty"].footXComp=-0.001;
cal["betty"].kickXComp=0.000;

-- Updated date: Mon Jun 17 17:42:42 2013
cal["betty"].servoBias={8,-10,0,-67,39,-8,-5,6,-51,-24,-43,10,};
cal["betty"].footXComp=-0.001;
cal["betty"].kickXComp=0.000;

-- Updated date: Mon Jun 17 17:51:39 2013
cal["betty"].servoBias={8,-10,0,-67,39,-8,-5,6,-51,-24,-43,10,};
cal["betty"].footXComp=-0.008;
cal["betty"].kickXComp=-0.005;

-- Updated date: Wed Jun 19 17:26:55 2013
cal["betty"].servoBias={8,-10,0,-67,39,-8,-5,6,-51,-24,-43,10,};
cal["betty"].footXComp=-0.008;
cal["betty"].kickXComp=0.010;

-- Updated date: Wed Jun 19 23:23:51 2013
cal["lucy"].servoBias={19,4,27,0,18,-13,3,0,-20,2,-7,-4,};
cal["lucy"].footXComp=-0.003;
cal["lucy"].kickXComp=0.000;

-- Updated date: Wed Jun 19 23:34:02 2013
cal["annie"].servoBias={0,0,0,0,0,0,-18,0,0,0,0,0,};
cal["annie"].footXComp=-0.005;
cal["annie"].kickXComp=-0.005;

-- Updated date: Wed Jun 19 17:37:20 2013
cal["betty"].servoBias={8,-10,0,-67,39,-8,-5,14,-51,-24,-43,6,};
cal["betty"].footXComp=-0.003;
cal["betty"].kickXComp=0.010;

-- Updated date: Wed Jun 19 21:53:42 2013
cal["betty"].servoBias={8,-10,0,-67,39,-8,-5,14,-51,-24,-43,6,};
cal["betty"].footXComp=-0.007;
cal["betty"].kickXComp=0.010;

-- Updated date: Thu Jun 20 11:08:52 2013
cal["betty"].servoBias={8,-10,43,-67,39,-8,-5,14,-104,-24,-43,6,};
cal["betty"].footXComp=-0.007;
cal["betty"].kickXComp=0.010;

-- Updated date: Thu Jun 20 11:18:59 2013
cal["betty"].servoBias={8,-10,43,-67,39,-8,-5,14,-104,-24,-43,6,};
cal["betty"].footXComp=0.005;
cal["betty"].kickXComp=0.010;

-- Updated date: Thu Jun 20 11:24:00 2013
cal["betty"].servoBias={8,-10,43,-67,39,-8,-5,14,-104,-24,-43,6,};
cal["betty"].footXComp=0.005;
cal["betty"].kickXComp=0.005;

-- Updated date: Fri Jun 21 00:52:36 2013
cal["lucy"].servoBias={19,4,27,0,18,-13,3,0,-20,2,-7,-4,};
cal["lucy"].footXComp=-0.003;
cal["lucy"].kickXComp=0.000;

-- Updated date: Tue Jun 25 11:44:43 2013
cal["jiminy"].servoBias={15,0,41,12,0,-14,-20,0,-23,-8,-32,-8,};
cal["jiminy"].footXComp=0.009;
cal["jiminy"].kickXComp=0.000;

-- Updated date: Tue Jun 25 11:45:55 2013
cal["jiminy"].servoBias={15,0,41,12,0,-14,-20,0,-23,-8,-32,-8,};
cal["jiminy"].footXComp=0.009;
cal["jiminy"].kickXComp=0.000;

-- Updated date: Fri Jun 28 18:34:52 2013
cal["lucy"].servoBias={19,4,6,0,18,-13,3,0,-20,2,-7,-4,};
cal["lucy"].footXComp=-0.003;
cal["lucy"].kickXComp=0.000;

-- Updated date: Fri Jun 28 16:06:46 2013
cal["annie"].servoBias={0,0,56,0,0,0,-18,0,-48,0,116,0,};
cal["annie"].footXComp=-0.011;
cal["annie"].kickXComp=-0.005;

-- Updated date: Fri Jun 28 16:09:08 2013
cal["annie"].servoBias={0,0,56,0,0,0,-18,0,-48,0,116,0,};
cal["annie"].footXComp=-0.005;
cal["annie"].kickXComp=-0.005;

-- Updated date: Fri Jun 28 16:11:27 2013
cal["annie"].servoBias={0,0,56,0,0,0,-18,0,-48,0,116,0,};
cal["annie"].footXComp=0.000;
cal["annie"].kickXComp=-0.005;

-- Updated date: Fri Jun 28 14:32:21 2013
cal["andy"].servoBias={0,0,18,0,0,0,0,0,-22,-14,0,0,};
cal["andy"].footXComp=-0.003;
cal["andy"].kickXComp=0.000;

-- Updated date: Fri Jun 28 16:47:11 2013
cal["jiminy"].servoBias={52,0,-2,12,1,-14,-13,0,60,11,-7,-8,};
cal["jiminy"].footXComp=0.009;
cal["jiminy"].kickXComp=0.000;

-- Updated date: Fri Jun 28 16:49:03 2013
cal["jiminy"].servoBias={52,0,-2,12,1,-14,-13,0,60,11,-7,-8,};
cal["jiminy"].footXComp=0.005;
cal["jiminy"].kickXComp=0.000;

-- Updated date: Sat Jun 29 16:34:39 2013
cal["linus"].servoBias={21,-16,37,-13,-8,-3,-31,-4,-30,-16,0,16,};
cal["linus"].footXComp=-0.001;
cal["linus"].kickXComp=0.005;

-- Updated date: Sat Jun 29 16:38:12 2013
cal["linus"].servoBias={21,5,24,1,-11,-3,-31,-4,-30,-11,0,16,};
cal["linus"].footXComp=0.002;
cal["linus"].kickXComp=0.005;

-- Updated date: Sat Jun 29 16:41:36 2013
cal["linus"].servoBias={21,-19,24,1,-10,-3,-31,-20,-30,-11,0,-3,};
cal["linus"].footXComp=0.002;
cal["linus"].kickXComp=0.005;

-- Updated date: Sat Jun 29 16:43:33 2013
cal["linus"].servoBias={21,-19,24,1,-10,-3,-31,-20,-30,-11,0,-3,};
cal["linus"].footXComp=0.001;
cal["linus"].kickXComp=0.005;

-- Updated date: Sat Jun 29 19:17:12 2013
cal["lucy"].servoBias={19,4,6,0,18,-13,3,14,-20,2,3,-4,};
cal["lucy"].footXComp=-0.003;
cal["lucy"].kickXComp=0.000;

-- Updated date: Sat Jun 29 19:18:08 2013
cal["lucy"].servoBias={19,4,6,0,18,-13,3,14,-20,2,3,-4,};
cal["lucy"].footXComp=0.001;
cal["lucy"].kickXComp=0.000;
