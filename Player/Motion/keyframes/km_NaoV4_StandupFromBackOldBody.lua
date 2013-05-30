local mot={};
mot.servos={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,};
mot.keyframes={  

  { --Laying on back, straighten out joints
   angles={
      0.0014920234680176,   --HeadYaw
      0.44481801986694,     --HeadPitch
      1.9159240722656,      --LShoulderPitch
      0.19170808792114,     --LShoulderRoll
      -1.5524501800537,     --LElbowYaw
      -0.37425398826599,    --LElbowRoll
      -0.23926210403442,    --LHipYawPitch
      -0.0060939788818359,  --LHipRoll
      0.29917192459106,     --LHipPitch
      -0.098217964172363,   --LKneePitch
      0.090464115142822,    --LAnklePitch
      0.13196587562561,     --LAnkleRoll
      0,                    --RHipYawPitch
      -0.032171964645386,   --RHipRoll
      0.25460195541382,     --RHipPitch
      -0.01990008354187,    --RKneePitch
      0.085946083068848,    --RAnklePitch
      -0.047512054443359,   --RAnkleRoll
      1.9175419807434,      --RShoulderPitch
      -0.17798590660095,    --RShoulderRoll
      1.5477640628815,      --RElbowYaw
      0.34672594070435,     --RElbowRoll
    },
  duration = .4; 
  },
  { --Prop up from back with hands, pull legs in
    angles={
      -0.13349986076355,    --HeadYaw
      0.53072214126587,     --HeadPitch
      2.1153440475464,      --LShoulderPitch
      0.13648414611816,     --LShoulderRoll
      -0.039926052093506,   --LElbowYaw
      -0.059783935546875,   --LElbowRoll
      -0.015298128128052,   --LHipYawPitch
      0.0046439170837402,   --LHipRoll
      0.41882395744324,     --LHipPitch
      1.1489241123199,      --LKneePitch
      0.49237203598022,     --LAnklePitch
      -0.0091619491577148,  --LAnkleRoll
      0,                    --RHipYawPitch
      -0.061317920684814,   --RHipRoll
      0.38959407806396,     --RHipPitch
      1.1781539916992,      --RKneePitch
      0.4617760181427,      --RAnklePitch
      -0.04597806930542,    --RAnkleRoll
      2.1123600006104,      --RShoulderPitch
      -0.1641800403595,     --RShoulderRoll
      -0.093616008758545,   --RElbowYaw
      0.16571402549744,     --RElbowRoll
    },
  duration = .4; 
  },
  { --Arms pulled to tailbone and wrists used as fulcrum; legs kick robot onto back 
    angles={
      -0.14423799514771,    --HeadYaw
      0.52611994743347,     --HeadPitch
      2.0969362258911,      --LShoulderPitch
      0.19937801361084,     --LShoulderRoll
      0.01990008354187,     --LElbowYaw
      -1.4265780448914,     --LElbowRoll
      -0.012229919433594,   --LHipYawPitch
      -0.0014920234680176,  --LHipRoll
      0.32985186576843,     --LHipPitch
      1.133584022522,       --LKneePitch
      0.54912996292114,     --LAnklePitch
      0.042994022369385,    --LAnkleRoll
      0,                    --RHipYawPitch
      -0.061317920684814,   --RHipRoll
      0.34204006195068,     --RHipPitch
      1.1305999755859,      --RKneePitch  
      0.5216019153595,      --RAnklePitch
      0.013848066329956,    --RAnkleRoll
      2.1169619560242,      --RShoulderPitch
      -0.25161790847778,    --RShoulderRoll
      -0.044528007507324,   --RElbowYaw
      1.4450697898865,      --REblowRoll
    },
duration = .4; 
  },
  { --Tuck legs in
    angles={ 
      -0.14883995056152,    --HeadYaw
      0.52611994743347,     --HeadPitch
      1.9036520719528,      --LShoulderPitch
      0.24693202972412,     --LShoulderRoll
      0.13188195228577,     --LElbowYaw
      -1.4772000312805,     --LElbowRoll
      -0.24693202972412,    --LHipYawPitch
      -0.07052206993103,    --LHipRoll
      -1.5968520641327,     --LHipPitch
      2.1138100624084,      --LKneePitch
      -1.1904258728027,     --LAnklePitch
      0.10281991958618,     --LAnkleRoll
      0,                    --RHipYawPitch
      0.0552659034729,      --RHipRoll
      -1.6214799880981,     --RHipPitch
      2.1246318817139,      --RKneePitch
      -1.1872740983963,     --RAnklePitch
      -0.039842128753662,   --RAnkleRoll
      2.1000881195068,      --RShoulderPitch
      -0.28383207321167,    --RShoulderRoll
      0.04597806930542,     --RElbowYaw
      1.4987602233887,      --RElbowRoll
    },
duration = .4; 
  },
  { --Throw legs out
    angles={ 
      -0.14577198028564,    --HeadYaw
      0.52611994743347,     --HeadPitch
      1.9005841016769,      --LShoulderPitch
      0.27147603034973,     --LShoulderRoll
      0.12114405632019,     --LElbowYaw
      -1.4971420764923,     --LElbowRoll
      -0.51078009605408,    --LHipYawPitch
      0.0031099319458008,   --LHipRoll
      -0.62122797966003,    --LHipPitch
      -0.092082023620605,   --LKneePitch
      0.85439610481262,     --LAnklePitch
      0.096683979034424,    --LAnkleRoll
      0,                    --RHipYawPitch
      0.064470052719116,    --RHipRoll
      -0.64125394821167,    --RHipPitch
      -0.093532085418701,   --RKneePitch
      0.93271398544312,     --RAnklePitch
      0.072139978408813,    --RAnkleRoll
      2.041796207428,       --RShoulderPitch
      -0.30684185028076,    --RShoulderRoll
      -4.1961669921875e-05, --RElbowYaw
      1.5217700004578,      --RElbowRoll
    },
duration = .05; 
  },
  { --Hold leg throw, allowing body to swing up
    angles={
      -0.14423799514771,    --HeadYaw
      0.51845002174377,     --HeadPitch
      1.899050116539,       --LShoulderPitch
      0.27147603034973,     --LShoulderRoll
      0.12267804145813,     --LElbowYaw
      -1.4986760616302,     --LElbowRoll
      -0.50464415550232,    --LHipYawPitch
      0.027653932571411,    --LHipRoll
      -0.13648414611816,    --LHipPitch
      -0.092082023620605,   --LKneePitch
      0.8129780292511,      --LAnklePitch
      0.10128593444824,     --LAnkleRoll
      0,                    --RHipYawPitch
      0.033789873123169,    --RHipRoll
      -0.096683979034424,   --RHipPitch
      -0.093532085418701,   --RKneePitch
      0.88362598419189,     --RAnklePitch
      0.070605993270874,    --RAnkleRoll
      2.04026222229,        --RShoulderPitch
      -0.30684185028076,    --RShoulderRoll
      0.0014920234680176,   --RElbowYaw
      1.5248379707336,      --RElbowRoll
    },
duration = .05; 
  },
  { --Sit up
    angles={
      -0.20866584777832,    --HeadYaw
      0.47242999076843,     --HeadPitch
      2.0877320766449,      --LShoulderPitch
      0.27301001548767,     --LShoulderroll
      0.07205605506897,     --LElbowYaw
      -1.4879380464554,     --LElbowRoll
      -0.48163414001465,    --LHipYawPitch
      0.1120240688324,      --LHipRoll
      -0.88047409057617,    --LHipPitch
      -0.092082023620605,   --LKneePitch
      0.8129780292511,      --LAnklePitch
      0.096683979034424,    --LAnkleRoll
      0,                    --RHipYawPitch
      -0.28067994117737,    --RHipRoll
      -0.86675214767456,    --RHipPitch
      -0.093532085418701,   --RKneePitch
      0.93271398544312,     --RAnklePitch
      0.070605993270874,    --RAnkleRoll
      2.0494661331177,      --RShoulderPitch
      -0.28383207321167,    --RShoulderRoll
      -0.0077118873596191,  --RElbowYaw
      1.4849538803101,      --RElbowRoll
    },
duration = .1; 
  },
  { --Straighten out arms,
    angles={
      -0.03072190284729,    --HeadYaw
      0.51998400688171,     --HeadPitch
      2.1184120178223,      --LShoulderPitch
      0.48623609542847,     --LShoulderroll
      -0.08134388923645,    --LElbowYaw
      -0.7684919834137,     --LElbowRoll
      -0.44481801986694,    --LHipYawPitch
      0.15804386138916,     --LHipRoll
      -1.078360080719,      --LHipPitch
      -0.093616008758545,   --LKneePitch
      0.81451201438904,     --LAnklePitch
      0.095149993896484,    --LAnkleRoll
      0,                    --RHipYawPitch
      -0.27761197090149,    --RHipRoll
      -1.0477638244629,     --RHipPitch
      -0.093532085418701,   --RKneePitch
      0.93271398544312,     --RAnklePitch
      0.076741933822632,    --RAnkleRoll
      2.0694079399109,      --RShoulderPitch
      -0.39274597167969,    --RShoulderRoll
      -0.17338395118713,    --RElbowYaw
      0.56148600578308,     --RElbowRoll
    },
duration = .4; 
  },

  { --Wait for 0.4s
    angles={
      -0.03072190284729,    --HeadYaw
      0.51998400688171,     --HeadPitch
      2.1184120178223,      --LShoulderPitch
      0.48623609542847,     --LShoulderRoll
      -0.08134388923645,    --LElbowYaw
      -0.7684919834137,     --LElbowRoll
      -0.44481801986694,    --LHipYawPitch
      0.15804386138916,     --LHipRoll
      -1.078360080719,      --LHipPitch
      -0.093616008758545,   --LKneePitch
      0.81451201438904,     --LAnklePitch
      0.095149993896484,    --LAnkleRoll
      0,                    --RHipYawPitch
      -0.27761197090149,    --RHipRoll
      -1.0477638244629,     --RHipPitch
      -0.093532085418701,   --RKneePitch
      0.93271398544312,     --RAnklePitch
      0.076741933822632,    --RAnkleRoll
      2.0694079399109,      --RShoulderPitch
      -0.39274597167969,    --RShoulderRoll
      -0.17338395118713,    --RElbowYaw
      0.56148600578308,     --RElbowRoll
    },
duration = .4; 
  },

  { --Pull legs in, put feet flat on the ground
    angles = {
      0.000,    --HeadYaw 
      0.349,    --HeadPitch
      2.094,    --LShoulderPitch
      0.000,    --LShoulderRoll
      0.087,    --LElbowYaw
      0.000,    --LElbowRoll
      -0.663,   --LHipYawPitch
      0.541,    --LHipRoll
      -1.571,   --LHipPitch
      1.676,    --LKneePitch
      0.785,    --LAnklePitch
      0.000,    --LAnkleRoll
      -0.524,   --RHipYawPitch
      -0.541,   --RHipRoll
      -1.571,   --RHipPitch
      1.676,    --RKneePitch
      0.785,    --RAnklePitch
      0.000,    --RAnkleRoll
      2.094,    --RShoulderPitch
      0.000,    --RShoulderRoll
      -0.087,   --RElbowYaw
      0.000,    --RElbowRoll
    },
    duration = 0.700;
  },

  { --Lean back on right hand, lean right with body
    angles = {
      -0.009,   --HeadYaw
      0.332,    --HeadPitch
      0.698,    --LShoulderPitch
      1.047,    --LShoulderRoll
      0.087,    --LElbowYaw
      -0.489,   --LElbowRoll
      -0.489,   --LHipYawPitch
      0.157,    --LHipRoll
      -0.855,   --LHipPitch
      2.199,    --LKneePitch
      -0.559,   --LAnklePitch
      -0.384,   --LAnkleRoll
      -0.524,   --RHipYawPitch
      -0.559,   --RHipRoll
      -1.518,   --RHipPitch
      1.222,    --RKneePitch
      0.785,    --RAnklePitch
      0.009,    --RAnkleRoll
      2.094,    --RShoulderPitch
      -0.576,   --RShoulderRoll
      -0.080,   --RElbowYaw
      0.070,    --RElbowRoll
      },
    duration = 0.700;
  },

  { --Put weight on feet, bending forward and leaning right
    angles = {
      -0.010,   --HeadYaw 
      0.332,    --HeadPitch
	    0.733,    --LShoulderPitch
      0.506,    --LShoulderRoll
      0.087,    --LElbowYaw
      -0.820,   --LElbowRoll
	    -0.873,   --LHipYawPitch
      -0.297,   --LHipRoll
      0.384,    --LHipPitch
      1.780,    --LKneePitch
      -1.222,   --LAnklePitch
      -0.105,   --LAnkleRoll
	    -0.524,   --RHipYawPitch
      -0.559,   --RHipRoll
      -1.571,   --RHipPitch
      1.065,    --RKneePitch
      0.698,    --RAnklePitch
      -0.129,   --RAnkleRoll
	    1.780,    --RShoulderPitch
      -0.262,   --RShoulderRoll
      -0.087,   --RElbowYaw
      0.052,    --RElbowRoll
      },
    duration = 0.750;
  },



  { --Lean onto left, slide right foot in
    angles = {
      -0.009,   --HeadYaw
      0.384,    --HeadPitch
	    0.733,    --LShoulderPitch
      0.506,    --LShoulderRoll
      0.087,    --LElbowYaw
      -0.803,   --LElbowRoll
 	    -0.401,   --LHipYawPitch
      0.192,    --LHipRoll
      -0.855,   --LHipPitch
      2.199,    --LKneePitch
      -1.222,   --LAnklePitch
      0.122,    --LAnkleRoll
 	    -0.873,   --RHipYawPitch
      -0.297,   --RHipRoll
      -0.890,   --RHipPitch
      0.873,    --RKneePitch
      0.401,    --RAnklePitch
      0.681,    --RAnkleRoll
 	    0.890,    --RSHoulderPitch
      -0.873,   --RShoulderRoll
      0.000,    --RElbowYaw
      0.454,    --RElbowRoll
      },
    duration = 1.050;
  },


  { --Right foot under shoulder, still leaning a bit left
    angles = {
      -0.009,   --HeadYaw
      0.384,    --HeadPitch
		  0.733,    --LShoulderPitch
      0.506,    --LShoulderRoll
      0.087,    --LElbowYaw
      -0.803,   --LElbowRoll
		  -0.401,   --LHipYawPitch
      0.367,    --LHipRoll
      -0.838,   --LHipPitch
      2.199,    --LKneePitch
      -1.222,   --LAnklePitch
      0.087,    --LAnkleRoll
		  -0.401,   --RHipYawPitch
      -0.017,   --RHipRoll
      -0.890,   --RHipPitch
      1.763,    --RKneePitch
      -0.471,   --RAnklePitch
      0.279,    --RAnkleRoll
		  0.890,    --RShoulderPitch
      -0.681,   --RShoulderRoll
      0.000,    --RElbowYaw
      0.559,    --RElbowRoll
      },
    duration = 1.050;
  },
  { --Feet square under shoulders, arms curled in front; weight centered
    angles = {
      -0.009,   --HeadYaw 
      0.384,    --HeadPitch
      1.710,    --LShoulderPitch
      0.209,    --LShoulderRoll
      -1.257,   --LElbowYaw
      -1.134,   --LElbowRoll
      0.000,    --LHipYawPitch
      0.000,    --LHipRoll
      -0.873,   --LHipPitch
      2.094,    --LKneePitch
      -1.222,   --LanklePitch
      0.000,    --LAnkleRoll
      -0.401,   --RHipYawPitch
      0.000,    --RHipRoll
      -0.873,   --RHipPitch
      2.094,    --RKneePitch
      -1.222,   --RAnklePitch
      0.000,    --RAnkleRoll
      1.710,    --RShoulderPitch
      -0.209,   --RShoulderRoll
      1.257,    --RElbowYaw
      1.134,    --RElbowRoll
      },
    duration = 0.840;
  },
  { --Head tilt back, begin standing up
    angles = {
      0.000,    --HeadYaw
      -0.436,   --HeadPitch
      2.094,    --LShoulderPitch
      0.349,    --LShoulderRoll
      -1.396,   --LElbowYaw
      -1.396,   --LElbowRoll
      0.000,    --LHipYawPitch
      0.017,    --LHipRoll
      -0.723,   --LHipPitch
      1.490,    --LKneePitch
      -0.767,   --LAnklePitch
      -0.017,   --LAnkleRoll
      -0.000,   --RHipYawPitch
      -0.017,   --RHipRoll
      -0.723,   --RHipPitch
      1.490,    --RKneePitch
      -0.767,   --RAnklePitch
      0.017,    --RAnkleRoll
      2.094,    --RShoulderPitch
      -0.349,   --RShoulderRoll
      1.396,    --RElbowYaw
      1.396,    --RElbowRoll
      },
    duration = 1.050;
  },
  { --Standing at full height; recovered
    angles = {
      -0.066,   --HeadYaw
      -0.678,   --HeadPitch
      1.468,    --LShoulderPitch
      0.229,    --LShoulderRoll
      -1.273,   --LElbowYaw
      -0.305,   --LElbowRoll
      0.000,    --LHipYawPitch
      -0.003,   --LHipRoll
      -0.396,   --LHipPitch
      0.946,    --LKneePitch
      -0.548,   --LAnklePitch
      0.002,    --LAnkleRoll
      0.000,    --RHipYawPitch
      0.026,    --RHipRoll
      -0.397,   --RHipPitch
      0.945,    --RKneePitch
      -0.548,   --RAnklePitch
      -0.025,   --RAnkleRoll
      1.494,    --RShoulderPitch
      -0.253,   --RShoulderRoll
      1.216,    --RElbowYaw
      0.502,    --RElbowRoll
      },
    duration = 1.050;
  },


};

return mot;
