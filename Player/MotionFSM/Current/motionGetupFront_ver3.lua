--[[ DESCRIPTION 
/* 
  moving : final target positions are assigned once
  support : keep track its position w.r.t ground
  free : keep track its position w.r.t body

  1) Put two hands next to the shoulders
    moving : larm, rarm
    support : knees, foot
    free : torso

  2) Push up
    moving : larm, rarm
    support : knees, foot
    free : torso

  3) Pull rarm 
  	moving : rarm
  	support : knees, foot
  	free :torso

  4) Pull larm & move CM backward(WaistPitch, Legs[3], Legs[4])
  	moving : larm, WaistPitch, rleg, lleg
  	support: knee, foot, rarm
  	free : WaistRoll

  5) Put the rfoot at the knee position using IK
  	init yes, '_new' file reference

  	moving : rleg
  	support : lleg([1] tilt, larm, rarm(increased heights), WaistPitch(if necessary for arms: if target > (armlength-shoulder2g[3])? then pitch bend)
  	free :  WaistRoll

  6) Put the lfoot at the knee position using IK & fold legs a little bit so that the body stand up a little bit & Use Waist_Pitch in order to keep the arms on the ground (max)

  7) Consider CM [( CMz increase : fold legs, fold waist), (|CMx-footx| -> 0: fold or bend legs, Arms[1]), CMy = 0), no arm orientation constraints

  8) Final Initial Pose 
/*
--]]


local state = {}
state._NAME = ...

local Body    = require'Body'
local vector  = require'vector'
local unix    = require'unix'
local util    = require'util'
local moveleg = require'moveleg'
local movearm = require'movearm'
local atan2   = math.atan2
local sqrt    = math.sqrt
local cos     = math.cos
local sin     = math.sin

require'mcm'
require'hcm'

local simple_ipc = require'simple_ipc'
local head_ch   = simple_ipc.new_publisher('HeadFSM!')

local qLArm0, qRArm0,qLLeg0,qRLeg0,qWaist0
local qLArm1, qRArm1,qLLeg1,qRLeg1,qWaist1
-- Constants
local pitch_offset = 15*DEG_TO_RAD

local knee_offset = 0.02
local stagemax = 10
local stop = 0

local gain1 = 0.01
local gain2 = 0.03
local gain3 = 0.08
local Kp = 0.35
local Kd = -0.1

--Robot Paramters
local neckOffsetZ = .165+.161; --Webots value
local neckOffsetX = 0;

local shoulderOffsetX = 0;    
local shoulderOffsetY = 0.234;
local shoulderOffsetZ = 0.165;

local upperArmLength = .246;
local elbowOffsetX =   .030; 

local lowerArmLength = .186; --Default 7DOF arm
local lowerArmLength = .250; --LONGARM model

local handOffsetX = 0.245; --Measured from robot
local handOffsetY = 0.035; --Measured from robot
local handOffsetZ = 0;

local hipOffsetX = 0;
local hipOffsetY = 0.072;
local hipOffsetZ = 0.282; 

local thighLength = 0.30;
local tibiaLength = 0.30;
local kneeOffsetX = 0.03;

local footHeight = 0.118; -- Webots value
local footToeX = 0.130; --from ankle to toe
local footHeelX = 0.130; --from ankle to heel

local DEG_TO_RAD = math.pi/180

--Initialize
local stage = 0
local mov = vector.new{0,0,0,0,0}
local ori_ctrl_larm, ori_ctrl_rarm, ori_ctrl_lleg, ori_ctrl_rleg = 0
local l_trPos_init2g, r_trPos_init2g, l_trPos2g, r_trPos2g, l_trPos, r_trPos, l_trPos_init, r_trPos_init = vector.new{0,0,0}


function state.entry()

  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_start = t_entry
  
  qLArm0, qLArm_init, qLArm1 = vector.new(Body.get_larm_position())  
  qRArm0, qRArm_init, qRArm1 = vector.new(Body.get_rarm_position())  
  qLLeg0, qLLeg_init, qLLeg1 = vector.new(Body.get_lleg_position())  
  qRLeg0, qRLeg_init, qRLeg1 = vector.new(Body.get_rleg_position())  
  qWaist0, qWaist_init, qWaist1 = vector.new(Body.get_waist_position())  
  
  f1 = io.open("com.txt", "w")
  f2 = io.open("rpy.txt", "w")

  setting = 1
  stage = 1  
  hcm.set_state_proceed(1)
  head_ch:send'teleop'
  body_ch:send'init'

end

function state.update()
	-- Real time updates:
	local t = Body.get_time()
	local t_diff = t - t_start

	local rpy = Body.get_rpy()	

	-- Check wheter the robot was fallen down or not
	if math.abs(rpy[1])<45*math.pi/180 and math.abs(rpy[2])<45*math.pi/180 then      
	return "done"
	end

	--[[ Rotation Matrices ]]--------------------------------------------------------------------------------------------
	-- My convention : about x(yaw), about y(pitch), about z(roll)
	-- convention change
	local roll = rpy[3]
	local yaw  = rpy[1]
	rpy[1] = roll
	rpy[3] = yaw

	rpyR_b2g ={} -- RPY rotation matrix of body w.r.t the global frame
	rpyR_b2g[1] = {}
	  rpyR_b2g[1][1] = cos(rpy[1])*cos(rpy[2])
	  rpyR_b2g[1][2] = cos(rpy[1])*sin(rpy[2])*sin(rpy[3]) - sin(rpy[1])*cos(rpy[3])
	  rpyR_b2g[1][3] = cos(rpy[1])*sin(rpy[2])*cos(rpy[3]) + sin(rpy[1])*sin(rpy[3])
	rpyR_b2g[2] = {}
	  rpyR_b2g[2][1] = sin(rpy[1])*cos(rpy[2])
	  rpyR_b2g[2][2] = sin(rpy[1])*sin(rpy[2])*sin(rpy[3]) + cos(rpy[1])*cos(rpy[3])
	  rpyR_b2g[2][3] = sin(rpy[1])*sin(rpy[2])*cos(rpy[3]) - cos(rpy[1])*sin(rpy[3]) 
	rpyR_b2g[3] = {}
	  rpyR_b2g[3][1] = -sin(rpy[2])
	  rpyR_b2g[3][2] = cos(rpy[2])*sin(rpy[3])
	  rpyR_b2g[3][3] = cos(rpy[2])*cos(rpy[3])

	rpy_target = {}
	rpy_target[1] = rpy[1]--roll
	rpy_target[2] = 0 --pitch
	rpy_target[3] = 0--rpy[3] --yaw
	rpyR_tr2g = {} -- RPY rotation matrix of the end-effector with targeted orientations w.r.t the global frame(super script)
	rpyR_tr2g[1] = {}
	  rpyR_tr2g[1][1] = cos(rpy_target[1])*cos(rpy_target[2])
	  rpyR_tr2g[1][2] = cos(rpy_target[1])*sin(rpy_target[2])*sin(rpy_target[3]) - sin(rpy_target[1])*cos(rpy_target[3])
	  rpyR_tr2g[1][3] = cos(rpy_target[1])*sin(rpy_target[2])*cos(rpy_target[3]) + sin(rpy_target[1])*sin(rpy_target[3])
	rpyR_tr2g[2] = {}
	  rpyR_tr2g[2][1] = sin(rpy_target[1])*cos(rpy_target[2])
	  rpyR_tr2g[2][2] = sin(rpy_target[1])*sin(rpy_target[2])*sin(rpy_target[3]) + cos(rpy_target[1])*cos(rpy_target[3])
	  rpyR_tr2g[2][3] = sin(rpy_target[1])*sin(rpy_target[2])*cos(rpy_target[3]) - cos(rpy_target[1])*sin(rpy_target[3]) 
	rpyR_tr2g[3] = {}
	  rpyR_tr2g[3][1] = -sin(rpy_target[2])
	  rpyR_tr2g[3][2] = cos(rpy_target[2])*sin(rpy_target[3])
	  rpyR_tr2g[3][3] = cos(rpy_target[2])*cos(rpy_target[3])

	rpyR_tr2b = util.mul(util.transpose(rpyR_b2g),rpyR_tr2g)

	--[[Get current values of all joints]]----------------------------------------------------------------------------

	qLArm_current   = Body.get_larm_position();
	qRArm_current   = Body.get_rarm_position();
	qLLeg_current   = Body.get_lleg_position();
	qRLeg_current   = Body.get_rleg_position();
	qWaist_current  = Body.get_waist_position();
	qHead_current   = Body.get_head_position();

	currentLArm = Body.get_forward_larm(qLArm_current)
	currentRArm = Body.get_forward_rarm(qRArm_current)
	currentLLeg = Body.get_forward_lleg(qLLeg_current)
	currentRLeg = Body.get_forward_rleg(qRLeg_current)

	l_armpos2g = util.mulvec(rpyR_b2g,vector.slice(currentLArm,1,3))
	r_armpos2g = util.mulvec(rpyR_b2g,vector.slice(currentRArm,1,3))
	l_shoulder = Body.get_forward_larm_origins(qLArm_current,2)
		l_shoulder2g = util.mulvec(rpyR_b2g, vector.slice(l_shoulder,1,3))
	r_shoulder = Body.get_forward_rarm_origins(qRArm_current,2)
		r_shoulder2g = util.mulvec(rpyR_b2g, vector.slice(r_shoulder,1,3))
	l_hip      = Body.get_forward_lleg_origins(qLLeg_current,2)
	r_hip      = Body.get_forward_rleg_origins(qRLeg_current,2)
		l_hip_posg = util.mulvec(rpyR_b2g,vector.slice(l_hip,1,3))
		r_hip_posg = util.mulvec(rpyR_b2g,vector.slice(r_hip,1,3))
	l_knee     = Body.get_forward_lleg_origins(qLLeg_current,4)
	r_knee     = Body.get_forward_rleg_origins(qRLeg_current,4)
		l_knee_posg = util.mulvec(rpyR_b2g,vector.slice(l_knee,1,3))
		r_knee_posg = util.mulvec(rpyR_b2g,vector.slice(r_knee,1,3))
			l_knee_posg[3] = l_knee_posg[3] - tibiaLength*sin(qLLeg_current[4]+qLLeg_current[3]-(1.57-rpy[2]))
		   	r_knee_posg[3] = r_knee_posg[3] - tibiaLength*sin(qRLeg_current[4]+qRLeg_current[3]-(1.57-rpy[2]))
	l_foot     = Body.get_forward_lleg_origins(qLLeg_current,6)
	r_foot     = Body.get_forward_rleg_origins(qRLeg_current,6)
		l_foot_posg = util.mulvec(rpyR_b2g,vector.slice(l_foot,1,3))
		r_foot_posg = util.mulvec(rpyR_b2g,vector.slice(r_foot,1,3))

	lleg_current2g   = util.mulvec(rpyR_b2g,vector.slice(currentLLeg,1,3))
	rleg_current2g   = util.mulvec(rpyR_b2g,vector.slice(currentRLeg,1,3))

	--[[ Find the lowest joint ]]---------------------------------------------------------------------------------------------

	numJoint = {};
	numJoint[1] = 7
	numJoint[2] = 7
	numJoint[3] = 6
	numJoint[4] = 6

	j_heights = {}
	local jointCount = 0

	for i=1,numJoint[1] do
	  or_pos = Body.get_forward_larm_origins(qLArm_current,i)
	  j_heights[jointCount+i] = rpyR_b2g[3][1]*or_pos[1] + rpyR_b2g[3][2]*or_pos[2] + rpyR_b2g[3][3]*or_pos[3]
	end
	jointCount = jointCount + numJoint[1]

	for i=1,numJoint[2] do
	  or_pos = Body.get_forward_rarm_origins(qRArm_current,i)
	  j_heights[jointCount+i] = rpyR_b2g[3][1]*or_pos[1] + rpyR_b2g[3][2]*or_pos[2] + rpyR_b2g[3][3]*or_pos[3]
	end
	jointCount = jointCount + numJoint[2]

	for i=1,numJoint[3] do
	  or_pos = Body.get_forward_lleg_origins(qLLeg_current,i)
	  j_heights[jointCount+i] = rpyR_b2g[3][1]*or_pos[1] + rpyR_b2g[3][2]*or_pos[2] + rpyR_b2g[3][3]*or_pos[3]
	end
	jointCount = jointCount + numJoint[3]

	for i=1,numJoint[4] do
	  or_pos = Body.get_forward_rleg_origins(qRLeg_current,i)
	  j_heights[jointCount+i] = rpyR_b2g[3][1]*or_pos[1] + rpyR_b2g[3][2]*or_pos[2] + rpyR_b2g[3][3]*or_pos[3]
	end
	jointCount = jointCount + numJoint[4]

	baseframe_h = util.min(j_heights) -- negative

	--[[ WRITING ]]----------------------------------------------------------------------------------------------------------
	
	if (lleg_current2g[3] - baseframe_h) < (rleg_current2g[3] - baseframe_h) then
		com_pos = Body.get_calculate_com_pos_global(qWaist_current, qLArm_current,qRArm_current,qLLeg_current,qRLeg_current,vector.slice(currentLLeg,1,3),0)
	else
		com_pos = Body.get_calculate_com_pos_global(qWaist_current, qLArm_current,qRArm_current,qLLeg_current,qRLeg_current,vector.slice(currentRLeg,1,3),1)
	end
	f1:write(stage)
	for k = 1,#com_pos do
		f1:write("\t",com_pos[k])
	end
	f1:write("\n")
	f2:write( stage,"\t",rpy[1],"\t",rpy[2],"\t",rpy[3],"\n")

	---------------------------------------------------------------------------------------------------------------------------------------- 
	--[[ STAGES ]]--------------------------------------------------------------------------------------------------------------------------
  	headangle = -util.sign(rpy[2])*30*DEG_TO_RAD
  	hcm.set_motion_headangle({0,headangle})
  	
  	if stage == 1 then
  		if setting ==1 then
  			init = 0
  			duration = 1
  			mov = vector.new{1,1,1,1,1}

  			ori_ctrl_rarm = 0
  			ori_ctrl_larm = 0

  			qLArm1 = qLArm_current
  				qLArm1[1] = util.sign(rpy[2])*2.8
  				qLArm1[4] = qLArm1[4]
  				qLArm1[7] = qLArm1[7] - 90*DEG_TO_RAD
  			qRArm1 = qRArm_current
  				qRArm1[1] = util.sign(rpy[2])*2.8
  				qRArm1[4] = qRArm1[4]
  				qRArm1[7] = qRArm1[7] + 90*DEG_TO_RAD

  			qLLeg1 = qLLeg_current
  			qRLeg1 = qRLeg_current
  			qWaist1 = qWaist_current

  			setting = 0
  		end

	elseif stage == 2 then
		-- Moving : larm, rarm
		-- Support : lknee, lfoot, rknee, rfoot
		-- Free : Torso
		print("STAGE2")
		if setting == 1 then -- Set parameters for the stage at first.
			--** STAGE PARAMETERS **--
			init = 1
			duration = 3
			mov = vector.new{1,1,0,0,1}
			stage_h = baseframe_h -- 0.15

			ori_ctrl_larm = 0
			ori_ctrl_rarm = 0

			-- LEFT ARM
			l_trPos2g = l_shoulder2g 
				l_trPos2g[3] = stage_h
			l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
				l_trPos[2] = l_trPos[2] + util.sign(l_trPos[2])*0.2 -- body y direction
				l_trPos[3] = l_trPos[3] + 0.2

			trL = util.concat(l_trPos, util.ypr(rpyR_tr2b,1,vector.slice(currentLArm,4,6)))
			qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
		    	qLArm1[6] = qLArm1[6] - util.sign(qLArm1[6])*5*DEG_TO_RAD
		    	qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD

			-- LEFT ARM INIT
			l_trPos_init2g = util.mulvec(rpyR_b2g, vector.slice(currentLArm,1,3))
				l_trPos_init2g[3] = l_trPos_init2g[3] * 0.2 + stage_h * 0.8
			l_trPos_init = util.mulvec(util.transpose(rpyR_b2g),l_trPos_init2g)
				l_trPos_init[2] = l_trPos_init[2] + util.sign(l_trPos_init[2])*0.2
				l_trPos_init[3] = l_trPos_init[3] + 0.05

			trL_init = util.concat(l_trPos_init, util.ypr(rpyR_tr2b,1,vector.slice(currentLArm,4,6)))
			qLArm_init = vector.new(Body.get_inverse_larm(qLArm_current, trL_init))
	    		qLArm_init[7] = qLArm_init[7] + 90*DEG_TO_RAD

	    	-- RIGHT ARM
	    	r_trPos2g = r_shoulder2g
				r_trPos2g[3] = stage_h
			r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
				r_trPos[2] = r_trPos[2] + util.sign(r_trPos[2])*0.2 -- body y direction 
				r_trPos[3] = r_trPos[3] + 0.2
			trR = util.concat(r_trPos, util.ypr(rpyR_tr2b,-1,vector.slice(currentRArm,4,6)))
	    	qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
		    	qRArm1[6] = qRArm1[6] - util.sign(qRArm1[6])*5*DEG_TO_RAD
		    	qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD

			-- RIGHT ARM INIT
			r_trPos_init2g = util.mulvec(rpyR_b2g, vector.slice(currentRArm,1,3))
				r_trPos_init2g[3] = r_trPos_init2g[3] * 0.2 + stage_h * 0.8
			r_trPos_init = util.mulvec(util.transpose(rpyR_b2g),r_trPos_init2g)
				r_trPos_init[2] = r_trPos_init[2] + util.sign(r_trPos_init[2])*0.2
				r_trPos_init[3] = r_trPos_init[3] + 0.05
			
			trR_init = util.concat(r_trPos_init, util.ypr(rpyR_tr2b,-1,vector.slice(currentRArm,4,6)))
	   		qRArm_init = vector.new(Body.get_inverse_rarm(qRArm_current, trR_init))
	    		qRArm_init[7] = qRArm_init[7] - 90*DEG_TO_RAD	
	    	
			-- WAIST
	    	qWaist1 = qWaist_current 
	    	qWaist_init = qWaist1
	    	
			setting = 0 

		end

	    qLLeg1 = vector.new(Body.get_lleg_position())
	    	--qLLeg1[3] = qLLeg1[3] - util.sign(qLLeg1[3]) * (math.max(baseframe_h,stage_h) - l_knee_posg[3] + knee_offset) * 0.08
	    	--qLLeg1[4] = qLLeg1[4] - util.sign(qLLeg1[4]) * ((math.max(baseframe_h,stage_h) - l_knee_posg[3] + knee_offset) * 0.1 - ((l_knee_posg[3] - knee_offset) - (lleg_current2g[3] - footToeX))*0.08)--(math.max(baseframe_h,stage_h) - lleg_current2g[3] + footToeX)*0.02)
	    qRLeg1 = vector.new(Body.get_rleg_position())
	    	--qRLeg1[3] = qRLeg1[3] - util.sign(qRLeg1[3]) * (math.max(baseframe_h,stage_h) - r_knee_posg[3] + knee_offset) * 0.08
	    	--qRLeg1[4] = qRLeg1[4] - util.sign(qRLeg1[4]) * ((math.max(baseframe_h,stage_h) - r_knee_posg[3] + knee_offset) * 0.1 - ((r_knee_posg[3] - knee_offset) - (rleg_current2g[3] - footToeX))*0.08) --(math.max(baseframe_h,stage_h) - rleg_current2g[3] + footToeX)*0.02)
	    
	    qLLeg_init = qLLeg1
	    qRLeg_init = qRLeg1


	elseif stage == 3 then
		print("STAGE 3")
		-- Moving : larm, rarm
		-- Support : lknee, lfoot, rknee, rfoot
		-- Free : Torso

		if setting == 1 then -- Set parameters for the stage at first.
			--** STAGE PARAMETERS **--
			init = 0
			duration = 3
			mov = vector.new{1,1,0,0,0}
			stage_h = baseframe_h - 0.3

			ori_ctrl_larm = 0
			ori_ctrl_rarm = 0

			-- LEFT ARM
			l_trPos2g = util.mulvec(rpyR_b2g,vector.slice(currentLArm,1,3))
				l_trPos2g[3] = stage_h
			l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
      			l_trPos[2] = l_trPos[2] - util.sign(l_trPos[2])*0.05 
				l_trPos[3] = l_trPos[3] --+ 0.1 
      		-- RIGHT ARM
      		r_trPos2g = util.mulvec(rpyR_b2g,vector.slice(currentRArm,1,3))
      			r_trPos2g[3] = stage_h
		    r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
		    	r_trPos[2] = r_trPos[2] -util.sign(r_trPos[2])*0.05
				r_trPos[3] = r_trPos[3] --+ 0.1

			trL = util.concat(l_trPos, util.ypr(rpyR_tr2b,1,vector.slice(currentLArm,4,6)))
		    trR = util.concat(r_trPos, util.ypr(rpyR_tr2b,-1,vector.slice(currentRArm,4,6)))

		    qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
		    	qLArm1[6] = qLArm1[6] - util.sign(qLArm1[6])*5*DEG_TO_RAD
	    		qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD
		    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
		    	qRArm1[6] = qRArm1[6] - util.sign(qRArm1[6])*5*DEG_TO_RAD
	    		qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD

	    	initPitch = rpy[2]

	    	setting = 0
	    end

	    qLLeg1 = vector.new(Body.get_lleg_position())
	    	qLLeg1[3] = qLLeg1[3] - util.sign(qLLeg1[3]) * (math.max(baseframe_h,stage_h) - l_knee_posg[3] + knee_offset) * 0.06
	    	qLLeg1[4] = qLLeg1[4] - util.sign(qLLeg1[4]) * ((math.max(baseframe_h,stage_h) - l_knee_posg[3] + knee_offset) * 0.12 - ((l_knee_posg[3] - knee_offset) - (lleg_current2g[3] - footToeX)) *0.08)--(math.max(baseframe_h,stage_h) - lleg_current2g[3] + footToeX)*0.02)
	    qRLeg1 = vector.new(Body.get_rleg_position())
	    	qRLeg1[3] = qRLeg1[3] - util.sign(qRLeg1[3]) * (math.max(baseframe_h,stage_h) - r_knee_posg[3] + knee_offset) * 0.08
	    	qRLeg1[4] = qRLeg1[4] - util.sign(qRLeg1[4]) * ((math.max(baseframe_h,stage_h) - r_knee_posg[3] + knee_offset) * 0.15- ((r_knee_posg[3] - knee_offset) - (rleg_current2g[3] - footToeX)) *0.08) --(math.max(baseframe_h,stage_h) - rleg_current2g[3] + footToeX)*0.02)
	   	
	   	qWaist1 = qWaist_current
			qWaist1[2] = qWaist1[2] + (rpy[2] - initPitch) 


	elseif stage == 4 then
		print("STAGE 4")
		--4) Pull larm & move CM backward(WaistPitch, Legs[3], Legs[4])
  		-- moving : larm, WaistPitch, rleg, lleg
  		-- support: knees, foot
  		-- free : rarm, WaistRoll

  		if setting == 1 then

  			init = 1
			duration = 2
			mov = vector.new{1,1,1,1,0}
			stage_h = baseframe_h	
			fold_ang = 25 * DEG_TO_RAD

			ori_ctrl_larm = 0
			ori_ctrl_rarm = 0

			-- LEFT ARM INIT
			l_trPos_init2g = util.mulvec(rpyR_b2g, vector.slice(currentLArm,1,3))
				l_trPos_init2g[3] = l_trPos_init2g[3] + 0.2
			l_trPos_init = util.mulvec(util.transpose(rpyR_b2g),l_trPos_init2g)
				l_trPos_init[3] = l_trPos_init[3] + 0.1

			trL_init = util.concat(l_trPos_init, util.ypr(rpyR_tr2b,1,vector.slice(currentLArm,4,6)))
			qLArm_init = vector.new(Body.get_inverse_larm(qLArm_current, trL_init))
	    		qLArm_init[7] = qLArm_init[7] + 90*DEG_TO_RAD

	    	-- LEFT ARM
			l_trPos2g = util.mulvec(rpyR_b2g,vector.slice(currentLArm,1,3))
				l_trPos2g[3] = stage_h
			l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
				l_trPos[3] = l_trPos[3] + 0.15 

			trL = util.concat(l_trPos, util.ypr(rpyR_tr2b,1,vector.slice(currentLArm,4,6)))
		    qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
		    	--qLArm1[6] = qLArm1[6] - util.sign(qLArm1[6])*5*DEG_TO_RAD
	    		qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD

			-- LEGS

			qLLeg1 = vector.new(Body.get_lleg_position())
	    		qLLeg1[3] = qLLeg1[3] + util.sign(qLLeg1[3]) * fold_ang*0.8
	    		qLLeg1[4] = qLLeg1[4] + util.sign(qLLeg1[4]) * fold_ang*0.8
	    		qLLeg1[5] = qLLeg1[5] + util.sign(qLLeg1[5]) * fold_ang/4
	    	qRLeg1 = vector.new(Body.get_rleg_position())
	    		qRLeg1[3] = qRLeg1[3] + util.sign(qRLeg1[3]) * fold_ang
	    		qRLeg1[4] = qRLeg1[4] + util.sign(qRLeg1[4]) * fold_ang
	    		qRLeg1[5] = qRLeg1[5] + util.sign(qRLeg1[5]) * fold_ang -- ready
	    
	    	qLLeg_init = vector.new(Body.get_lleg_position())
	    		qLLeg_init[3] = qLLeg_init[3] + util.sign(qLLeg_init[3]) * fold_ang/2*0.8
	    		qLLeg_init[4] = qLLeg_init[4] + util.sign(qLLeg_init[4]) * fold_ang/2*0.8
	    		qLLeg_init[5] = qLLeg_init[5] + util.sign(qLLeg_init[5]) * fold_ang/8
	    	qRLeg_init = vector.new(Body.get_rleg_position())
	    		qRLeg_init[3] = qRLeg_init[3] + util.sign(qRLeg_init[3]) * fold_ang/2
	    		qRLeg_init[4] = qRLeg_init[4] + util.sign(qRLeg_init[4]) * fold_ang/2
	    		qRLeg_init[5] = qRLeg_init[5] + util.sign(qRLeg_init[5]) * fold_ang/2 -- ready
  			
  			initPitch = rpy[2] 

			qRArm1 = qRArm_current
	    	qRArm_init = qRArm_current
	    		--qRArm_init[4] = qRArm_init[4] + util.sign(qRArm_init[4])* 20 * DEG_TO_RAD


  			setting = 0
  		end

  		-- Real Time update at this stage.
		
	    qWaist1 = qWaist_current
			qWaist1[2] = qWaist1[2] + (rpy[2] - initPitch) 
	    qWaist_init = qWaist1
	
	elseif stage == 5 then
		print("STAGE 5")
		--4) Pull larm & move CM backward(WaistPitch, Legs[3], Legs[4])
  		-- moving : larm, WaistPitch, rleg, lleg
  		-- support: knees, foot
  		-- free : rarm, WaistRoll

  		if setting == 1 then

  			init = 1
			duration = 2
			mov = vector.new{1,1,1,1,0}
			stage_h = baseframe_h	
			fold_ang = 25 * DEG_TO_RAD

			ori_ctrl_larm = 0
			ori_ctrl_rarm = 0

			-- RIGHT ARM INIT
			r_trPos_init2g = util.mulvec(rpyR_b2g, vector.slice(currentRArm,1,3))
				r_trPos_init2g[3] = r_trPos_init2g[3] + 0.2
			r_trPos_init = util.mulvec(util.transpose(rpyR_b2g),r_trPos_init2g)
				r_trPos_init[3] = r_trPos_init[3] + 0.1

			trR_init = util.concat(l_trPos_init, util.ypr(rpyR_tr2b,1,vector.slice(currentRArm,4,6)))
			qRArm_init = vector.new(Body.get_inverse_rarm(qRArm_current, trR_init))
	    		qRArm_init[7] = qRArm_init[7] - 90*DEG_TO_RAD

	    	-- RIGHT ARM
			r_trPos2g = util.mulvec(rpyR_b2g,vector.slice(currentRArm,1,3))
				r_trPos2g[3] = stage_h
			r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
				r_trPos[3] = r_trPos[3] + 0.15 

			trR = util.concat(l_trPos, util.ypr(rpyR_tr2b,1,vector.slice(currentRArm,4,6)))
		    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
		    	--qLArm1[6] = qLArm1[6] - util.sign(qLArm1[6])*5*DEG_TO_RAD
	    		qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD

			-- LEGS

			qLLeg1 = vector.new(Body.get_lleg_position())
	    		qLLeg1[3] = qLLeg1[3] + util.sign(qLLeg1[3]) * fold_ang*0.8
	    		qLLeg1[4] = qLLeg1[4] + util.sign(qLLeg1[4]) * fold_ang*0.8
	    		qLLeg1[5] = qLLeg1[5] + util.sign(qLLeg1[5]) * fold_ang/4
	    	qRLeg1 = vector.new(Body.get_rleg_position())
	    		qRLeg1[3] = qRLeg1[3] + util.sign(qRLeg1[3]) * fold_ang
	    		qRLeg1[4] = qRLeg1[4] + util.sign(qRLeg1[4]) * fold_ang
	    		qRLeg1[5] = qRLeg1[5] + util.sign(qRLeg1[5]) * fold_ang*1.2 -- ready
	    
	    	qLLeg_init = vector.new(Body.get_lleg_position())
	    		qLLeg_init[3] = qLLeg_init[3] + util.sign(qLLeg_init[3]) * fold_ang/2*0.8
	    		qLLeg_init[4] = qLLeg_init[4] + util.sign(qLLeg_init[4]) * fold_ang/2*0.8
	    		qLLeg_init[5] = qLLeg_init[5] + util.sign(qLLeg_init[5]) * fold_ang/8
	    	qRLeg_init = vector.new(Body.get_rleg_position())
	    		qRLeg_init[3] = qRLeg_init[3] + util.sign(qRLeg_init[3]) * fold_ang/2
	    		qRLeg_init[4] = qRLeg_init[4] + util.sign(qRLeg_init[4]) * fold_ang/2
	    		qRLeg_init[5] = qRLeg_init[5] + util.sign(qRLeg_init[5]) * fold_ang/2 -- ready
  			
  			initPitch = rpy[2] 

			qLArm1 = qLArm_current
	    	qLArm_init = qLArm_current
	    		--qRArm_init[4] = qRArm_init[4] + util.sign(qRArm_init[4])* 20 * DEG_TO_RAD


  			setting = 0
  		end

  		-- Real Time update at this stage.
		
	    qWaist1 = qWaist_current
			qWaist1[2] = qWaist1[2] + (rpy[2] - initPitch) 
	    qWaist_init = qWaist1

	elseif stage == 6 then
		print("STAGE 6")
		if setting == 1 then -- Set parameters for the stage at first.
			--** STAGE PARAMETERS **--
			init = 0
			duration = 2
			mov = vector.new{1,1,0,0,1}
			stage_h = baseframe_h 

			ori_ctrl_larm = 0
			ori_ctrl_rarm = 0

			-- LEFT ARM
			l_trPos2g = util.mulvec(rpyR_b2g,vector.slice(currentLArm,1,3))
			l_trPos2g[3] = stage_h - 0.01
			l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
      			l_trPos[2] = l_trPos[2] - util.sign(l_trPos[2])*0 -- body y direction
				--l_trPos[3] = l_trPos[3] + 0.1 
      		-- RIGHT ARM
      		r_trPos2g = util.mulvec(rpyR_b2g,vector.slice(currentRArm,1,3))
      		r_trPos2g[3] = stage_h - 0.11
		    r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
		    	r_trPos[2] = r_trPos[2] - util.sign(r_trPos[2])*0.02 -- body y direction 
				--r_trPos[3] = r_trPos[3] + 0.1

			trL = util.concat(l_trPos, util.ypr(rpyR_tr2b,1,vector.slice(currentLArm,4,6)))
		    trR = util.concat(r_trPos, util.ypr(rpyR_tr2b,-1,vector.slice(currentRArm,4,6)))

		    qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
		    	--qLArm1[6] = qLArm1[6] - util.sign(qLArm1[6])*5*DEG_TO_RAD
	    		qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD
		    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
		    	--qRArm1[6] = qRArm1[6] - util.sign(qRArm1[6])*5*DEG_TO_RAD
	    		qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD

	    	initPitch = rpy[2]
	    	qWaist1 = qWaist_current
	    		qWaist1[1] = 15 * DEG_TO_RAD
				qWaist1[2] = qWaist1[2] + (rpy[2] - initPitch) 

	    	setting = 0
	    end

	    qLLeg1 = vector.new(Body.get_lleg_position())
	    qRLeg1 = vector.new(Body.get_rleg_position())
	   	
	   	print(vector.new(util.mulvec(rpyR_b2g, vector.slice(currentLArm,1,3))))
	   	print(vector.new(util.mulvec(rpyR_b2g, vector.slice(currentRArm,1,3))))


	elseif stage == 7 then
		print("STAGE 7")
		if setting == 1 then
			init = 0
    		duration = 5
    		mov = vector.new{1,1,0,1,0}
			stage_h = baseframe_h - 0.01

			ori_ctrl_larm = 0
			ori_ctrl_rarm = 0

			initYaw = rpy[3]
			initPitch = rpy[2] 

    --------Right Leg

		    rleg_trPos2g = {}
		      rleg_trPos2g[1] = r_hip_posg[1]
		      rleg_trPos2g[2] = r_hip_posg[2]
		      rleg_trPos2g[3] = stage_h

		    rleg_trPos = util.mulvec(util.transpose(rpyR_b2g), rleg_trPos2g)
		    rleg_trPos[3] = rleg_trPos[3] - 0.02 -- negative farther than hip joint (vertical body)  util.sign(rleg_trPos[3])*
		    rleg_trPos[2] = rleg_trPos[2] - util.sign(rleg_trPos[2])*0.1

		    trRLeg = {}
		    trRLeg[1] = rleg_trPos[1]
		    trRLeg[2] = rleg_trPos[2]
		    trRLeg[3] = rleg_trPos[3]
		    trRLeg[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
		    trRLeg[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) )--pitch
		    trRLeg[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll

			setting = 0
		end
		
		qLLeg1 = vector.new(Body.get_lleg_position())
	    	--qLLeg1[3] = qLLeg1[3] + util.sign(qLLeg1[3]) * (baseframe_h - l_knee_posg[3] + knee_offset) * 0.08
	    	qLLeg1[4] = qLLeg1[4] + util.sign(qLLeg1[4]) * ((baseframe_h - l_knee_posg[3] + knee_offset) * 0.02 - ((l_knee_posg[3] - knee_offset) - (lleg_current2g[3] - footToeX)) *0.01)--(math.max(baseframe_h,stage_h) - lleg_current2g[3] + footToeX)*0.02)

	    qRLeg1 = vector.new(Body.get_inverse_rleg(trRLeg))
	    	qRLeg1[6] = qRLeg1[6] + (initYaw - rpy[3]) - qRLeg_current[1] + 5*DEG_TO_RAD
    	qWaist1 = qWaist_current
			qWaist1[2] = qWaist1[2] + (rpy[2] - initPitch) 
      	
	    --qLLeg_init = qLLeg1
    	--qRLeg_init = vector.new(Body.get_inverse_rleg(trRLeg_init))
   		--qWaist_init = qWaist1
   		qLArm1 = qLArm_current
   		qRArm1 = qRArm_current

	elseif stage == 8 then

		if setting == 1 then
			init = 1
    		duration = 6
    		mov = vector.new{1,1,1,1,1}
			stage_h = baseframe_h

			ori_ctrl_larm = 0
			ori_ctrl_rarm = 0

			initYaw = rpy[3]
			initPitch = rpy[2] 

    --------LEFT Leg

		    lleg_trPos2g = {}
		      lleg_trPos2g[1] = l_hip_posg[1]
		      lleg_trPos2g[2] = l_hip_posg[2]
		      lleg_trPos2g[3] = stage_h

		    lleg_trPos = util.mulvec(util.transpose(rpyR_b2g), lleg_trPos2g)
		    lleg_trPos[3] = lleg_trPos[3] -- 0.02 -- negative farther than hip joint (vertical body)  util.sign(rleg_trPos[3])*
		    lleg_trPos[2] = lleg_trPos[2] -- util.sign(lleg_trPos[2])*0.1

		    trLLeg = {}
		    trLLeg[1] = lleg_trPos[1]
		    trLLeg[2] = lleg_trPos[2]
		    trLLeg[3] = lleg_trPos[3]
		    trLLeg[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
		    trLLeg[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) )--pitch
		    trLLeg[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll

		    qLLeg1 = vector.new(Body.get_inverse_lleg(trLLeg))
		    	qLLeg1[5] = qLLeg1[5] - 10*DEG_TO_RAD
		    	qLLeg1[6] = 0
    		qWaist1 = qWaist_current
    			qWaist1[1] = 0
				qWaist1[2] = 20*DEG_TO_RAD --qWaist1[2] + (rpy[2] - initPitch) 
			qWaist_init = qWaist1

			qLLeg_init = qLLeg_current
			qRLeg_init = qRLeg_current
				qRLeg_init[1] = 0
				qRLeg_init[6] = 0
				qRLeg_init[5] = qRLeg_init[5] - 10*DEG_TO_RAD
			qRLeg1 = qRLeg_init

			-- LEFT ARM
			l_trPos2g = util.mulvec(rpyR_b2g,vector.slice(currentLArm,1,3))
			l_trPos2g[3] = stage_h - 0.1
			l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
      			l_trPos[2] = l_trPos[2] - util.sign(l_trPos[2])*0.1 -- body y direction
				--l_trPos[3] = l_trPos[3] + 0.1 
      		-- RIGHT ARM
      		r_trPos2g = util.mulvec(rpyR_b2g,vector.slice(currentRArm,1,3))
      		r_trPos2g[3] = stage_h 
		    r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
		    	r_trPos[2] = r_trPos[2] + util.sign(r_trPos[2])*0.1 -- body y direction 
				--r_trPos[3] = r_trPos[3] + 0.1
			setting = 0

		end
	
   		trL = util.concat(l_trPos, util.ypr(rpyR_tr2b,1,vector.slice(currentLArm,4,6)))
		trR = util.concat(r_trPos, util.ypr(rpyR_tr2b,-1,vector.slice(currentRArm,4,6)))

		qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
		    	--qLArm1[6] = qLArm1[6] - util.sign(qLArm1[6])*5*DEG_TO_RAD
	    	qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD
		qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
			qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD	
		    	--qRArm1[6] = qRArm1[6] - util.sign(qRArm1[6])*5*DEG_TO_RAD

		qLArm_init = qLArm1
		qRArm_init = qRArm1

	elseif stage == 9 then

		if setting == 1 then
			init = 0
    		duration = 2
    		mov = vector.new{1,1,1,1,1}
			stage_h = baseframe_h

			ori_ctrl_larm = 0
			ori_ctrl_rarm = 0

			qRLeg1 = qRLeg_current
			qLLeg1 = qLLeg_current
				qLLeg1[3] = qRLeg1[3]
				qLLeg1[4] = qRLeg1[4]

			setting = 0

		end

		qLArm1 = qLArm_current
		qRArm1 = qRArm_current
				qRArm1[1] = 2.4
				qRArm1[4] = -1.7

		qLArm_init = qLArm1
		qRArm_init = qRArm1

   	elseif stage == 10 then

		if setting == 1 then
			init = 1
    		duration = 8
    		mov = vector.new{1,1,1,1,1}
			stage_h = baseframe_h

			ori_ctrl_larm = 0
			ori_ctrl_rarm = 0
				
			qWaist1 = qWaist_current

			qLArm_init = qLArm_current
				qLArm_init[1] = 2.4
				qLArm_init[4] = -1.7
			qLArm1 = qLArm_init
			qRArm_init = qRArm_current
			qRArm1 = qRArm_current
				qRArm1[1] = 2.4
				qRArm1[4] = -1.7
			

			qLLeg1 = qLLeg_current
				qLLeg1[3] = util.sign(qLLeg1[3])*30*DEG_TO_RAD
				qLLeg1[5] = 0
			qRLeg1 = qRLeg_current
				qRLeg1[3] = util.sign(qRLeg1[3])*30*DEG_TO_RAD
				qRLeg1[5] = 0
			qLLeg_init = qLLeg_current
				qLLeg_init[3] = util.sign(qLLeg_init[3])*30*DEG_TO_RAD
				qLLeg_init[5] = 0
			qRLeg_init = qRLeg_current
				qRLeg_init[3] = util.sign(qRLeg_init[3])*30*DEG_TO_RAD
				qRLeg_init[5] = 0

			setting = 0
		end



    end
	---------------------------------------------------------------------------------------------------------------------------------------- 
	--[[ Orienatation Feedback Realtime Control ]]------------------------------------------------------------------------------------------
	
	if ori_ctrl_larm == 1 then
	    local ori_larm = {}
	    ori_larm[1] = currentLArm[6] --roll
	    ori_larm[2] = currentLArm[5] --pitch
	    ori_larm[3] = currentLArm[4] --yaw

	    local R_c2b_l = {} -- left end-effector RPY matrix
	    R_c2b_l[1] = {}
	      R_c2b_l[1][1] = cos(ori_larm[1])*cos(ori_larm[2])
	      R_c2b_l[1][2] = cos(ori_larm[1])*sin(ori_larm[2])*sin(ori_larm[3]) - sin(ori_larm[1])*cos(ori_larm[3])
	      R_c2b_l[1][3] = cos(ori_larm[1])*sin(ori_larm[2])*cos(ori_larm[3]) + sin(ori_larm[1])*sin(ori_larm[3])
	    R_c2b_l[2] = {}
	      R_c2b_l[2][1] = sin(ori_larm[1])*cos(ori_larm[2])
	      R_c2b_l[2][2] = sin(ori_larm[1])*sin(ori_larm[2])*sin(ori_larm[3]) + cos(ori_larm[1])*cos(ori_larm[3])
	      R_c2b_l[2][3] = sin(ori_larm[1])*sin(ori_larm[2])*cos(ori_larm[3]) - cos(ori_larm[1])*sin(ori_larm[3]) 
	    R_c2b_l[3] = {}
	      R_c2b_l[3][1] = -sin(ori_larm[2])
	      R_c2b_l[3][2] = cos(ori_larm[2])*sin(ori_larm[3])
	      R_c2b_l[3][3] = cos(ori_larm[2])*cos(ori_larm[3])

	    local l_ori2g = util.ypr(R_c2b_l,1,vector.slice(currentLArm,4,6))

	    -- PITCH
	    qLArm1[6] = qLArm1[6] + (rpy_target[2] - l_ori2g[2])
	    -- ROLL
	    qLArm1[7] = qLArm1[7] + (rpy_target[1] - l_ori2g[3])

	end

	if ori_ctrl_rarm == 1 then
		local ori_rarm = {}
		ori_rarm[1] = currentLArm[6] --roll
		ori_rarm[2] = currentLArm[5] --pitch
		ori_rarm[3] = currentLArm[4] --yaw

		local R_c2b_r = {}
		R_c2b_r[1] = {}
		  R_c2b_r[1][1] = cos(ori_rarm[1])*cos(ori_rarm[2])
		  R_c2b_r[1][2] = cos(ori_rarm[1])*sin(ori_rarm[2])*sin(ori_rarm[3]) - sin(ori_rarm[1])*cos(ori_rarm[3])
		  R_c2b_r[1][3] = cos(ori_rarm[1])*sin(ori_rarm[2])*cos(ori_rarm[3]) + sin(ori_rarm[1])*sin(ori_rarm[3])
		R_c2b_r[2] = {}
		  R_c2b_r[2][1] = sin(ori_rarm[1])*cos(ori_rarm[2])
		  R_c2b_r[2][2] = sin(ori_rarm[1])*sin(ori_rarm[2])*sin(ori_rarm[3]) + cos(ori_rarm[1])*cos(ori_rarm[3])
		  R_c2b_r[2][3] = sin(ori_rarm[1])*sin(ori_rarm[2])*cos(ori_rarm[3]) - cos(ori_rarm[1])*sin(ori_rarm[3]) 
		R_c2b_r[3] = {}
		  R_c2b_r[3][1] = -sin(ori_rarm[2])
		  R_c2b_r[3][2] = cos(ori_rarm[2])*sin(ori_rarm[3])
		  R_c2b_r[3][3] = cos(ori_rarm[2])*cos(ori_rarm[3])

	    local r_ori2g = util.ypr(R_c2b_r,-1,vector.slice(currentRArm,4,6))

		-- PITCH
		qRArm1[6] = qRArm1[6] + (rpy_target[2] - r_ori2g[2])
		-- ROLL
		qRArm1[7] = qRArm1[7] + (rpy_target[1] - r_ori2g[3])

	end

	----------------------------------------------------------------------------------------------------------------------------------------
	--[[ Set the Joint Angles ]]------------------------------------------------------------------------------------------------------------
	print("Duration",duration)
	print("time_diff", t_diff)
	if stage>0 and stage < (stagemax+1) then    
	    local ph = math.max(0,math.min(1, t_diff/duration))
	    if init == 1 then -- When there exist an initial state
	      ph = ph*2
	      print(vector.new(mov))
	      if t_diff < 0.5*duration then
	        Body.set_larm_command_position( mov[1] * ((1-ph)*qLArm0 + ph * qLArm_init)  +  (1-mov[1]) * qLArm_init )    
	        Body.set_rarm_command_position( mov[2] * ((1-ph)*qRArm0 + ph * qRArm_init)  +  (1-mov[2]) * qRArm_init )
	        Body.set_lleg_command_position( mov[3] * ((1-ph)*qLLeg0 + ph * qLLeg_init)  +  (1-mov[3]) * qLLeg_init )
	        Body.set_rleg_command_position( mov[4] * ((1-ph)*qRLeg0 + ph * qRLeg_init)  +  (1-mov[4]) * qRLeg_init )
	        Body.set_waist_command_position(mov[5] * ((1-ph)*qWaist0 + ph * qWaist_init)+  (1-mov[5]) * qWaist_init)
	      else
	        ph = ph-1
	        Body.set_larm_command_position( mov[1] * ((1-ph)*qLArm_init + ph * qLArm1)  +  (1-mov[1]) * qLArm1 )    
	        Body.set_rarm_command_position( mov[2] * ((1-ph)*qRArm_init + ph * qRArm1)  +  (1-mov[2]) * qRArm1 )
	        Body.set_lleg_command_position( mov[3] * ((1-ph)*qLLeg_init + ph * qLLeg1)  +  (1-mov[3]) * qLLeg1 )
	        Body.set_rleg_command_position( mov[4] * ((1-ph)*qRLeg_init + ph * qRLeg1)  +  (1-mov[4]) * qRLeg1 )
	        Body.set_waist_command_position(mov[5] * ((1-ph)*qWaist_init + ph * qWaist1)+  (1-mov[5]) * qWaist1)
	      end
	    else
	      Body.set_larm_command_position( mov[1] * ((1-ph)*qLArm0 + ph * qLArm1)  +  (1-mov[1]) * qLArm1 )    
	   	  Body.set_rarm_command_position( mov[2] * ((1-ph)*qRArm0 + ph * qRArm1)  +  (1-mov[2]) * qRArm1 )
	      Body.set_lleg_command_position( mov[3] * ((1-ph)*qLLeg0 + ph * qLLeg1)  +  (1-mov[3]) * qLLeg1 )
	      Body.set_rleg_command_position( mov[4] * ((1-ph)*qRLeg0 + ph * qRLeg1)  +  (1-mov[4]) * qRLeg1 )
	      Body.set_waist_command_position(mov[5] * ((1-ph)*qWaist0 + ph * qWaist1)+  (1-mov[5]) * qWaist1)
	    end

	    if t_diff>duration and hcm.get_state_proceed()==1 then
		  	if stop == 1 then
		  		hcm.set_state_proceed(0)
		  	end
		  	-- GO TO NEXT STAGE
		  	setting = 1
		  	stage = stage + 1
		  	t_start = t

		  	qLArm0 = qLArm1
	      	qRArm0 = qRArm1
	     	qLLeg0 = qLLeg1
	      	qRLeg0 = qRLeg1
	      	qWaist0 = qWaist1
	      
	      	unix.usleep(500000)
	    end
	else
		hcm.set_state_proceed(0)
	    print("DONE")
	    --f1:close()
	    return "done"  
	end


end

function state.exit()
  print(state._NAME..' Exit')
  -- TODO: Store things in shared memory?
  hcm.set_motion_headangle({0,0*math.pi/180})
end

return state
























 