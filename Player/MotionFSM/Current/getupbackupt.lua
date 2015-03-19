elseif stage == 6 then
		if setting == 1 then
			init = 1
    		duration = 5
    		mov = vector.new{1,0,0,1,0}
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


 			rleg_trPos_init2g = {}
		      rleg_trPos_init2g[1] = rleg_current2g[1] 
		      rleg_trPos_init2g[2] = rleg_current2g[2]
		      rleg_trPos_init2g[3] = rleg_current2g[3]

			rleg_trPos_init = util.mulvec(util.transpose(rpyR_b2g),rleg_trPos_init2g)
		    --rleg_trPos_init[3] = math.min(rleg_trPos[3],rleg_trPos_init[3] + 0.05) -- closer than current right foot joint (vertical body) util.sign(rleg_trPos_init[3])*
		    --rleg_trPos_init[2] = rleg_trPos_init[2] - util.sign(rleg_trPos_init[2])*0.05

		    trRLeg_init = {}
		    trRLeg_init[1] = rleg_trPos_init[1]
		    trRLeg_init[2] = rleg_trPos_init[2]
		    trRLeg_init[3] = rleg_trPos_init[3]
		    trRLeg_init[4] = currentRLeg[4]--atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
		    trRLeg_init[5] = currentRLeg[5]--*0.5 + atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2)) * 0.5--pitch
		    trRLeg_init[6] = currentRLeg[6]--atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1]) --roll : but about x-axis

		    -- LEFT ARM
			l_trPos2g = l_shoulder2g 
				l_trPos2g[3] = stage_h + 0.05

			-- LEFT ARM INIT
			l_trPos_init2g = util.mulvec(rpyR_b2g, vector.slice(currentLArm,1,3))
				l_trPos_init2g[3] = stage_h + 0.1
			

	    	-- RIGHT ARM
	    	--r_trPos2g = r_shoulder2g
			--	r_trPos2g[3] = stage_h - 0.1
			

			-- RIGHT ARM INIT
			r_trPos_init2g = util.mulvec(rpyR_b2g, vector.slice(currentRArm,1,3))
				r_trPos_init2g[3] = stage_h - 0.15

			setting = 0
		end
		l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
		trL = util.concat(l_trPos, util.ypr(rpyR_tr2b,1,vector.slice(currentLArm,4,6)))
		qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
	    	--qLArm1[6] = qLArm1[6] - util.sign(qLArm1[6])*5*DEG_TO_RAD
	    	qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD +(initYaw - rpy[3])

	    l_trPos_init = util.mulvec(util.transpose(rpyR_b2g),l_trPos_init2g)
		trL_init = util.concat(l_trPos_init, util.ypr(rpyR_tr2b,1,vector.slice(currentLArm,4,6)))
		qLArm_init = vector.new(Body.get_inverse_larm(qLArm_current, trL_init))
    		qLArm_init[7] = qLArm_init[7] + 90*DEG_TO_RAD + (initYaw - rpy[3])

		r_knee[3] = r_knee[3] + tibiaLength*cos(qRLeg_current[4]+qRLeg_current[3])

    	r_trPos2g = vector.slice(currentRArm,1,3)
    		r_trPos2g[3] = baseframe_h
    	r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)	
    		r_trPos[3] = math.max(r_trPos[3], r_knee[3])
    		r_trPos[2] = -0.2--r_trPos[2] - util.sign(r_trPos[2]) * 0.05
		trR = util.concat(r_trPos, util.ypr(rpyR_tr2b,-1,vector.slice(currentRArm,4,6)))
    	qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
	    	--[6] = qRArm1[6] - util.sign(qRArm1[6])*5*DEG_TO_RAD
	    	qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD - (initYaw - rpy[3])
	    --qRArm_init = qRArm1

		r_trPos_init = util.mulvec(util.transpose(rpyR_b2g),r_trPos_init2g)
		trR_init = util.concat(r_trPos_init, util.ypr(rpyR_tr2b,-1,vector.slice(currentRArm,4,6)))
   		qRArm_init = vector.new(Body.get_inverse_rarm(qRArm_current, trR_init))
    		qRArm_init[7] = qRArm_init[7] - 90*DEG_TO_RAD - (initYaw - rpy[3])

		qLLeg1 = vector.new(Body.get_lleg_position())
	    	qLLeg1[3] = qLLeg1[3] - util.sign(qLLeg1[3]) * (baseframe_h - l_knee_posg[3] + knee_offset) * 0.08
	    	qLLeg1[4] = qLLeg1[4] - util.sign(qLLeg1[4]) * ((baseframe_h - l_knee_posg[3] + knee_offset) * 0.12 - ((l_knee_posg[3] - knee_offset) - (lleg_current2g[3] - footToeX)) *0.09)--(math.max(baseframe_h,stage_h) - lleg_current2g[3] + footToeX)*0.02)

	    qRLeg1 = vector.new(Body.get_inverse_rleg(trRLeg))
	    	qRLeg1[6] = qRLeg1[6] + (initYaw - rpy[3])
    	qWaist1 = qWaist_current
			qWaist1[2] = qWaist1[2] + (rpy[2] - initPitch) 
      	
	    qLLeg_init = qLLeg1
    	qRLeg_init = vector.new(Body.get_inverse_rleg(trRLeg_init))
   		qWaist_init = qWaist1