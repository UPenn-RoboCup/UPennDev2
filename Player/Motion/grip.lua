-- Stabilized grip motion state
-- by SJ, edited by Steve

module(..., package.seeall);

require('Body')
require('keyframe')
require('walk')
require('vector')
require('Config')
require 'util'

-- These should be Config variables...
footX = Config.walk.footX;
footY = Config.walk.footY;
supportX = Config.walk.supportX;

bodyTilt = Config.walk.bodyTilt;
bodyTilt1=40*math.pi/180;
--bodyTilt1=20*math.pi/180; --With long hand
bodyTilt1 = Config.walk.bodyTilt;

bodyHeight = Config.walk.bodyHeight;
bodyHeight0 = bodyHeight;
bodyHeight1 = 0.21;
bodyHeight2 = 0.22;

--bodyShift1 = 0.035;
bodyShift=0;
--bodyShift0 = 0;
bodyShift0 = 0.035;
--bodyShift1 = 0.015; --with long hand
bodyShift1 = 0.035; --with long hand
--bodyShift2 = -0.020;
bodyShift2 = 0.035;
--bodyShift3 = -0.010;
bodyShift3 = 0.015;

-- Times
t_throw = {1.0,2.0,3.0};
--t_grab  = {1.0,1.5,2.0,3.0,4.0};
t_grab  = {.75,1.5,3.0,5.0,7.0};

qRArm= vector.zeros(3);
-- Starting Arm pose
qLArm0 = Config.walk.qLArm;
qRArm0 = Config.walk.qRArm;

--Pickup
qLArm1 = math.pi/180*vector.new({90, -50,0});	
qRArm1 = math.pi/180*vector.new({90,-5,0});	

--Grasp
qLArm2 = math.pi/180*vector.new({70, -50,0});	
qRArm2 = math.pi/180*vector.new({70, 5,0});	

--Windup
qLArm3 = math.pi/180*vector.new({-90,-5,-120});	
qRArm3 = math.pi/180*vector.new({-90,5,-120});	

--Throw
qLArm4 = math.pi/180*vector.new({40,20,-0});	
qRArm4 = math.pi/180*vector.new({40,-20,-0});	

qLArm4 = math.pi/180*vector.new({20,20,-0});	
qRArm4 = math.pi/180*vector.new({20,-20,-0});	

qGrip0 = -10*math.pi/180;
qGrip1 = 45*math.pi/180;
qGrip = 0;

-- Shifting and compensation parameters
ankleShift=vector.new({0, 0});
kneeShift=0;
hipShift=vector.new({0,0});

ankleImuParamX=Config.kick.ankleImuParamX;
ankleImuParamY=Config.kick.ankleImuParamY;
kneeImuParamX=Config.kick.kneeImuParamX;
hipImuParamY=Config.kick.hipImuParamY;
armImuParamX=Config.kick.armImuParamX;
armImuParamY=Config.kick.armImuParamX;

qLHipRollCompensation=0;
qRHipRollCompensation=0;
qLHipPitchCompensation=0;
qRHipPitchCompensation=0;

pTorso = vector.new({0, 0, bodyHeight, 0,bodyTilt,0});
pLLeg=vector.zeros(6);
pRLeg=vector.zeros(6);

-- Pickup or throw
throw=0;
active=false;

--[[
walk.starting_foot=1; --after left kick, start walking with left foot
walk.starting_foot=0; 
--]]

function entry()
	print("Motion SM:".._NAME.." entry");
	walk.stop();
	started = false;
	active = true;
	-- disable joint encoder reading
	Body.set_syncread_enable(0);

	uLeft   = vector.new({-supportX, footY, 0});
	uRight  = vector.new({-supportX, -footY, 0});
	uLeft1  = vector.new({-supportX, footY, 0});
	uRight1 = vector.new({-supportX, -footY, 0});

	uBody  = vector.new({0,0,0});
	uBody0 = vector.new({0,0,0});
	uBody1 = vector.new({0,0,0});

	zLeft,  zRight  = 0,0;
	zLeft1, zRight1 = 0,0;
	aLeft,  aRight  = 0,0;
	aLeft1, aRight1 = 0,0;
end

function update()
	if (not started and walk.active) then
		walk.update();
		return;
	elseif not started then
		started=true;
		Body.set_head_hardness(.5);

		if throw==0 then
			--[[
			Body.set_larm_hardness({0.5,0.3,0.5});
			Body.set_rarm_hardness({0.5,0.3,0.5});
			--]]
			Body.set_larm_hardness({0.5,1,0.5});
			Body.set_rarm_hardness({0.5,1,0.5});
		else
			Body.set_larm_hardness({1,0.3,1});
			Body.set_rarm_hardness({1,0.3,1});
		end
		Body.set_lleg_hardness(1);
		Body.set_rleg_hardness(1);
		Body.set_aux_hardness(.5);
		t0 = Body.get_time();
	end


	local t=Body.get_time();
	t=t-t0;
	if throw==0 then
		local t_pickup = t_grab;
		if t<t_pickup[1] then 
			--Open grip and extend hand, lower body
			ph = t/t_pickup[1];
			qLArm = ph*qLArm1 + (1-ph)*qLArm0;
			qRArm = ph*qRArm1 + (1-ph)*qRArm0;
			qGrip = ph*qGrip1 + (1-ph)*qGrip0;
			bodyHeight = ph*bodyHeight1 + (1-ph)*bodyHeight0;
			bodyShift=bodyShift0*(1-ph)+ bodyShift1*ph;
		elseif t<t_pickup[2] then
			--bend front
			ph=(t-t_pickup[1])/(t_pickup[2]-t_pickup[1]);
			bodyTilt = ph* bodyTilt1 + (1-ph)*Config.walk.bodyTilt;
		elseif t<t_pickup[3] then
			--Grasp
			ph=(t-t_pickup[2])/(t_pickup[3]-t_pickup[2]);
			qLArm= ph * qLArm2 + (1-ph)*qLArm1;
			qRArm= ph * qRArm2 + (1-ph)*qRArm1;
			qGrip=ph*qGrip0 + (1-ph)*qGrip1;
		elseif t<t_pickup[4] then
			--repose
			ph=(t-t_pickup[3])/(t_pickup[4]-t_pickup[3]);
			bodyTilt = ph* Config.walk.bodyTilt+(1-ph)*bodyTilt1;
			bodyHeight = ph*bodyHeight2 + (1-ph)*bodyHeight1;
			bodyShift=bodyShift2*ph+ bodyShift1*(1-ph);

		elseif t<t_pickup[5] then
			--Stand up
			ph=(t-t_pickup[4])/(t_pickup[5]-t_pickup[4]);
			bodyHeight = ph*bodyHeight0 + (1-ph)*bodyHeight2;
			--	qRArm= ph * qRArm0 + (1-ph)*qRArm1;
			bodyShift=bodyShift3*ph+ bodyShift2*(1-ph);
		else
			walk.has_ball=1;
			return "done";
		end
	else	--Throw==1
		local t_pickup = t_throw;
		if t<t_pickup[1] then
			--Windup
			ph=(t)/(t_pickup[1]);
			qLArm= ph * qLArm3 + (1-ph)*qLArm2;
			qRArm= ph * qRArm3 + (1-ph)*qRArm2;
			bodyShift = bodyShift2*ph+ bodyShift3*(1-ph);
		elseif t<t_pickup[2] then
			--Throw
			ph=(t-t_pickup[1])/(t_pickup[2]-t_pickup[1]);
			qLArm= ph * qLArm4 + (1-ph)*qLArm3;
			qRArm= ph * qRArm4 + (1-ph)*qRArm3;
			-- For speed, just command the final position
			qLArm = qLArm4;
			qRArm = qRArm4;
		elseif t<t_pickup[3] then
			--Reposition
			ph = (t-t_pickup[2])/(t_pickup[3]-t_pickup[2]);
			qLArm = ph * qLArm0 + (1-ph)*qLArm4;
			qRArm = ph * qRArm0 + (1-ph)*qRArm4;
			bodyShift = bodyShift0*ph+ bodyShift2*(1-ph);
		else
			walk.has_ball=0;
			return "done";	
		end
	end

	pTorso[3],pTorso[5] = bodyHeight,bodyTilt;
	pLLeg[1],pLLeg[2],pLLeg[3],pLLeg[5],pLLeg[6]=uLeft[1],uLeft[2],zLeft,aLeft,uLeft[3];
	pRLeg[1],pRLeg[2],pRLeg[3],pRLeg[5],pRLeg[6]=uRight[1],uRight[2],zRight,aRight,uRight[3];
	uTorso=util.pose_global(vector.new({-footX,0,0}),uBody);
	pTorso[1],pTorso[2],pTorso[6]=uTorso[1]+bodyShift,uTorso[2],uTorso[3];
	motion_legs();
	Body.set_larm_command(qLArm);
	Body.set_rarm_command(qRArm);
	Body.set_aux_command(qGrip);
end


function motion_legs()

	--Ankle stabilization using gyro feedback
	imuGyr = Body.get_sensor_imuGyrRPY();

	gyro_roll=imuGyr[1];
	gyro_pitch=imuGyr[2];

	ankleShiftX=util.procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4]);
	ankleShiftY=util.procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4]);
	kneeShiftX=util.procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4]);
	hipShiftY=util.procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4]);

	ankleShift[1]=ankleShift[1]+ankleImuParamX[1]*(ankleShiftX-ankleShift[1]);
	ankleShift[2]=ankleShift[2]+ankleImuParamY[1]*(ankleShiftY-ankleShift[2]);
	kneeShift = kneeShift+kneeImuParamX[1]*(kneeShiftX-kneeShift);
	hipShift[2] = hipShift[2] + hipImuParamY[1]*(hipShiftY-hipShift[2]);

	--  pTorso[1], pTorso[2]  = uTorsoActual[1]+uTorsoShift[1], uTorsoActual[2]+uTorsoShift[2];

	qLegs = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);

	qLegs[2] = qLegs[2] + qLHipRollCompensation + hipShift[2];
	qLegs[8] = qLegs[8] + qRHipRollCompensation + hipShift[2];
	qLegs[3] = qLegs[3] + qLHipPitchCompensation;
	qLegs[9] = qLegs[9] + qRHipPitchCompensation;
	qLegs[4] = qLegs[4] + kneeShift;
	qLegs[10] = qLegs[10] + kneeShift;
	qLegs[5] = qLegs[5]  + ankleShift[1];
	qLegs[11] = qLegs[11]  + ankleShift[1];
	qLegs[6] = qLegs[6] + ankleShift[2];
	qLegs[12] = qLegs[12] + ankleShift[2];
	Body.set_lleg_command(qLegs);

end


function exit()
	print("Pickup exit");
	active = false;
	walk.active = true;
	Body.set_lleg_slope(32);
	Body.set_rleg_slope(32);
	--walk.start();
	--[[
	walk.uLeft=util.pose_global(pose_relative(uLeft,uBody),walk.uBody);
	walk.uRight=util.pose_global(pose_relative(uRight,uBody),walk.uBody);
	walk.should_reset_foot=false;
	--]]
end

function set_distance(xdist)
	local dist = math.min(0.15,math.max(0,xdist));

	--[[
	print("===Ball X distance:",dist)

	if dist<0.04 then
	--For 0cm
	bodyTiltTarget=0*math.pi/180;
	qRArm1 = math.pi/180*vector.new({90,-15,0});	--Pickup
	elseif dist<0.08 then
	--For 6cm
	bodyTiltTarget=10*math.pi/180;
	qRArm1 = math.pi/180*vector.new({70,-10,0});	--Pickup
	elseif dist<0.12 then
	--For 10cm
	bodyTiltTarget=20*math.pi/180;
	qRArm1 = math.pi/180*vector.new({50,-10,0});	--Pickup
	elseif dist<0.14 then
	--For 12cm
	bodyTiltTarget=30*math.pi/180;
	qRArm1 = math.pi/180*vector.new({30,0,0});	--Pickup
	else
	--For 16cm
	bodyTiltTarget=45*math.pi/180;
	qRArm1 = math.pi/180*vector.new({20,0,0});	--Pickup
	end

	--]]

end
