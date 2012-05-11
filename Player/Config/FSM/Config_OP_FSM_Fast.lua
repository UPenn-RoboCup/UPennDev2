module(..., package.seeall);
require('vector')

--FSM parameters

--How much should we slow down all SM timings?
speedFactor = 1.0;

fsm={};

--Should we consider obstacle?
fsm.enable_obstacle_detection = 1;

--fsm.playMode = 1; --For Demo without orbit
--fsm.playMode = 2; --Simple Behavior with orbit
fsm.playMode = 3; --Advanced Behavior 

fsm.enable_walkkick = 1;

fsm.wait_kickoff = 1; --initial wait at opponent's kickoff

--------------------------------------------------
--BodyReady : make robot move to initial position
--------------------------------------------------
fsm.bodyReady={};
fsm.bodyReady.maxStep = 0.06;
fsm.bodyReady.thClose = {0.30,15*math.pi/180} --r and theta
fsm.bodyReady.tStart = 5.0;  --initial localization time

--------------------------------------------------
--BodySearch : make robot turn to search the ball
--------------------------------------------------
fsm.bodySearch={};
fsm.bodySearch.vSpin = 0.3; --Turn velocity
fsm.bodySearch.timeout = 10.0*speedFactor;

--------------------------------------------------
--BodyChase : move the robot directly towards the ball
--------------------------------------------------
fsm.bodyChase={};
fsm.bodyChase.maxStep = 0.08;
fsm.bodyChase.rClose = 0.35;
fsm.bodyChase.timeout = 20.0*speedFactor;
fsm.bodyChase.tLost = 3.0*speedFactor;
fsm.bodyChase.rFar = 1.2;

--------------------------------------------------
--BodyAnticipate : Sit down and wait for kick (goalie)
--------------------------------------------------
fsm.bodyAnticipate={};
fsm.bodyAnticipate.rClose = 1.0;
-- How far out of position are we allowed to be?
fsm.bodyAnticipate.thFar = {0.2,0.2,15*math.pi/180};

fsm.bodyGoaliePosition = {}; 
fsm.bodyGoaliePosition.thClose = {.2, .2,10*math.pi/180}

--------------------------------------------------
--BodyOrbit : make the robot orbit around the ball
--------------------------------------------------
fsm.bodyOrbit={};
fsm.bodyOrbit.maxStep = 0.03;
fsm.bodyOrbit.rOrbit = 0.25;
fsm.bodyOrbit.rFar = 0.40;
fsm.bodyOrbit.thAlign = 10*math.pi/180;
fsm.bodyOrbit.timeout = 30.0 * speedFactor;
fsm.bodyOrbit.tLost = 3.0*speedFactor;

--------------------------------------------------
--BodyPosition : Advanced chase-orbit
--------------------------------------------------
fsm.bodyPosition={};

--Trajectory parameters
fsm.bodyPosition.rTurn = 0.25; 
fsm.bodyPosition.rDist1 = 0.40; 
fsm.bodyPosition.rDist2 = 0.20; 
fsm.bodyPosition.rTurn2 = 0.08; 
fsm.bodyPosition.rOrbit = 0.60; 

fsm.bodyPosition.rClose = 0.35; 
fsm.bodyPosition.thClose = {0.15,0.15,10*math.pi/180};

fsm.bodyPosition.tLost =  5.0*speedFactor; 
fsm.bodyPosition.timeout = 30*speedFactor; 

--Velocity generation parameters

--Slow speed
fsm.bodyPosition.maxStep1 = 0.06;

--Medium speed
fsm.bodyPosition.maxStep2 = 0.07;
fsm.bodyPosition.rVel2 = 0.5;
fsm.bodyPosition.aVel2 = 45*math.pi/180;
fsm.bodyPosition.maxA2 = 0.2;
--fsm.bodyPosition.maxY2 = 0.02;
fsm.bodyPosition.maxY2 = 0.01;

--Full speed front dash
fsm.bodyPosition.maxStep3 = 0.10;
fsm.bodyPosition.rVel3 = 0.7;
fsm.bodyPosition.aVel3 = 20*math.pi/180;
fsm.bodyPosition.maxA3 = 0.0;
fsm.bodyPosition.maxY3 = 0.0;

--------------------------------------------------
--BodyApproach :  Align the robot for kick
--------------------------------------------------
fsm.bodyApproach={};
fsm.bodyApproach.maxStep = 0.04; --Max walk velocity
fsm.bodyApproach.timeout = 10.0*speedFactor;
fsm.bodyApproach.rFar = 0.45; --Max ball distance
fsm.bodyApproach.tLost = 3.0*speedFactor;--ball detection timeout

--x and y target position for stationary straight kick
fsm.bodyApproach.xTarget11={0, 0.11,0.13}; --min, target, max
fsm.bodyApproach.yTarget11={0.01, 0.035, 0.04}; --min, target ,max

--x and y target position for stationary kick to left
fsm.bodyApproach.xTarget12={0, 0.11,0.13}; --min, target, max
fsm.bodyApproach.yTarget12={-0.005, 0.01, 0.025}; --min, target ,max

--Target position for straight walkkick 
fsm.bodyApproach.xTarget21={0, 0.13,0.15}; --min, target, max
fsm.bodyApproach.yTarget21={0.01, 0.035, 0.04}; --min, target ,max

--Target position for side walkkick to left
fsm.bodyApproach.xTarget22={0, 0.13,0.15}; --min, target, max
fsm.bodyApproach.yTarget22={0.005, 0.02, 0.035}; --min, target ,max

--------------------------------------------------
--BodyAlign : Align robot before kick
--------------------------------------------------
-------------------------------------------------
--BodyKick : Stationary Kick
--------------------------------------------------
fsm.bodyKick={};

--initial wait 
fsm.bodyKick.tStartWait = 0.5;
fsm.bodyKick.tStartWaitMax = 1.0;
fsm.bodyKick.thGyroMag = 100; 

--Longer wait (until we set gyro values correctly)
fsm.bodyKick.tStartWait = 1.0;
fsm.bodyKick.tStartWaitMax = 1.5;





--ball position checking params
fsm.bodyKick.kickTargetFront = {0.12,0.035};

--For kicking to the left
fsm.bodyKick.kickTargetSide = {0.12,0.01};
fsm.bodyKick.kickTh = {0.03,0.025};

--delay for camera following the ball
fsm.bodyKick.tFollowDelay = 2.2; 
--------------------------------------------------
--BodyWalkKick : Dynamic Kick
--------------------------------------------------
fsm.bodyWalkKick={};
fsm.bodyWalkKick.timeout = 2.0*speedFactor; 
--------------------------------------------------
--BodyGotoCenter : Going to center when ball is lost
--------------------------------------------------
fsm.bodyGotoCenter={};
fsm.bodyGotoCenter.maxStep=0.06;
fsm.bodyGotoCenter.rClose=0.30;
fsm.bodyGotoCenter.timeout=10.0*speedFactor;

--------------------------------------------------
--HeadTrack : Track the ball
--------------------------------------------------
fsm.headTrack = {};
fsm.headTrack.timeout = 6.0 * speedFactor;
fsm.headTrack.tLost = 1.5 * speedFactor;
fsm.headTrack.minDist = 0.30; --If ball is closer than this, don't look up
fsm.headTrack.fixTh={0.20,0.08}; --Fix yaw axis if ball is within this box

--------------------------------------------------
--HeadReady : Track the horizonal line for localization
--------------------------------------------------
fsm.headReady={}
fsm.headReady.dist = 3.0; 
fsm.headReady.height = 0.5; 
fsm.headReady.tScan= 1.0*speedFactor; 

--------------------------------------------------
--HeadReadyLookGoal : Look Goal during bodyReady
--------------------------------------------------
fsm.headReadyLookGoal={}
fsm.headReadyLookGoal.timeout = 1.5 * speedFactor;

--------------------------------------------------
--HeadScan: Scan around for ball
--------------------------------------------------
fsm.headScan={};
fsm.headScan.pitch0 = 25*math.pi/180;
fsm.headScan.pitchMag = 25*math.pi/180;
fsm.headScan.yawMag = 60*math.pi/180;
fsm.headScan.pitchTurn0 = 20*math.pi/180;
fsm.headScan.pitchTurnMag = 20*math.pi/180;
fsm.headScan.yawMagTurn = 45*math.pi/180;
fsm.headScan.tScan = 3.0*speedFactor;
fsm.headScan.timeout = 7.0*speedFactor; --to headLookGoal

--------------------------------------------------
--HeadKick: Fix headangle for approaching
--------------------------------------------------
fsm.headKick={};
fsm.headKick.pitch0=58*math.pi/180;
fsm.headKick.xMax = 0.30;
fsm.headKick.yMax = 0.07;
fsm.headKick.tLost = 3.0*speedFactor;
fsm.headKick.timeout = 3.0*speedFactor;

--------------------------------------------------
--HeadKickFollow: Follow ball after kick
--------------------------------------------------
fsm.headKickFollow={};
fsm.headKickFollow.pitch={50*math.pi/180, 0*math.pi/180};
fsm.headKickFollow.pitchSide = 30*math.pi/180;
fsm.headKickFollow.yawMagSide = 90*math.pi/180;
fsm.headKickFollow.tFollow = 1.0*speedFactor;

--------------------------------------------------
--HeadLookGoal: Look up to see the goal
--------------------------------------------------
fsm.headLookGoal={};
fsm.headLookGoal.yawSweep = 50*math.pi/180;
fsm.headLookGoal.tScan = 1.0*speedFactor;

--------------------------------------------------
--HeadSweep: Look around to find the goal
--------------------------------------------------
fsm.headSweep={};
fsm.headSweep.tScan=1.0*speedFactor;
