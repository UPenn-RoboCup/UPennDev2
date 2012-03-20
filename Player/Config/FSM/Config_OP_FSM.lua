module(..., package.seeall);
require('vector')

--FSM parameters

speedFactor = 1.0;

fsm={};

----------------
--BodySearch
----------------
fsm.bodySearch={};
fsm.bodySearch.vSpin = 0.3; --Turn velocity
fsm.bodySearch.timeout = 10.0*speedFactor;

----------------
--BodyChase 
----------------
fsm.bodyChase={};
fsm.bodyChase.maxStep = 0.08;
fsm.bodyChase.rClose = 0.25;
fsm.bodyChase.timeout = 20.0*speedFactor;
fsm.bodyChase.tLost = 3.0*speedFactor;

----------------
--BodyOrbit 
----------------
fsm.bodyOrbit={};
fsm.bodyOrbit.maxStep = 0.06;
fsm.bodyOrbit.rOrbit = 0.20;
fsm.bodyOrbit.rFar = 0.45;
fsm.bodyOrbit.thAlign = 10*math.pi/180;
fsm.bodyOrbit.timeout = 30.0 * speedFactor;
fsm.bodyOrbit.tLost = 3.0*speedFactor;

---------------------------
--BodyPosition (advanced chase-orbit)
---------------------------
fsm.bodyPosition={};
fsm.bodyPosition.maxStep = 0.06;--Normal velocity
fsm.bodyPosition.maxStep2 = 0.08;--Front dash velocity

--Trajectory parameters
fsm.bodyPosition.rTurn = 0.25; 
fsm.bodyPosition.rDist1 = 0.40; 
fsm.bodyPosition.rDist2 = 0.20; 
fsm.bodyPosition.rTurn2 = 0.08; 
fsm.bodyPosition.rOrbit = 0.60; 

fsm.bodyPosition.thClose = {0.08,0.15,10*math.pi/180};
fsm.bodyPosition.tLost =  5.0*speedFactor; 
fsm.bodyPosition.timeout = 30*speedFactor; 


----------------
--BodyApproach
----------------
fsm.bodyApproach={};
fsm.bodyApproach.maxStep = 0.03; --Max walk velocity
fsm.bodyApproach.timeout = 10.0*speedFactor;
fsm.bodyApproach.rFar = 0.45; --Max ball distance
fsm.bodyApproach.tLost = 3.0*speedFactor;--ball detection timeout

--x and y target position for straight kick
fsm.bodyApproach.xTarget0={0, 0.13,0.14}; --min, target, max
fsm.bodyApproach.yTarget0={0.03, 0.05, 0.06}; --min, target ,max

--Target position for straight walkkick 
fsm.bodyApproach.xTarget1={0, 0.17,0.20}; --min, target, max
fsm.bodyApproach.yTarget1={0.03, 0.05, 0.06}; --min, target ,max

--Target position for side walkkick 
fsm.bodyApproach.xTarget2={0, 0.13,0.14}; --min, target, max
fsm.bodyApproach.yTarget2={0.03, 0.05, 0.06}; --min, target ,max


