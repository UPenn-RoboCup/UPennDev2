--[[
Servo bias set code for darwin OP by SJ
Haven't checked for nao compatibility at all!!
--]]


module(... or "", package.seeall)

require('unix')
webots = false;
darwin = true;
local cwd = unix.getcwd();
-- the webots sim is run from the WebotsController dir (not Player)
if string.find(cwd, "WebotsController") then
  webots = true;
  cwd = cwd.."/Player"
  package.path = cwd.."/?.lua;"..package.path;
end

computer = os.getenv('COMPUTER') or "";
if (string.find(computer, "Darwin")) then
   -- MacOS X uses .dylib:
   package.cpath = cwd.."/Lib/?.dylib;"..package.cpath;
else
   package.cpath = cwd.."/Lib/?.so;"..package.cpath;
end

package.path = cwd.."/Util/?.lua;"..package.path;
package.path = cwd.."/Config/?.lua;"..package.path;
package.path = cwd.."/Lib/?.lua;"..package.path;
package.path = cwd.."/Dev/?.lua;"..package.path;
package.path = cwd.."/Motion/?.lua;"..package.path;
package.path = cwd.."/Vision/?.lua;"..package.path;
package.path = cwd.."/World/?.lua;"..package.path;
package.path = cwd.."/BodyFSM/?.lua;"..package.path;
package.path = cwd.."/HeadFSM/?.lua;"..package.path;

require('Config');
require('Body')
require('vector')
require("getch")
require('Kinematics');

-- initialize state machines
getch.enableblock(1);


jointNames={"HipYaw","HipRoll","HipPitch","KneePitch","AnklePitch","AnkleRoll"};
jointindex=4;
hardness=vector.new({0,0,0,0,0,0});

incBiasStep=vector.zeros(12);
radPerStep=vector.zeros(12);
dir=vector.ones(12);

for i=1,12 do
    radPerStep[i]=Config.servo.moveRange[5+i]/Config.servo.steps[5+i];
end

for i=1, #Config.servo.dirReverse do
    dirReverse=Config.servo.dirReverse[i];
    if dirReverse>5 and dirReverse<18 then 
    	dir[Config.servo.dirReverse[i]-5]=-1;
    end
end


-- main loop


Body.set_lleg_hardness(0);
Body.set_rleg_hardness(0);
Body.set_syncread_enable(1);
q=vector.zeros(12);
q0=vector.zeros(12);
isended=false;

count=0;
t0 = unix.time();

function init()
	bodyHeight = Config.walk.bodyHeight;
	footX = Config.walk.footX;
	footY = Config.walk.footY;
	supportX = Config.walk.supportX;
	qLArm = Config.walk.qLArm;
	qRArm = Config.walk.qRArm;

	bodyTilt=Config.walk.bodyTilt;
	pTorso = vector.new({0, 0, bodyHeight, 0,bodyTilt,0});
	pLLeg = vector.new({-supportX+footX, footY, 0, 0,0,0});
	pRLeg = vector.new({-supportX+footX, -footY, 0, 0,0,0});
--q0 = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, 0);
	Body.set_larm_hardness(0);
	Body.set_rarm_hardness(0);
	Body.set_larm_command(qLArm);
	Body.set_rarm_command(qRArm);

	Body.set_head_command({0,0});

end

print(" Key commands \n j/l: Left adjust\n i/,: Right adjust\n 1/2: change joint");
print(" 5/6: torque off/on for all joints \n 7/8: torque off/on for current couple of joints");
print(" k: set current reading for current couple of joints\n t: set current reading for all joints  \n h/;: set current reading for left/right joint");
print(" 9: Exit")

function update()

  local str=getch.get();
  if #str>0 then
	local byte=string.byte(str,1);
		q1=vector.slice(Body.get_sensor_position(),6,18);



		if byte==string.byte("5") then	
			hardness=vector.new({0,0,0,0,0,0});
		elseif byte==string.byte("6") then 
			hardness=vector.new({1,1,1,1,1,1});
		elseif byte==string.byte("7") then hardness[jointindex]=0;	
		elseif byte==string.byte("8") then hardness[jointindex]=1;
			

		elseif byte==string.byte("2") then jointindex=jointindex%6+1;
		elseif byte==string.byte("1") then jointindex=(jointindex-2)%6+1;		

		elseif byte==string.byte("j") then		
			incBiasStep[jointindex]=incBiasStep[jointindex]-1;
		elseif byte==string.byte("l") then		
			incBiasStep[jointindex]=incBiasStep[jointindex]+1;
		elseif byte==string.byte("i") then 
			incBiasStep[jointindex+6]=incBiasStep[jointindex+6]-1;
		elseif byte==string.byte(",") then 
			incBiasStep[jointindex+6]=incBiasStep[jointindex+6]+1;
		
		elseif byte==string.byte("k") then
			incBiasStep[jointindex]=(q1[jointindex]-q0[jointindex]) /radPerStep[jointindex];
			incBiasStep[jointindex+6]=(q1[jointindex+6]-q0[jointindex+6])/radPerStep[jointindex+6];
		elseif byte==string.byte("t") then
			for i=1,12 do
				incBiasStep[i]= - dir[i]* (q1[i]-q0[i]) /radPerStep[i];
			end
			for i=1,6 do
				print(string.format("\n %s: L%d, R%d)",
				jointNames[i],incBiasStep[i],incBiasStep[i+6]));
			end
		elseif byte==string.byte("h") then
			incBiasStep[jointindex]=q1[jointindex]/radPerStep[jointindex];
		elseif byte==string.byte(";") then
			incBiasStep[jointindex+6]=q1[jointindex+6]/radPerStep[jointindex+6];

		elseif byte==string.byte("9") then
			isended=true;
		else
			print(" Key commands \n j/l: Left adjust\n i/,: Right adjust\n 1/2: change joint");
			print(" 5/6: torque off/on for all joints \n 7/8: torque off/on for current couple of joints");
			print(" k: set current reading for current couple of joints\n t: set current reading for all joints  \n h/;: set current reading for left/right joint");
			print(" 9: Exit")
		end

		for i=1,12 do q[i]=dir[i]*incBiasStep[i]*radPerStep[i];end
		print(string.format("\n %s: L%d, R%d)",
			jointNames[jointindex],incBiasStep[jointindex],incBiasStep[jointindex+6]));

		Body.set_lleg_hardness(hardness);
		Body.set_rleg_hardness(hardness);

  end

   Body.set_lleg_command(q0-q);
end


init()
while not isended do
 update()
end
Body.set_lleg_hardness(0);
Body.set_rleg_hardness(0);

print(string.format("walk.servoBias={%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}",
	unpack(Config.walk.servoBias-incBiasStep)));

