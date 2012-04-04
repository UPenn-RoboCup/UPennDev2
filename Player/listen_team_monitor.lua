module(... or '', package.seeall)


-- Add the required paths
cwd = '.';
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
package.path = cwd.."/World/?.lua;"..package.path;
package.path = cwd.."/Vision/?.lua;"..package.path;
package.path = cwd.."/Motion/?.lua;"..package.path; 


require ('Config')
require ('cutil')
require ('vector')
require ('serialization')
require ('Comm')
require ('util')
require ('wcm')
require ('gcm')
require ('vcm')
require 'unix'

Comm.init(Config.dev.ip_wireless,111111);
print('Receiving Team Message From',Config.dev.ip_wireless);

function push_team_struct(obj)
--  wcm.set_teamdata_id(obj.id);
  id=obj.id;
--  states.role[id]=obj.id; --robot id?
  states.posex[id]=obj.pose.x;
  states.posey[id]=obj.pose.y;
  states.posea[id]=obj.pose.a;
  states.ballx[id]=obj.ball.x;
  states.bally[id]=obj.ball.y;
  states.ballt[id]=obj.ball.t;
  states.fall[id]=obj.fall;
  states.penalty[id]=obj.penalty;
  states.battery_level[id]=obj.battery_level;
  
  wcm.set_teamdata_posex(states.posex)
  wcm.set_teamdata_posey(states.posey)
  wcm.set_teamdata_posea(states.posea)
  wcm.set_teamdata_ballx(states.ballx)
  wcm.set_teamdata_bally(states.bally)
  wcm.set_teamdata_ballt(states.ballt)
  wcm.set_teamdata_fall(states.fall)
  wcm.set_teamdata_penalty(states.penalty)
  wcm.set_teamdata_battery_level(states.battery_level)
end

while( true ) do
  while (Comm.size() > 0) do
    msg=Comm.receive();
print(msg)
    t = serialization.deserialize(msg);
    if (t and (t.teamNumber) and (t.teamNumber == state.teamNumber) and (t.id)) then
--      t.tReceive = Body.get_time();
      push_team_struct(t);
    end
  end
end
