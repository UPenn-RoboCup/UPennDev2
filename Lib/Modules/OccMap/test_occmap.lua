module(... or '', package.seeall)

-- Add the required paths
cwd = '.';

uname  = io.popen('uname -s')
system = uname:read();

package.cpath = cwd.."/?.so;"..package.cpath;
package.cpath = cwd.."/../../../Player/Lib/?.so;"..package.cpath;

package.path = cwd.."/../../../Player/Config/?.lua;"..package.path;
package.path = cwd.."/../../../Player/Util/?.lua;"..package.path;
package.path = cwd.."/../../../Player/World/?.lua;"..package.path;

require("util")
require("ocm")
require("OccMap")
require("unix")

local time = unix.time();
print(time)
OccMap.init(50, 25, 40, time);

width = 80;
--free_bound = {0.71, 0.6, 0.5, -0.17, -0.2, -0.3};
--free_bound_type = {1, 1, 1};
free_bound = {0.990064,0.906163,0.834005,0.742290,0.690532,0.644603,0.583879,0.548826,0.517076,0.473873,0.448592,0.425359,0.393025,0.373948,0.356227,0.331100,0.316204,0.302251,0.289153,0.270204,0.271686,0.266695,0.261856,0.263306,0.258598,0.260029,0.261455,0.262876,0.264292,0.265702,0.267108,0.268508,0.269903,0.271293,0.278900,0.280286,0.288065,0.289447,0.297403,0.298781,0.313869,0.322392,0.338670,1.535959,1.532805,1.529690,1.526613,1.523575,1.520574,1.517609,1.514681,1.511787,1.508929,1.506104,1.503313,1.500554,1.497828,1.495134,1.492470,1.489838,1.487235,1.484662,1.482118,1.479602,1.477115,1.474655,1.472222,1.469816,1.467437,1.465083,1.462755,1.460452,1.458173,1.455918,1.453688,1.451481,1.449297,1.447135,1.444997,1.442880,1.054737,0.958137,0.875056,0.777902,0.717806,0.664478,0.600911,0.559868,0.522692,0.478002,0.448152,0.420720,0.387672,0.364958,0.343858,0.318483,0.300600,0.283848,0.268123,0.249331,0.243201,0.233381,0.223858,0.217973,0.208839,0.203080,0.197342,0.191625,0.185930,0.180255,0.174601,0.168967,0.163354,0.157762,0.154667,0.149038,0.145816,0.140152,0.136799,0.131097,0.129859,0.126267,0.124855,0.463456,0.442239,0.421288,0.400596,0.380160,0.359975,0.340036,0.320338,0.300877,0.281650,0.262651,0.243877,0.225324,0.206988,0.188865,0.170951,0.153243,0.135737,0.118430,0.101319,0.084400,0.067670,0.051125,0.034764,0.018582,0.002577,-0.013254,-0.028915,-0.044406,-0.059733,-0.074896,-0.089899,-0.104743,-0.119433,-0.133970,-0.148356,-0.162593};
free_bound_type = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2}

OccMap.vision_update(free_bound, free_bound_type, width, unix.time());
while (true) do
  OccMap.vision_update(free_bound, free_bound_type, width, unix.time());
  unix.sleep(100)
end
--[[
for i = 1, 50 do
  cur_time = unix.time();
  OccMap.time_decay(cur_time);
  OccMap.vision_update(OccMap.empty_userdata(width), 
                      OccMap.empty_userdata(width), width, cur_time);
end
--]]

-- Map Retrieve Test Case with matlab
occmap = OccMap.retrieve_map();
ocm.set_occ_map(occmap);

-- get map updated time
map_updated_time = OccMap.retrieve_map_updated_time();

--[[
-- Map related data retrieve test case
occdata = OccMap.retrieve_data();
util.ptable(occdata.robot_pos);
--]]

--occ.odometry_update(3,2,1);
--util.ptable(occmap.map)
--print(occmap.map[450]);

--[[
for i = 1 , 1 do
  x = 0.05; --math.random()/40;
  y = 0.54; --math.random()/40;
  a = math.random()/40;
  OccMap.odometry_update(x, y, a);
end
--]]

odom = OccMap.retrieve_odometry();
util.ptable(odom);
