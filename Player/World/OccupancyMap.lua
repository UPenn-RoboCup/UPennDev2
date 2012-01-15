module(..., package.seeall);

require('PoseFilter');
require('Filter2D');
require('Config');	
require('Body')
require('shm');
require('vcm');
require('unix'); -- Get Time
require('wcm');

nCol = Config.camera.width/2;
Div = Config.occmap.div;
Interval = 2*math.pi/Div;
HalfInter = Interval/2;
occDa = 2.5*math.pi/180;
occDr = 0.1;

occFilter = {};

occmap = {};
occmap.t = 0;
occmap.x = {};
occmap.y = {};
occmap.r = vector.zeros(Div);

function entry()
  for i = 1,Div do
    occFilter[i] = Filter2D.new();
	occFilter[i]:observation_xy(0.0,0.0,occDa,occDr);
  end
end

function getIdx(theta)
  theta = -theta;
  if theta < 0 then theta = theta + 2*math.pi; end
  theta = theta + HalfInter;
  local fanIdx = math.floor((theta-theta%Interval)/Interval + 0.5)+1;
  if fanIdx > Div then fanIdx = fanIdx - 1; end
  return fanIdx
end

function update_odometry()
  
end

--[[ Another Approach for testing
function update_occ()
  local noccmap = {};
  noccmap.r = vector.zeros(Div);
  noccmap.n = vector.zeros(Div);
  if (vcm.get_freespace_detect() == 1) then
--  print("update occmap");
    occmap.t = Body.get_time();
    local freeBound = vcm.get_freespace_bound();
	-- Filter Update
    for i = 1,nCol do
      local v = vector.new({freeBound[i],freeBound[i+nCol]});
	  occFilter[i]:observation_xy(v[1],v[2],occDa,occDr);
      local xyOcc = vector.new({0,0});
      xyOcc[1],xyOcc[2] = occFilter[i]:get_ra();
      local r = math.sqrt(xyOcc[1]^2+xyOcc[2]^2);
      local a = math.atan2(xyOcc[2],xyOcc[1]);
	  local idx = getIdx(a);
	  noccmap.r[idx] = noccmap.r[idx] + r;
      noccmap.n[idx] = noccmap.n[idx] + 1; 
    end
    for i = 1,Div do
	  if (noccmap.n[i]~=0) then
        noccmap.r[i] = noccmap.r[i]/noccmap.n[i];
        occmap.r[i] = noccmap.r[i];
      end
	end
  end
  wcm.set_occmap_t(occmap.t);
  wcm.set_occmap_r(occmap.r);
end
--]]

function update()
  if (vcm.get_freespace_detect() == 1) then
--  print("update occmap");
    occmap.t = Body.get_time();
    local freeBound = vcm.get_freespace_vboundA();
	-- Filter Update
    for i = 1,nCol do
      local v = vector.new({freeBound[i],freeBound[i+nCol]});
	  local r = math.sqrt(v[1]^2 + v[2]^2);
      local a = math.atan2(v[2], v[1]);
      local idx = getIdx(a);
	  occFilter[idx]:observation_ra(r,a,occDa,occDr);
    end
  end
  for i = 1,Div do
	  local raOcc = vector.new({0,0});
      raOcc[1],raOcc[2] = occFilter[i]:get_ra();
--	  print(string.format('%d %1.3f %3.3f',i,raOcc[1],raOcc[2]*180/math.pi));
      occmap.r[i] = raOcc[1];
  end
  wcm.set_occmap_t(occmap.t);
  wcm.set_occmap_r(occmap.r);
end
