module(..., package.seeall);

require('PoseFilter');
require('Filter2D');
require('Config');	
require('Body')
require('shm');
require('vcm');
require('unix'); -- Get Time
require('wcm');

Div = 72;
Interval = 2*math.pi/Div;
HalfInter = Interval/2;

occuFilter = Filter2D.new();
occumap = {};
occumap.t = 0;
occumap.x = {};
occumap.y = {};
occumap.r = vector.zeros(Div);

function entry()
end

function getIdx(theta)
  if theta < 0 then theta = theta + 2*math.pi; end
  theta = theta + HalfInter;
  local fanIdx = math.floor((theta-theta%Interval)/Interval + 0.5)+1;
  if fanIdx > Div then fanIdx = fanIdx - 1; end
  return fanIdx
end

function update()
  --print("update occumap");
  if (vcm.get_freespace_detect() == 1) then
    occumap.t = Body.get_time();
    local freeBound = vcm.get_freespace_bound();
	local freeCol = vcm.get_freespace_nCol(); 
	for idx = 1,freeCol do
      occumap.y[idx] = freeBound[idx];
	  occumap.x[idx] = freeBound[idx+freeCol];
	  
	  local r = math.sqrt(occumap.x[idx]^2+occumap.y[idx]^2);
	  local theta = math.atan2(occumap.x[idx],occumap.y[idx]);
	  occumap.r[getIdx(theta)] = r;
    end
  end
	wcm.set_occumap_t(occumap.t);
	wcm.set_occumap_r(occumap.r);

end
