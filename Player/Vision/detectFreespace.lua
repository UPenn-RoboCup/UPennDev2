module(..., package.seeall);

require('Config');	-- For Ball and Goal Size
require('ImageProc');
require('HeadTransform');	-- For Projection
require('Vision');
require('Debug');
require('shm');
require('vcm');
require('Body');
require('vector');

-- Define Color
colorOrange = Config.color.orange;
colorYellow = Config.color.yellow;
colorCyan = Config.color.cyan;
colorField = Config.color.field;
colorWhite = Config.color.white;

use_point_goal=Config.vision.use_point_goal;

headZ = Config.head.camOffsetZ;

function detect(color)
  local freespace = {};
  freespace.detect = 1;
  freespace.block = 0; -- whether there are cols being wholely blocked
  freespace.vboundA = {}; -- freespace boundary in mm from labelA
  freespace.vboundB = {}; -- freespace boundary in mm from labelB
  freespace.pboundA = {}; -- freespace boundary in labelA
  freespace.pboundB = {}; -- freespace boundary in labelB
  -- Get label handle
  labelA = Vision.labelA;
  labelB = Vision.labelB;
  
  local pboundA = ImageProc.field_occupancy(labelA.data,labelA.m,labelA.n);
  local pboundB = ImageProc.field_occupancy(labelB.data,labelB.m,labelB.n);
  for i = 1,labelA.m do
    local pbound = vector.new({i,labelA.n-pboundA[i]});
	freespace.pboundA[i],freespace.pboundA[i+labelA.m] =pbound[1],pbound[2];
    local vbound = HeadTransform.rayIntersectA(pbound);
    freespace.vboundA[i],freespace.vboundA[i+labelA.m] = vbound[1],vbound[2];   
  end
  
  for i = 1,labelB.m do
    local pbound = vector.new({i,labelB.n-pboundB[i]});
	freespace.pboundB[i],freespace.pboundB[i+labelB.m] =pbound[1],pbound[2];
    local vbound = HeadTransform.rayIntersectB(pbound);
    freespace.vboundB[i],freespace.vboundB[i+labelB.m] = vbound[1],vbound[2];   
  end


  local horizon = HeadTransform.get_horizonA();
--  print(horizon);
  freespace.nCol = nCol;
  freespace.nRow = nRow;
  return freespace;
end
