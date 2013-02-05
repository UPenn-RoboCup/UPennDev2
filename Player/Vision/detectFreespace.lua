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
require('util');

-- Define Color
colorOrange = Config.color.orange;
colorYellow = Config.color.yellow;
colorCyan = Config.color.cyan;
colorField = Config.color.field;
colorWhite = Config.color.white;

use_point_goal=Config.vision.use_point_goal;

headZ = Config.head.camOffsetZ;

enable_lut_for_obstacle = Config.vision.enable_lut_for_obstacle or 0;

function detect(color)
  local freespace = {};
  freespace.detect = 1;
  freespace.block = 0; -- whether there are cols being wholely blocked
  freespace.vboundB = {}; -- freespace boundary in mm from labelB
  freespace.pboundB = {}; -- freespace boundary in labelB
  freespace.tboundB = {}; -- freespace boundary type in labelB

  -- Get label handle
  labelB = Vision.labelB;
 
  if enable_lut_for_obstacle == 1 then
    FreeB = ImageProc.field_occupancy(labelB.data_obs,labelB.m,labelB.n);
  else
    FreeB = ImageProc.field_occupancy(labelB.data,labelB.m,labelB.n);
  end
  
  for i = 1,labelB.m do
    local pbound = vector.new({i,labelB.n-FreeB.range[i]});
		freespace.pboundB[i],freespace.pboundB[i+labelB.m] = pbound[1],pbound[2];
    vbound, outrange = HeadTransform.rayIntersectB(pbound);
    freespace.vboundB[i],freespace.vboundB[i+labelB.m] = vbound[1],vbound[2];   
    freespace.tboundB[i] = FreeB.flag[i];
    if outrange == 1 then
      freespace.tboundB[i] = 3;
    end
  end

  -- count whole block columns and decide if the view is blocked
  blocked_col = 0
  for i = 1, labelB.m do
    if freespace.tboundB[i] == 3 then
      blocked_col = blocked_col + 1;
    end
  end
  if blocked_col > 0.75 * labelB.m then
    vcm.set_freespace_allBlocked(1);
  else
    vcm.set_freespace_allBlocked(0);
  end

  local horizon = HeadTransform.get_horizonA();
--  print(horizon);
  freespace.nCol = labelB.m;
  freespace.nRow = labelB.n;
  return freespace;
end
