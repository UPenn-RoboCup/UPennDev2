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


headZ = Config.head.camOffsetZ;

function detect(color)
  local boundary = {};
  boundary.detect = 1;
  boundary.top = {};
  boundary.bottom = {};

  -- Separate Frame into 8 columns : 80 x 60
  local nCol = Config.vision.freespace_scanColA; -- Search space column

  -- Search label A column by column for freespace
  for nC = 1 , nCol do
    -- Search box width
    local Xoff = Vision.labelA.m/nCol;
    local topA = {(nC-0.5)*Xoff,1}
	local bottomA = {(nC-0.5)*Xoff,Vision.labelA.n}

    --Project to 2D coordinate
		-- TODO 0.1 adpated from previous code, but did not make sense
    local topV = HeadTransform.rayIntersectA(topA); 
    local bottomV = HeadTransform.rayIntersectA(bottomA); 

    boundary.top[nC],boundary.top[nC+nCol] = topV[1],topV[2];
    boundary.bottom[nC],boundary.bottom[nC+nCol] = bottomV[1],bottomV[2];
  end -- end for search for columns

  return boundary;
end
