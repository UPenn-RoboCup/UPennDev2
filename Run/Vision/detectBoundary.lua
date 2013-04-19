module(..., package.seeall);

local Config = require('Config');	-- For Ball and Goal Size
local ImageProc = require('ImageProc');
local HeadTransform = require('HeadTransform');	-- For Projection
local Vision = require('Vision');
local Debug = require('Debug');
local shm = require('shm');
local vcm = require('vcm');
local Body = require('Body');
local vector = require('vector');


function detect(color)
  local boundary = {};
  boundary.detect = 1;
  boundary.top = {};
  boundary.bottom = {};

  local nCol = Config.camera.width/2/Config.vision.scaleB; -- Search space column

  -- Search label A column by column for freespace
  for nC = 1 , nCol do
    -- Search box width
    local topB = vector.new({nC,1});
	local bottomB = vector.new({nC,Vision.labelB.n});

    --Project to 2D coordinate
    local topV = HeadTransform.rayIntersectB(topB); 
    local bottomV = HeadTransform.rayIntersectB(bottomB); 

    boundary.top[nC],boundary.top[nC+nCol] = topV[1],topV[2];
    boundary.bottom[nC],boundary.bottom[nC+nCol] = bottomV[1],bottomV[2];
  end -- end for search for columns
  return boundary;
end
