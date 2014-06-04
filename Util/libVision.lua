-- Vision auxiliary functions
local libVision={}

local Config     = require'Config'
local jpeg       = require'jpeg'
local udp        = require'udp'
local mp         = require'msgpack'
local ColorLUT   = require'ColorLUT'
local ImageProc  = require'ImageProc'


local scaleA = Config.vision.scaleA  
local scaleB = Config.vision.scaleB

function libVision.add_debug_message(str)
--  print(str)
end


function libVision.bboxStats(color, bboxB, rollAngle, scale, labelA)
  scale = scale or scaleB;
  bboxA = {};
  bboxA[1] = scale*bboxB[1];
  bboxA[2] = scale*bboxB[2] + scale - 1;
  bboxA[3] = scale*bboxB[3];
  bboxA[4] = scale*bboxB[4] + scale - 1;
  if rollAngle then
 --hack: shift boundingbox 1 pix helps goal detection
 --not sure why this thing is happening...
--    bboxA[1]=bboxA[1]+1;
      bboxA[2]=bboxA[2]+1;
    return ImageProc.tilted_color_stats(
      labelA.data, labelA.m, labelA.n, color, bboxA,rollAngle);
  else
    return ImageProc.color_stats(
      labelA.data, labelA.m, labelA.n, color, bboxA);
  end
end

function libVision.bboxB2A(bboxB)
  bboxA = {};
  bboxA[1] = scaleB*bboxB[1];
  bboxA[2] = scaleB*bboxB[2] + scaleB - 1;
  bboxA[3] = scaleB*bboxB[3];
  bboxA[4] = scaleB*bboxB[4] + scaleB - 1;
  return bboxA;
end

function libVision.bboxArea(bbox)
  return (bbox[2] - bbox[1] + 1) * (bbox[4] - bbox[3] + 1);
end

return libVision
