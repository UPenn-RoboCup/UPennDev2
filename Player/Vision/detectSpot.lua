module(..., package.seeall);

require('Config');	-- For Ball and Goal Size
require('ImageProc');
require('HeadTransform');	-- For Projection
require('Vision');
require('vcm'); -- for horizon checks
-- Dependency
require('Detection');


-- Define Color
colorOrange = 1;
colorYellow = 2;
colorCyan = 4;
colorField = 8;
colorWhite = 16;

use_point_goal=Config.vision.use_point_goal;
headInverted=Config.vision.headInverted;

function detect()
  --TODO: test spot detection

  spot = {};
  spot.detect = 0;

  if (Vision.colorCount[colorWhite] < 100) then 
    vcm.add_debug_message(string.format("\nNot enough white pixels\n"));
    return spot; 
  end
  if (Vision.colorCount[colorField] < 5000) then 
    vcm.add_debug_message(string.format("\nNot enough field pixels\n"));
    return spot; 
  end

  local spotPropsB = ImageProc.field_spots(Vision.labelB.data, Vision.labelB.m, Vision.labelB.n);
  if (not spotPropsB) then 
    vcm.add_debug_message(string.format("\nFailed spotPropsB\n"));
    return spot; 
  end
  spot.propsB = spotPropsB[1];
  if (spot.propsB.area < 6) then 
    vcm.add_debug_message(string.format("\nspot.propsB.area is less than 6\n"));
    return spot;
  end

  -- get the color statistics of the region (in the large size image)
  local spotStats = Vision.bboxStats(colorWhite, spot.propsB.boundingBox);

  -- check the major and minor axes
  -- the spot is symmetrical so the major and minor axes should be the same
  --debugprint('Spot: checking ratio');

  if (spotStats.axisMinor < .2*spotStats.axisMajor) then
    vcm.add_debug_message(string.format("\nFAILED AXIS TEST: axis Minor value is %f\naxis Major value is %f\n", spotStats.axisMinor, spotStats.axisMajor));
    return spot;
  end
  -- Makes sure that the Spot is below the horizon;
  horizonA = vcm.get_image_horizonA();
  horizonB = vcm.get_image_horizonB();
  vcm.add_debug_message(string.format("\nhorizonA is %d\n horizonB is %d\nspotStats.centroid[1] is at %f, spotStats.centroid[2] is at  %f\n", horizonA, horizonB, spotStats.centroid[1], spotStats.centroid[2]));
  if (spotStats.centroid[1] < horizonB or spotStats.centroid[2] < horizonB) then
    vcm.add_debug_message(string.format("\nHorizon fail\n"));
    return spot;
  end
  -- util.ptable (spotStats.centroid)

  spot.propsA = spotStats;

  local	vcentroid = HeadTransform.coordinatesA(spotStats.centroid, 1); 
  vcentroid = HeadTransform.projectGround(vcentroid,0);
  vcentroid[4] = 1;
  spot.v = vcentroid;
  --debugprint('Spot found');
  vcm.add_debug_message(string.format("\nSpot detected; launching tactical nuke\n"));
  spot.detect = 1;
  return spot;
end
