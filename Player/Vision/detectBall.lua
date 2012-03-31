module(..., package.seeall);

require('Config');      -- For Ball and Goal Size
require('ImageProc');
require('HeadTransform');       -- For Projection
require('Vision');
require('Body');
require('shm');
require('vcm');
require('Detection');
require('Debug');

-- Define Color
colorOrange = Config.color.orange;
colorYellow = Config.color.yellow;
colorCyan = Config.color.cyan;
colorField = Config.color.field;
colorWhite = Config.color.white;

diameter = Config.vision.ball.diameter;
check_for_ground = Config.vision.ball.check_for_ground;

th_min_color=Config.vision.ball.th_min_color;
th_min_color2=Config.vision.ball.th_min_color2;
th_min_fill_rate=Config.vision.ball.th_min_fill_rate;
th_height_max=Config.vision.ball.th_height_max;
th_ground_boundingbox=Config.vision.ball.th_ground_boundingbox;
th_min_green1=Config.vision.ball.th_min_green1;
th_min_green2=Config.vision.ball.th_min_green2;

function detect(color)
  local ball = {};
  ball.detect = 0;
  vcm.add_debug_message(string.format("Ball: pixel count: %d\n",
	Vision.colorCount[color]));

  -- threshold check on the total number of ball pixels in the image
  if (Vision.colorCount[color] < th_min_color) then  	
    vcm.add_debug_message("pixel count fail");
    return ball;  	
  end

  -- find connected components of ball pixels
  ballPropsB = ImageProc.connected_regions(Vision.labelB.data, Vision.labelB.m, Vision.labelB.n, color);
--TODO: horizon cutout
-- ballPropsB = ImageProc.connected_regions(labelB.data, labelB.m, 
--	labelB.n, HeadTransform.get_horizonB(),color);
  if (#ballPropsB == 0) then return ball; end

-- get largest blob
-- TODO: check max k largest blobs

  ball.propsB = ballPropsB[1];
  ball.propsA = Vision.bboxStats(color, ballPropsB[1].boundingBox);
  ball.bboxA = Vision.bboxB2A(ballPropsB[1].boundingBox);

  local fill_rate = ball.propsA.area / 
	Vision.bboxArea(ball.propsA.boundingBox);

  vcm.add_debug_message(string.format("Area:%d\nFill rate:%2f\n",
     ball.propsA.area,fill_rate));

  if (ball.propsA.area < th_min_color2) then
    vcm.add_debug_message("Area check fail");
    return ball;
  end
  if (fill_rate < th_min_fill_rate) then
    vcm.add_debug_message("Fillrate check fail");
    return ball;
  end

  -- diameter of the area
  dArea = math.sqrt((4/math.pi)*ball.propsA.area);

  -- Find the centroid of the ball
  ballCentroid = ball.propsA.centroid;

  -- Coordinates of ball
  scale = math.max(dArea/diameter, ball.propsA.axisMajor/diameter);
  v = HeadTransform.coordinatesA(ballCentroid, scale);

  --Ball height check
  vcm.add_debug_message(string.format(
	"Ball v0: %.2f %.2f %.2f\n",v[1],v[2],v[3]));
  if v[3] > th_height_max then
    vcm.add_debug_message("Height check fail");
    return ball;
  end

  if check_for_ground>0 then
    -- ground check
    -- is ball cut off at the bottom of the image?
    local vmargin=Vision.labelA.n-ballCentroid[1];
    if vmargin > dArea then
    -- bounding box below the ball
      fieldBBox = {};
      fieldBBox[1] = ballCentroid[1] + th_ground_boundingbox[1];
      fieldBBox[2] = ballCentroid[1] + th_ground_boundingbox[2];
      fieldBBox[3] = ballCentroid[2] + .5*dArea 
				     + th_ground_boundingbox[3];
      fieldBBox[4] = ballCentroid[2] + .5*dArea 
 				     + th_ground_boundingbox[4];
      -- color stats for the bbox
      fieldBBoxStats = ImageProc.color_stats(Vision.labelA.data, Vision.labelA.m, Vision.labelA.n, colorField, fieldBBox);
      -- is there green under the ball?

      vcm.add_debug_message(string.format("Green check:%d\n",
		   	   fieldBBoxStats.area));

      if (fieldBBoxStats.area < th_min_green1) then
        -- if there is no field under the ball 
	-- it may be because its on a white line
        whiteBBoxStats = ImageProc.color_stats(Vision.labelA.data,
 	     Vision.labelA.m, Vision.labelA.n, colorWhite, fieldBBox);
        if (whiteBBoxStats.area < th_min_green2) then
          vcm.add_debug_message("Green check fail");
          return ball;
        end
      end
    end
  end
  
  v=HeadTransform.projectGround(v,diameter/2);

  ball.v = v;
  ball.detect = 1;
  ball.r = math.sqrt(ball.v[1]^2 + ball.v[2]^2);

  -- How much to update the particle filter
  ball.dr = 0.25*ball.r;
  ball.da = 10*math.pi/180;

  vcm.add_debug_message(string.format(
	"Ball detected\nv: %.2f %.2f %.2f\n",v[1],v[2],v[3]));
  return ball;
end
