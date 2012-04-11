module(..., package.seeall);

require('Config');	-- For Ball and Goal Size
require('ImageProc');
require('HeadTransform');	-- For Projection
require('Body')
require('Vision');

-- Dependency
require('Detection');

-- Define Color
colorOrange = Config.color.orange;
colorYellow = Config.color.yellow;
colorCyan = Config.color.cyan;
colorField = Config.color.field;
colorWhite = Config.color.white;

--Use tilted boundingbox? (robots with nonzero bodytilt)
use_tilted_bbox = Config.vision.use_tilted_bbox or 0;
--Use center post to determine post type (disabled for OP)
use_centerpost=Config.vision.goal.use_centerpost or 0;
--Check the bottom of the post for green
check_for_ground = Config.vision.goal.check_for_ground or 0;


--Post dimension
postDiameter = 0.10;
postHeight = Config.world.goalHeight or 0.80;
goalWidth = Config.world.goalWidth or 1.40;

--------------------------------------------------------------
--Vision threshold values (to support different resolutions)
--------------------------------------------------------------
th_min_color_count=Config.vision.goal.th_min_color_count;
th_min_area = Config.vision.goal.th_min_area;
th_nPostB = Config.vision.goal.th_nPostB;
th_min_orientation = Config.vision.goal.th_min_orientation;
th_min_fill_extent = Config.vision.goal.th_min_fill_extent;
th_aspect_ratio = Config.vision.goal.th_aspect_ratio;
th_edge_margin = Config.vision.goal.th_edge_margin;
th_bottom_boundingbox = Config.vision.goal.th_bottom_boundingbox;
th_ground_boundingbox = Config.vision.goal.th_ground_boundingbox;
th_min_green_ratio = Config.vision.goal.th_min_green_ratio;
th_min_bad_color_ratio = Config.vision.goal.th_min_bad_color_ratio;
th_goal_separation = Config.vision.goal.th_goal_separation;
th_min_area_unknown_post = Config.vision.goal.th_min_area_unknown_post;

function detect(color,color2)
  if color==colorYellow then vcm.add_debug_message("\nGoal: Yellow post check\n")
  else vcm.add_debug_message("\nGoal: Blue post check\n"); end
  local goal = {};
  goal.detect = 0;

  local postB;
  if use_tilted_bbox>0 then
    --where shoud we update the roll angle? HeadTransform?
    tiltAngle = HeadTransform.getCameraRoll();
    vcm.set_camera_rollAngle(tiltAngle);
    postB = ImageProc.tilted_goal_posts(Vision.labelB.data, 
	Vision.labelB.m, Vision.labelB.n, color, th_nPostB,tiltAngle);
  else
    tiltAngle=0;
    postB = ImageProc.goal_posts(Vision.labelB.data, 
	Vision.labelB.m, Vision.labelB.n, color, th_nPostB);
  end

  if (not postB) then 	
    vcm.add_debug_message("No post detected\n")
    return goal; 
  end

  local npost = 0;
  local ivalidB = {};
  local postA = {};
  vcm.add_debug_message(string.format("Checking %d posts\n",#postB));

  for i = 1,#postB do
    local valid = true;
    
    if use_tilted_bbox>0 then
      postStats = Vision.bboxStats(color, postB[i].boundingBox,tiltAngle);
    else
      postStats = Vision.bboxStats(color, postB[i].boundingBox);
    end

    -- size and orientation check
    vcm.add_debug_message(string.format("Area check: %d\n", 
	postStats.area));
    print('poststats area'..postStats.area);
    print('th_min_area'..th_min_area);
    if (postStats.area < th_min_area) then
      vcm.add_debug_message("Area check fail\n");
      valid = false;
    end

    if valid then
      local orientation= postStats.orientation - tiltAngle;
      vcm.add_debug_message(string.format("Orientation check: %.1f\n", 
	 180/math.pi*tiltAngle));
      if (math.abs(orientation) < th_min_orientation) then
        vcm.add_debug_message("orientation check fail\n");
        valid = false;
      end
    end
      
    --fill extent check
    if valid then
      extent = postStats.area / (postStats.axisMajor * postStats.axisMinor);
      vcm.add_debug_message(string.format("Fill extent check: %.2f\n", extent));
      if (extent < th_min_fill_extent) then 
        vcm.add_debug_message("Fill extent check fail\n");
        valid = false; 
      end
    end

    --aspect ratio check
    if valid then
      local aspect = postStats.axisMajor/postStats.axisMinor;
      vcm.add_debug_message(string.format("Aspect check: %d\n",aspect));
      if (aspect < th_aspect_ratio[1]) or 
	(aspect > th_aspect_ratio[2]) then 
        vcm.add_debug_message("Aspect check fail\n");
        valid = false; 
      end
    end

    --check edge margin
    if valid then

      local leftPoint= postStats.centroid[1] - 
	postStats.axisMinor/2 * math.abs(math.cos(tiltAngle));
      local rightPoint= postStats.centroid[1] + 
	postStats.axisMinor/2 * math.abs(math.cos(tiltAngle));

      vcm.add_debug_message(string.format(
	"Left and right point: %d / %d\n", leftPoint, rightPoint));

      local margin = math.min(leftPoint,Vision.labelA.m-rightPoint);

      vcm.add_debug_message(string.format("Edge margin check: %d\n",margin));

      if margin<=th_edge_margin then
        vcm.add_debug_message("Edge margin check fail\n");
        valid = false;
      end

    end

    -- ground check at the bottom of the post
    if valid and check_for_ground>0 then 
      local bboxA = Vision.bboxB2A(postB[i].boundingBox);
      if (bboxA[4] < th_bottom_boundingbox * Vision.labelA.n) then

        -- field bounding box 
        local fieldBBox = {};
        fieldBBox[1] = bboxA[1] + th_ground_boundingbox[1];
        fieldBBox[2] = bboxA[2] + th_ground_boundingbox[2];
        fieldBBox[3] = bboxA[4] + th_ground_boundingbox[3];
        fieldBBox[4] = bboxA[4] + th_ground_boundingbox[4];

        local fieldBBoxStats;
	if use_tilted_bbox>0 then
        -- color stats for the bbox
         fieldBBoxStats = ImageProc.tilted_color_stats(Vision.labelA.data, 
		Vision.labelA.m,Vision.labelA.n,colorField,fieldBBox,tiltAngle);
	else
         fieldBBoxStats = ImageProc.color_stats(Vision.labelA.data, 
		Vision.labelA.m,Vision.labelA.n,colorField,fieldBBox,tiltAngle);
	end
        local fieldBBoxArea = Vision.bboxArea(fieldBBox);

	green_ratio=fieldBBoxStats.area/fieldBBoxArea;
        vcm.add_debug_message(string.format(
		"Green ratio check: %.2f\n",green_ratio));

        -- is there green under the ball?
        if (green_ratio<th_min_green_ratio) then
          vcm.add_debug_message("Green check fail");
          valid = false;
        end
      end
    end

    if valid then
      --bad color check (to check landmarks out)
      local badColorStats=Vision.bboxStats(color2,postB[i].boundingBox,tiltAngle);
      local extent2= badColorStats.area /
          (postStats.axisMajor * postStats.axisMinor);
      vcm.add_debug_message(string.format(
	"Bad color check: %.2f\n", extent2/extent));

      if extent2/extent>th_min_bad_color_ratio then
         vcm.add_debug_message("Bad color check fail\n");
         valid = false; 
      end
    end

--[[
    -- check for posts in the ball
    if (valid and color == colorYellow and Vision.ball.detect == 1) then
      -- is the centroid of the post in the bounding box of the ball?
      local pcent = postStats.centroid;
      --print('pcent '..pcent[1]..', '..pcent[2]);
      --print('ball bbox: '..ball.bboxA[1]..', '..ball.bboxA[2]..
      --','..ball.bboxA[3]..', '..ball.bboxA[4]$

      if ((pcent[1] > ball.bboxA[1] - 10 
	   and pcent[1] < ball.bboxA[2] + 10)
        and 
	  (pcent[1] > ball.bboxA[1] - 10 
	   and pcent[1] <  ball.bboxA[2] + 10)) then
        print('failed outside ball check');
        valid = false;
      end
    en
--]]

    if (valid) then
      ivalidB[#ivalidB + 1] = i;
      npost = npost + 1;
      postA[npost] = postStats;
    end
  end

  vcm.add_debug_message(string.format("Total %d valid posts\n", npost ));

  if ((npost < 1) or (npost > 2)) then 
    vcm.add_debug_message("Post number failure\n");
    return goal; 
  end

  goal.propsB = {};
  goal.propsA = {};
  goal.v = {};

  for i = 1,npost do
    goal.propsB[i] = postB[ivalidB[i]];
    goal.propsA[i] = postA[i];

    scale1 = postA[i].axisMinor / postDiameter;
    scale2 = postA[i].axisMajor / postHeight;
    scale3 = math.sqrt(postA[i].area / (postDiameter*postHeight) );

    if goal.propsB[i].boundingBox[3]<2 then 
      --This post is touching the top, so we shouldn't use the height
      vcm.add_debug_message("Post touching the top\n");
      scale = math.max(scale1,scale3);
    else
      scale = math.max(scale1,scale2,scale3);
    end

    if scale==scale1 then
      vcm.add_debug_message("Post distance measured by width\n");
    elseif scale==scale2 then
      vcm.add_debug_message("Post distance measured by height\n");
    else
      vcm.add_debug_message("Post distance measured by area\n");
    end

    goal.v[i] = HeadTransform.coordinatesA(postA[i].centroid, scale);

    vcm.add_debug_message(string.format("post[%d] = %.2f %.2f %.2f\n",
	 i, goal.v[i][1], goal.v[i][2], goal.v[i][3]));
  end

  if (npost == 2) then
    goal.type = 3; --Two posts

--[[
    -- check for valid separation between posts:
    local dGoal = postA[2].centroid[1]-postA[1].centroid[1];
    local dPost = math.max(postA[1].axisMajor, postA[2].axisMajor);
    local separation=dGoal/dPost;
    vcm.add_debug_message(string.format(
	"Two goal separation:%f\n",separation))
    if (separation<th_goal_separation[1] or 
        separation>th_goal_separation[2]) then
      vcm.add_debug_message("Goal separation check fail\n")
      return goal;
    end
--]]

  else
    goal.v[2] = vector.new({0,0,0,0});

    -- look for crossbar:
    local postWidth = postA[1].axisMinor;

    local leftX = postA[1].boundingBox[1]-5*postWidth;
    local rightX = postA[1].boundingBox[2]+5*postWidth;
    local topY = postA[1].boundingBox[3]-5*postWidth;
    local bottomY = postA[1].boundingBox[3]+5*postWidth;
    local bboxA = {leftX, rightX, topY, bottomY};

    local crossbarStats = ImageProc.color_stats(Vision.labelA.data, Vision.labelA.m, Vision.labelA.n, color, bboxA,tiltAngle);
    local dxCrossbar = crossbarStats.centroid[1] - postA[1].centroid[1];
    local crossbar_ratio = dxCrossbar/postWidth; 

    vcm.add_debug_message(string.format(
	"Crossbar stat: %.2f\n",crossbar_ratio));

    --If the post touches the top, it should be a unknown post
    if goal.propsB[1].boundingBox[3]<3 then --touching the top
      dxCrossbar = 0; --Should be unknown post
    end

    if (math.abs(dxCrossbar) > 0.6*postWidth) then
      if (dxCrossbar > 0) then
	if use_centerpost>0 then
	  goal.type = 1;  -- left post
	else
	  goal.type = 0;  -- unknown post
	end
      else
	if use_centerpost>0 then
	  goal.type = 2;  -- right post
	else
	  goal.type = 0;  -- unknown post
	end
      end
    else
      -- unknown post
      goal.type = 0;
        -- eliminate small posts without cross bars
      vcm.add_debug_message(string.format(
	"Unknown single post size check:%d\n",postA[1].area));
      
      if (postA[1].area < th_min_area_unknown_post) then
        vcm.add_debug_message("Post size too small");
        return goal;
      end
    end
  end
  
-- added for test_vision.m
  if Config.vision.copy_image_to_shm then
      vcm.set_goal_postBoundingBox1(postB[ivalidB[1]].boundingBox);
      vcm.set_goal_postCentroid1({postA[1].centroid[1],postA[1].centroid[2]});
      vcm.set_goal_postAxis1({postA[1].axisMajor,postA[1].axisMinor});
      vcm.set_goal_postOrientation1(postA[1].orientation);
      if npost == 2 then
        vcm.set_goal_postBoundingBox2(postB[ivalidB[2]].boundingBox);
        vcm.set_goal_postCentroid2({postA[2].centroid[1],postA[2].centroid[2]});
        vcm.set_goal_postAxis2({postA[2].axisMajor,postA[2].axisMinor});
        vcm.set_goal_postOrientation2(postA[2].orientation);
      else
        vcm.set_goal_postBoundingBox2({0,0,0,0});
      end
  end

  if goal.type==0 then
    vcm.add_debug_message(string.format("Unknown single post detected\n"));
  elseif goal.type==1 then
    vcm.add_debug_message(string.format("Left post detected\n"));
  elseif goal.type==2 then
    vcm.add_debug_message(string.format("Right post detected\n"));
  elseif goal.type==3 then
    vcm.add_debug_message(string.format("Two posts detected\n"));
  end

  goal.detect = 1;
  return goal;
end
