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

--Use center post to determine post type (disabled for OP)
use_centerpost=Config.vision.goal.use_centerpost or 0;
--Cut top portion of detected post (for OP)
cut_top_post = Config.vision.goal.cut_top_post or 0;
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
th_min_areaB = Config.vision.goal.th_min_areaB;
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
  local postB = ImageProc.goal_posts(Vision.labelB.data, 
	Vision.labelB.m, Vision.labelB.n, color, th_nPostB);
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
    postStats = Vision.bboxStats(color, postB[i].boundingBox);

   --cut top region of the post off for OP
    if cut_top_post ==1 then 
      local leftX = postB[i].boundingBox[1];
      local rightX = postB[i].boundingBox[2];
      local topY = postB[i].boundingBox[3];
      local bottomY = postB[i].boundingBox[4];
      local topY2 = topY-(topY-bottomY)*0.4;
      boundingBoxLower={leftX,rightX,topY2,bottomY};
      postStats2 = Vision.bboxStats(color, boundingBoxLower);

      --TODO: Compare thickness
      --[[
      local thickness1 = postStats.axisMinor;
      local thickness2 = postStats2.axisMinor;
      if thickness1 > 1.1* thickness2 then
	postStats=postStats2;
      end
      --]]

      postStats=postStats2;
    end

    -- size and orientation check
    vcm.add_debug_message(string.format("Size and orientation check: \narea %d orientation %d\n", 
	postStats.area, 180/math.pi*postStats.orientation));

    if (postStats.area < th_min_areaB) then
      vcm.add_debug_message("size check fail\n");
      valid = false;
    end

    if (math.abs(postStats.orientation) < th_min_orientation) then
      vcm.add_debug_message("orientation check fail\n");
      valid = false;
    end
      
    --fill extent check
    local extent = postStats.area / (postStats.axisMajor * postStats.axisMinor);
    vcm.add_debug_message(string.format("Fill extent check: %d\n", extent));
    if (extent < th_min_fill_extent) then 
      vcm.add_debug_message("Fill extent check fail\n");
      valid = false; 
    end

    --aspect ratio check
    local aspect = postStats.axisMajor/postStats.axisMinor;
    vcm.add_debug_message(string.format("Aspect check: %d\n",aspect));
    if (aspect < th_aspect_ratio[1]) or 
	(aspect > th_aspect_ratio[2]) then 
      vcm.add_debug_message("Aspect check fail\n");
      valid = false; 
    end

    --check edge margin
    local margin=math.min(postStats.centroid[1], 
			  Vision.labelA.m-postStats.centroid[1]);
    vcm.add_debug_message(string.format("Edge margin check: %d\n",margin));

    if margin<=th_edge_margin then
      vcm.add_debug_message("Edge margin check fail\n");
      valid = false;
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

        -- color stats for the bbox
        local fieldBBoxStats = ImageProc.color_stats(Vision.labelA.data, 
		Vision.labelA.m,Vision.labelA.n,colorField,fieldBBox);
        local fieldBBoxArea = Vision.bboxArea(fieldBBox);

	green_ratio=fieldBBoxStats.area/fieldBBoxArea;
        vcm.add_debug_message(string.format(
		"Green ratio check: %d\n",green_ratio));

        -- is there green under the ball?
        if (green_ratio<th_min_green_ratio) then
          vcm.add_debug_message("Green check fail");
          valid = false;
        end
      end
    end

    if valid then
      --bad color check (to check landmarks out)
      local badColorStats=Vision.bboxStats(color2,postB[i].boundingBox);
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

    scale = math.max(postA[i].axisMinor / postDiameter,
                      postA[i].axisMajor / postHeight,
                      math.sqrt(postA[i].area / (postDiameter*postHeight)));

    goal.v[i] = HeadTransform.coordinatesA(postA[i].centroid, scale);

    vcm.add_debug_message(string.format("post[%d] = %.2f %.2f %.2f\n",
	 i, goal.v[i][1], goal.v[i][2], goal.v[i][3]));
  end

  if (npost == 2) then
    goal.type = 3; --Two posts

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

  else
    goal.v[2] = vector.new({0,0,0,0});

    -- look for crossbar:
    local postWidth = postA[1].axisMinor;
    local leftX = postA[1].boundingBox[1]-5*postWidth;
    local rightX = postA[1].boundingBox[2]+5*postWidth;
    local topY = postA[1].boundingBox[3]-postWidth;
    local bottomY = postA[1].boundingBox[3]+2*postWidth;
    local bboxA = {leftX, rightX, topY, bottomY};

    local crossbarStats = ImageProc.color_stats(Vision.labelA.data, Vision.labelA.m, Vision.labelA.n, color, bboxA);
    local dxCrossbar = crossbarStats.centroid[1] - postA[1].centroid[1];
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

      if (postA[1].area < th_min_area_unknown_post) then
        vcm.add_debug_message("Post size too small");
        return goal;
      end
    end
  end
  
-- added for test_vision.m
  if Config.vision.copy_image_to_shm then
--    vcm.set_goal_postBoundingBox1(postB[ivalidB[1]].boundingBox);
      --added 
      vcm.set_goal_postBoundingBox1(vector.zeros(4));
      vcm.set_goal_postCentroid1({postA[1].centroid[1],postA[1].centroid[2]});
      vcm.set_goal_postAxis1({postA[1].axisMajor,postA[1].axisMinor});
      vcm.set_goal_postOrientation1(postA[1].orientation);

      if npost == 2 then
--      vcm.set_goal_postBoundingBox2({postB[ivalidB[2]].boundingBox});
        vcm.set_goal_postBoundingBox2(vector.zeros(4));
        vcm.set_goal_postCentroid2({postA[2].centroid[1],postA[2].centroid[2]});
        vcm.set_goal_postAxis2({postA[2].axisMajor,postA[2].axisMinor});
        vcm.set_goal_postOrientation2(postA[2].orientation);
      else
        vcm.set_goal_postBoundingBox2(vector.zeros(4));
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
