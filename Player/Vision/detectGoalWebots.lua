module(..., package.seeall);

require('Config');	-- For Ball and Goal Size
require('ImageProc');
require('HeadTransform');	-- For Projection
require('Body')
require('Vision');

-- Dependency
require('Detection');

-- Define Color
colorOrange = 1;
colorYellow = 2;
colorCyan = 4;
colorField = 8;
colorWhite = 16;

use_point_goal=Config.vision.use_point_goal;
--Use center post to determine post type (disabled for OP)
use_centerpost=Config.vision.use_centerpost or 0;
--Cut top portion of detected post (for OP)
cut_top_post = Config.vision.cut_top_post or 0;

function detect(color,color2)
  if color==colorYellow then vcm.add_debug_message("\nGoal: Yellow post check\n")
  else vcm.add_debug_message("\nGoal: Blue post check\n"); end
  local goal = {};
  goal.detect = 0;

  local postDiameter = 0.10;
  local postHeight = Config.world.goalHeight or 0.80;
  local goalWidth = Config.world.goalWidth or 1.40;
  local nPostB = 3;  -- appropriate for scaleB = 4
  local postB = ImageProc.goal_posts(Vision.labelB.data, Vision.labelB.m, Vision.labelB.n, color, nPostB);
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

    if cut_top_post ==1 then --cut top of the post for OP
      local leftX = postB[i].boundingBox[1];
      local rightX = postB[i].boundingBox[2];
      local topY = postB[i].boundingBox[3];
      local bottomY = postB[i].boundingBox[4];
      local topY2 = topY-(topY-bottomY)*0.4;
      boundingBoxLower={leftX,rightX,topY2,bottomY};
      postStats2 = Vision.bboxStats(color, boundingBoxLower);

      --Compare thickness
      --[[
      local thickness1 = postStats.axisMinor;
      local thickness2 = postStats2.axisMinor;
      if thickness1 > 1.1* thickness2 then
	postStats=postStats2;
      end
      --]]

      postStats=postStats2;
    end
    -- size and orientation
    vcm.add_debug_message(string.format("Size and orientation check: \narea %d orientation %d\n", 
	postStats.area, 180/math.pi*postStats.orientation));
    if (math.abs(postStats.orientation) < 60*math.pi/180) then
      vcm.add_debug_message("orientation check fail\n");
      valid = false;
    end
      
    --fill extent check
    local extent = postStats.area / (postStats.axisMajor * postStats.axisMinor);
    vcm.add_debug_message(string.format("Fill extent check: %d\n", extent));

    --bad color check (to check landmarks out)
    local badColorStats=Vision.bboxStats(color2,postB[i].boundingBox);
    local extent2= badColorStats.area /
          (postStats.axisMajor * postStats.axisMinor);
    vcm.add_debug_message(string.format("Bad color check: %.2f\n", extent2/extent));

    if extent2/extent>0.1 then
       vcm.add_debug_message("Bad color check fail\n");
       valid = false; 
    end

    --aspect ratio check
    local aspect = postStats.axisMajor/postStats.axisMinor;
    vcm.add_debug_message(string.format("Aspect ratio check: %f\n", aspect ));

    if ((aspect < 2.5) or (aspect > 15)) then 
       vcm.add_debug_message("Aspect check fail\n");
--     valid = false; 
    end

    -- ground check
    -- is post at the bottom
    local bboxA = Vision.bboxB2A(postB[i].boundingBox);
    if (valid) then
      if (bboxA[4] < 0.9 * Vision.labelA.n) then

        -- field bounding box 
        local fieldBBox = {};
        fieldBBox[1] = bboxA[1] - 15;
        fieldBBox[2] = bboxA[2] + 15;
        fieldBBox[3] = bboxA[4] - 15;
        fieldBBox[4] = bboxA[4] + 10;

        -- color stats for the bbox
        local fieldBBoxStats = ImageProc.color_stats(Vision.labelA.data, Vision.labelA.m, Vision.labelA.n, colorField, fieldBBox);
        local fieldBBoxArea = Vision.bboxArea(fieldBBox);

 --       vcm.add_debug_message(string.format("Ground Check %f\n", aspect ));
        --print('field check: area: '..fieldBBoxArea..' bbox: '..fieldBBoxStats.area);
        -- is there green under the ball?
      end
    end

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
    local dgoal = postA[2].centroid[1]-postA[1].centroid[1];
    local dpost = math.max(postA[1].axisMajor, postA[2].axisMajor);
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
      if (postA[1].area < 50) then
        vcm.add_debug_message("Post size too small");
        return goal;
      end
    end
  end
  
-- added for test_vision.m
  if Config.vision.copy_image_to_shm then
--      vcm.set_goal_postBoundingBox1(postB[ivalidB[1]].boundingBox);
      --added 
      vcm.set_goal_postBoundingBox1(vector.zeros(4));

      vcm.set_goal_postCentroid1({postA[1].centroid[1],postA[1].centroid[2]});
      vcm.set_goal_postAxis1({postA[1].axisMajor,postA[1].axisMinor});
      vcm.set_goal_postOrientation1(postA[1].orientation);

      if npost == 2 then
--        vcm.set_goal_postBoundingBox2({postB[ivalidB[2]].boundingBox});
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
