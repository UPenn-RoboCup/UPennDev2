detectLine = {}

local ImageProc = require'ImageProc'
local HeadTransform = require'HeadTransform'
local libVision = require'libVision'
local vector = require'vector'
local util = require'util'
require 'vcm'

colorField = Config.vision.colorField
min_white_pixel = Config.vision.line.min_white_pixel or 200
min_green_pixel = Config.vision.line.min_green_pixel or 5000

max_width = Config.vision.line.max_width or 8
connect_th = Config.vision.line.connect_th or 1.4
max_gap = Config.vision.line.max_gap or 1
min_count = Config.vision.line.min_count or 20
min_length = Config.vision.line.min_length or 10
min_aspect_ratio = Config.vision.line.min_aspect_ratio or 2.5
max_height = Config.vision.line.max_height or 0.3
min_angle_diff = Config.vision.line.min_angle_diff or 15
max_angle_diff = Config.vision.line.max_angle_diff or 70


-- Debug message
local debug_msg
local function add_debug_message(str)
  debug_msg = debug_msg..str
end

function detectLine.detect(lineColor,colorCount,labelA,labelB,HeadTransform,t)
  debug_msg = '### LINE debugging ###\n'
  line = {}
  line.detect = 0
  line_final = {}
  line_final.detect  = 0
  
  -- add_debug_message(string.format(
  --   "Checking white pixel count: %d\n", colorCount[lineColor]))
  if (colorCount[lineColor] < min_white_pixel) then 
    add_debug_message(string.format("FAIL: under %d white pixels\n", min_white_pixel))
    -- print(debug_msg)
    line.debug_msg = debug_msg
    return line
  end
  
  -- TODO: Necessary??
  -- add_debug_message(string.format(
  --   "Checking green pixel count: %d\n", colorCount[colorField]))
  if (colorCount[colorField] < min_green_pixel) then 
    add_debug_message(string.format("FAIL: under %d greeb pixels\n", min_green_pixel))
    -- print(debug_msg)
    line.debug_msg = debug_msg
    return line 
  end

  -- Detect Lines
  linePropsB = ImageProc.field_lines(labelB.data, labelB.m,
		 labelB.n, max_width, connect_th, max_gap, min_count)

  if #linePropsB==0 then 
    add_debug_message("FAIL to detect any lines\n")
    -- print(debug_msg)
    line.debug_msg = debug_msg
    return line 
  end

  line.propsB = linePropsB  -- TODO: may not need
  nLines = #line.propsB

  linecount = 0
  linecount_final = 0

  horizonA = HeadTransform.get_horizonA()
  horizonB = HeadTransform.get_horizonB() 

  -- First round check, check each sigle line
  line.vendpoint={}
  line.endpoint={}
  line.angle={}
  line.length={}
  
  --TODO: check cpp code, is propsB in order??
  add_debug_message(string.format( "%d LINES DETECTED\n", nLines))
  for i=1,math.min(8,nLines) do
    add_debug_message(string.format('**** Checking line %i / %i...\n', i, nLines))
    local propsB = line.propsB[i]
    local valid = true
    
    -- Length check
    -- propsB[i].endpoint = {x1,x2,y1,y2}
    local length = propsB.length
    add_debug_message(string.format('length: %d\n', length))
    if valid then
      if length < min_length then
        valid = false
        add_debug_message("FAIL at line length check\n")
      end
    end
      
    -- Ratio check
    if valid then
      local LWratio = length/propsB.max_width
      add_debug_message(string.format("Length/width ratio: %d\n", LWratio))
      if LWratio < min_aspect_ratio then
        valid = false
        add_debug_message("FAIL at l/w ratio check\n")
      end
    end

    -- Horizon check (in image frame)
    -- if valid then
    --   if line.propsB[i].endpoint[3]>horizonB or 
    --   line.propsB[i].endpoint[4]>horizonB then
    --     valid = false
    --     add_debug_message(string.format("FAIL. Line above horizon\n"))
    --   end
    -- end
    
    -- Ground check (in global frame)
    local vendpoint_old = {}
    if valid then
      vendpoint_old[1] = HeadTransform.coordinatesB(
        vector.new({propsB.endpoint[1], propsB.endpoint[3]}) )
      vendpoint_old[2] = HeadTransform.coordinatesB(
	      vector.new({propsB.endpoint[2], propsB.endpoint[4]}) )
      add_debug_message(string.format('Endpoint heights:%d\t%d\n',
        vendpoint_old[1][3],vendpoint_old[2][3]))
        
      if vendpoint_old[1][3]>max_height or vendpoint_old[2][3]>max_height then
        valid = false
        add_debug_message("FAIL. Line not on ground\n")
      end
    end


    -- X distance check (in global frame)
    -- {x,y,z,1} Homogeneous coordinates
    local vendpoint = {}  
    local lineX
    if valid then
      vendpoint[1] = HeadTransform.projectGround(vendpoint_old[1],0)
      vendpoint[2] = HeadTransform.projectGround(vendpoint_old[2],0)
      lineX = 0.5*(vendpoint[1][1]+vendpoint[2][1])
 
      local goal1 = vcm.get_goal_v1()
      local goal2 = vcm.get_goal_v2()
      local goal_posX = 0
      if (goal1[1] > 0 or goal2[1] > 0) then
        goal_posX = math.max(goal1[1], goal2[1])
      else
        goal_posX = math.min(goal1[1], goal2[1])
      end
      add_debug_message(string.format(
        "Goal posX: %d, LineX: %d\n", goal_posX,lineX))
        
      --TODO: something wrong in lineX  
      -- if goal_posX>0 and goal_posX-lineX<-0.05 then
      --   valid = false
      --   add_debug_message('FAIL. Line behind goal post\n')
      -- end
    end

    if valid then
      linecount = linecount+1
      line.length[linecount]=length
      line.endpoint[linecount]= propsB.endpoint  -- i1,i2,j1,j2
      line.vendpoint[linecount]=vendpoint  -- x,y,z,1
      line.angle[linecount]=math.abs(math.atan2(vendpoint[1][2]-vendpoint[2][2], 
        vendpoint[1][1]-vendpoint[2][1]))
    end
     
  end
  
  add_debug_message(string.format("---- %d valid lines ----\n", linecount))

  -- second round check, check pairs of lines
  local line_valid = vector.ones(linecount)
  for i = 1, linecount do
    for j = i+1, linecount do
      local angle_diff = util.mod_angle(line.angle[i] - line.angle[j])
      angle_diff = math.abs(angle_diff) * 180 / math.pi
      angle_diff = math.min(angle_diff, 180-angle_diff)
      
      local x1 = line.vendpoint[i][1][1]
      local y1 = line.vendpoint[i][1][2]
      local x2 = line.vendpoint[i][2][1]
      local y2 = line.vendpoint[i][2][2]
      local x3 = line.vendpoint[j][1][1]
      local y3 = line.vendpoint[j][1][2]
      local x4 = line.vendpoint[j][2][1]
      local y4 = line.vendpoint[j][2][2]
      
      local Cross = get_crosspoint(x1,y1,x2,y2,x3,y3,x4,y4)
          
      -- In all checks on line pairs, always kill the shorter one. 
      -- Check angle difference
      if line_valid[i]*line_valid[j] ==1 then
        --if (angle_diff > min_angle_diff and angle_diff < max_angle_diff) then
        if angle_diff < min_angle_diff then
          if line.length[i]<line.length[j] then
            line_valid[i] = 0
          else
            line_valid[j] = 0
          end
          add_debug_message(string.format('Line %d and Line %d overlap\n',i,j))
        end        
      end
      
      -- Check if crossed
      if line_valid[i]*line_valid[j] ==1 then
        if ((Cross[1]-x1)*(Cross[1]-x2)<0 and (Cross[1]-x3)*(Cross[1]-x4)<0) then
          if line.length[i]<line.length[j] then
            line_valid[i] = 0
          else
            line_valid[j] = 0
          end
          add_debug_message(string.format('Line %d and Line %d cross\n',i,j))
        end        
      end
      
    end -- end j loop
  end -- end i loop



-- copy the remaining lines in a new array that will be returned.
  line_final.vendpoint={}
  line_final.endpoint={}
  line_final.angle={}
  line_final.length={}

  for i = 1, linecount do
    if (line_valid[i] == 1) then
      linecount_final = linecount_final + 1
      line_final.angle[linecount_final] = line.angle[i]
      line_final.vendpoint[linecount_final] = line.vendpoint[i]
      -- Transform to scaleA
      local endpointA = libVision.bboxB2A(line.endpoint[i])
      line_final.endpoint[linecount_final] = endpointA
      line_final.length[linecount_final] = line.length[i]
    end
  end

  line_final.nLines = linecount_final

  if linecount_final>0 then
    line_final.detect = 1
  end
  
  -- print(debug_msg)
  line_final.debug_msg = debug_msg
  return line_final
end

--get the cross point of two line segements. 
--(x1, y1) (x2, y2) are endpoints for the first line, 
--(x3, y3) (x4, y4) are endpoints for the other line
function get_crosspoint(x1,y1,x2,y2,x3,y3,x4,y4)
  k1 = (y2 - y1)/(x2 - x1)
  k2 = (y4 - y3)/(x4 - x3)
  if (k1 == k2) then
    return {x1,y1}  -- not crossed
  end
  local x = (y3 - y1 + k1*x1 -k2*x3)/(k1 - k2)
  local y = k1*(x - x2) + y2
  return {x,y}
end

return detectLine
