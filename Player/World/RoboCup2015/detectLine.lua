local detectObstacle = {}
local ok, ffi = pcall(require, 'ffi')
local ImageProc = require'ImageProc'
local ImageProc2 = require'ImageProc.ffi2'
local T = require'Transform'
local vector = require'vector'
local util = require'util'
local function bboxB2A(bboxB, scaleB)
	return {
		scaleB * bboxB[1],
		scaleB * bboxB[2] + scaleB - 1,
		scaleB * bboxB[3],
		scaleB * bboxB[4] + scaleB - 1,
	}
end
local label_flag, grid_x, grid_y, th_min_area, min_black_fill_rate
local th_aspect_ratio, th_max_height, th_min_height, th_min_orientation
local min_ground_fill_rate
local colors
local config
function detectObstacle.entry(cfg, Image)
  config = cfg
	label_flag = config.label
	grid_x = config.grid_x
	grid_y = config.grid_y
	th_min_area = config.th_min_area
	min_black_fill_rate = config.min_black_fill_rate
	th_aspect_ratio = config.th_aspect_ratio
	th_max_height = config.th_max_height
	th_min_height = config.th_min_height
	th_min_orientation = config.th_min_orientation
	min_ground_fill_rate = config.min_ground_fill_rate
  colors = Image.colors
end
function detectObstacle.update(Image)
  if type(Image)~='table' then
    return false, 'Bad Image'
  end
	local line_cfg = Config.vision.line
  local lines, line_debug = {}, ''

  lines.detect = 0
  local linePropsB = ImageProc.field_lines(labelB_t, line_cfg.max_width,
      line_cfg.connect_th, line_cfg.max_gap, line_cfg.min_length)

  if #linePropsB==0 then
    return 'no linePropsB'
  end

  lines.propsB = linePropsB
  lines.v, lines.endpoint = {},{}
  lines.angle, lines.length= {},{}

  local num_line = 4
  for i = 1,num_line do
    lines.endpoint[i] = vector.zeros(4)
    lines.v[i] = {
      vector.zeros(4),
      vector.zeros(4)
    }
    lines.angle[i] = 0
  end

  local bestindex = 1
  local bestlength = 0
  local linecount = 0

  local length, vendpoint, vHeight = 0, {}, 0
  for i=1, #linePropsB do
    length = math.sqrt(
    	(lines.propsB[i].endpoint[1]-lines.propsB[i].endpoint[2])^2+
    	(lines.propsB[i].endpoint[3]-lines.propsB[i].endpoint[4])^2);

      vendpoint[1] = check_coordinateB(vector.new(
    		{lines.propsB[i].endpoint[1],lines.propsB[i].endpoint[3]}),1);
      vendpoint[2] = check_coordinateB(vector.new(
    		{lines.propsB[i].endpoint[2],lines.propsB[i].endpoint[4]}),1);

    vHeight = 0.5*(vendpoint[1][3]+vendpoint[2][3])

    local vHeightMax = 0.50 --TODO

    --TODO: added debug message
    if length>line_cfg.min_length and linecount<num_line and vHeight<vHeightMax then
      linecount = linecount + 1
      lines.length[linecount] = length
      lines.endpoint[linecount] = vector.new(lines.propsB[i].endpoint)*scaleB
      vendpoint[1] = projectGround(vendpoint[1],0)
      vendpoint[2] = projectGround(vendpoint[2],0)
      lines.v[linecount] = {}
      lines.v[linecount][1] = vendpoint[1]
      lines.v[linecount][2] = vendpoint[2]
      lines.angle[linecount]=math.abs(math.atan2(vendpoint[1][2]-vendpoint[2][2],
			    vendpoint[1][1]-vendpoint[2][1]));
    end
  end

  lines.nLines = linecount
  if lines.nLines>0 then
    lines.detect = 1
    -- print('line v:', unpack(lines.v[1][1]), unpack(lines.v[1][2]))
  end
  return 'blah', lines
end
function detectObstacle.exit()
end
return detectObstacle
