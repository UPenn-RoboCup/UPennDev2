-- libDetect
-- (c) 2014 Stephen McGill
-- General Detection methods
local libVision = {}
local torch  = require'torch'
local bit    = require'bit'
local lshift = bit.lshift
local rshift = bit.rshift
local bor = bit.bor
local band = bit.band
local ImageProc = require'ImageProc'
local T = require'libTransform'

local w, h, wa, ha, wb, hb
-- Form the labelA and labelB tensors
local labelA_t, labelB_t, lut_t, cc_t
-- torch raw data
local lut_d, lA_d, lB_d, cc_d
-- Scale to support legacy
local scaleA, scaleB = 2, 2
-- Keep the Head Transform updated
local trHead = T.eye()
-- Image center, as float
local x0A, y0A, focalA

function libVision.get_labels()
  return labelA_t, labelB_t
end

function libVision.setup (w0, h0)
  -- Take in the width and height of the original YUYV image
  w, h = w0, h0
  wa, ha = w/scaleA, h/scaleA
  wb, hb = wa/scaleB, ha/scaleB
  labelA_t, labelB_t = torch.ByteTensor(ha, wa), torch.ByteTensor(hb, wb)
  -- Raw data access
  lA_d, lB_d = labelA_t:data(), labelB_t:data()
  -- Color count tensor
  cc_t = torch.IntTensor(256):zero()
  cc_d = cc_t:data()
  --
  local focal_length, focal_base = 545.6, 640
  x0A, y0A = 0.5 * (labelA_t:size(2) - 1), 0.5 * (labelA_t:size(1) - 1)
  focalA = focal_length / (focal_base / labelA_t:size(2))
end

function libVision.load_lut (fname)
  local f_lut = torch.DiskFile( fname , 'r')
  f_lut.binary(f_lut)
  -- We know the size of the LUT
  local lut_s = f_lut:readByte(262144)
  f_lut:close()
  -- Form a tensor for us
  lut_t = torch.ByteTensor(lut_s)
  -- Raw data
  lut_d = lut_t:data()
end

-- Take in a pointer (or string) to the image
-- Return nothing, since given the tensors
function libVision.yuyv_to_labelA (yuyv_ptr)
  -- The yuyv pointer changes each time
  -- Cast the lightuserdata to cdata
  local yuyv_d = ffi.cast("uint32_t*", yuyv_ptr)
  -- Reset the color count
  cc_t:zero()
  -- Temporary variables for the loop
  -- NOTE: 4 bytes yields 2 pixels, so stride of (4/2)*w
  local a_ptr, yuyv_ptr, stride, index, cdt = lA_d, yuyv_d, w / 2
  for j=0,ha-1 do
    for i=0,wa-1 do
      index = bor(
        rshift(band(yuyv_ptr[0], 0xFC000000), 26),
        rshift(band(yuyv_ptr[0], 0x0000FC00), 4),
        lshift(band(yuyv_ptr[0], 0xFC), 10)
      )
      cdt = lut_d[index]
      --if cdt~=0 then print('cdt',cdt,index) end
      -- Increment the color count
      cc_d[cdt] = cc_d[cdt] + 1
      -- Move the labelA pointer
      a_ptr[0] = cdt
      a_ptr = a_ptr + 1
      -- Move the image pointer
      yuyv_ptr = yuyv_ptr + 1
    end
    -- stride to next
    yuyv_ptr = yuyv_ptr + stride
  end
  --
end

-- Bit OR on blocks of 2x2 to get to labelB
-- NOTE: could do 4x4 blocks, too, if we add a parameter
function libVision.form_labelB()
  -- Zero the downsampled image
  labelB_t:zero()
  -- Assumes a subsample of 2 only
  local a_ptr, b_ptr = lA_d, lB_d
  -- Offset a row
  local a_ptr1 = a_ptr + wa
  -- Start the loop
  for jb=0,hb-1 do
    for ib=0,wb-1 do
      b_ptr[0] = bor(a_ptr[0],a_ptr[1],a_ptr1[0],a_ptr1[1])
      -- Move to the next pixel
      a_ptr = a_ptr + 2
      a_ptr1 = a_ptr1 + 2
      -- Move b
      b_ptr = b_ptr + 1
    end
    -- Move another row, too
    a_ptr = a_ptr + wa
    a_ptr1 = a_ptr1 + wa
  end
end

-- Simple bbox with no tilted color stats
-- TODO: Use the FFI for color stats, should be super fast
local function bboxStats (color, bboxB)
  local bboxA = {
    scaleB * bboxB[1],
    scaleB * bboxB[2] + scaleB - 1,
    scaleB * bboxB[3],
    scaleB * bboxB[4] + scaleB - 1
  }
  local area = (bboxA[2] - bboxA[1] + 1) * (bboxA[4] - bboxA[3] + 1)
  return ImageProc.color_stats(labelA_t, color, bboxA), area
end

local function check_prop (prop, th_area, th_fill)
  -- Grab the statistics in labelA
  local stats, box_area = bboxStats(1, prop.boundingBox);
  local area = stats.area
  -- If no pixels then return
  if area < th_area then return'Area' end
  -- Get the fill rate
  local fill_rate = area / box_area
  if fill_rate < th_fill then return'Fill rate' end
  return stats
end

local function check_coordinate(centroid, scale, maxD, maxH)
  local v = torch.Tensor({
    focalA,
    -(centroid[1] - x0A),
    -(centroid[2] - y0A),
    scale,
  })
  v = trHead * v / v[4]
  -- Check the distance
  if v[1]*v[1] + v[2]*v[2] > maxD*maxD then
    return"Distance"
  elseif v[3] > maxH then
    return"Height"
  end
  return v
end

local util = require'util'
local b_diameter = 0.065
function libVision.ball()
  -- The ball is color 1
  local cc = cc_d[1]
  if cc<6 then return'Color count' end
  -- Connect the regions in labelB
  local ballPropsB = ImageProc.connected_regions(labelB_t, 1)
  local nProps = #ballPropsB
  if nProps==0 then return'Connected regions' end
  --
  for i=1,math.min(5,nProps) do
    -- Check the image properties
    local propsB = ballPropsB[i]
    local propsA = check_prop(propsB, 4, 0.35)
    if type(propsA)=='string' then return string.format("Failed %s", propsA) end
    -- Check the coordinate on the field
    local dArea = math.sqrt((4/math.pi) * propsA.area)
    local scale = math.max(dArea/b_diameter, propsA.axisMajor/b_diameter);
    local v = check_coordinate(propsA.centroid, scale, 5.0, 0.20)
    if type(propsA)=='string' then return string.format("Failed %s", v) end
    -- TODO: Check if outside the field
    -- TODO: Ground color check
    return propsA.centroid, v
  end
  --
end

return libVision
