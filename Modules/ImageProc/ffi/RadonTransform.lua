local RadonTransform = {}

local ok, ffi = pcall(require,'ffi')
assert(ok, 'No ffi!')
local torch = require'torch'
-- Cache
local std, cos, sin = torch.std, math.cos, math.sin
local fabs, min, max, floor = math.abs, math.min, math.max, math.floor

-- These are the integer constants for avoiding floating point precision
-- Max radius: the diagonal of the image
-- NR: number of radii (100 of MAXR with 200 of NR is .5 R precision)
-- TODO: Now just assuming 1 pixel resolution, and just having a constant buffer
-- so that we do not malloc each time
local MAXR, NR = 222

local NTH = 90 -- Number of angles (2 degree res)
--local NTH = 45 -- Number of angles (4 degree res)
--local NTH = 36 -- 5 degree resolution
--local NTH = 180 -- (1 degree res)

local i0, j0, r0, th0 = 0, 0, 0, 0
local flip_center = false

local count_d    = ffi.new("int32_t["..NTH.."]["..MAXR.."]")
local line_sum_d = ffi.new("int32_t["..NTH.."]["..MAXR.."]")
local line_min_d = ffi.new("int32_t["..NTH.."]["..MAXR.."]")
local line_max_d = ffi.new("int32_t["..NTH.."]["..MAXR.."]")
-- TODO: Automatically wrap into torch

-- Save our lookup table discretization
local th_d = ffi.new('double[?]', NTH)
local sin_d, cos_d = ffi.new('double[?]', NTH), ffi.new('double[?]', NTH)
for i = 0, NTH-1 do
  -- We only need 0 to Pi
  -- TODO: Change based on the prior
  th_d[i] = math.pi * i / NTH
  cos_d[i] = cos(th_d[i])
  sin_d[i] = sin(th_d[i])
end

-- Export a bit
local props = {
  count_d = count_d,
  line_sum_d = line_sum_d,
  line_min_d = line_min_d,
  line_max_d = line_max_d,
  cos_d = cos_d,
  sin_d = sin_d,
  th_d = th_d,
  NTH = NTH,
  MAXR = MAXR,
  NR = NR,
  i0 = i0,
  j0 = j0,
  r0 = r0,
}

-- a horizontal pixel could be part of a 45 degree line
-- NOTE: This we can change based on the prior
local DIAGONAL_THRESHOLD = math.sqrt(2) / 2
local sin_index_thresh, cos_index_thresh = {}, {}
for ith = 0, NTH-1 do
  if math.abs(sin_d[ith]) >= DIAGONAL_THRESHOLD then
    table.insert(sin_index_thresh, ith)
  end
  if math.abs(cos_d[ith]) >= DIAGONAL_THRESHOLD then
    table.insert(cos_index_thresh, ith)
  end
end

-- i, j get a center coordinate
local function init(w, h, angle_prior)
  i0 = floor(w / 2)
  j0 = floor(h / 2)
  r0 = floor(math.sqrt(i0 ^ 2 + j0 ^ 2))

  flip_center = angle_prior and angle_prior>45 and angle_prior<135

  -- Resize for the image
  NR = math.ceil(math.sqrt(w * w + h * h))
  -- Update the export
  props.NR = NR
  props.i0 = i0
  props.j0 = j0
  props.r0 = r0
  -- Size of the zeroing
  local b_size32 = NTH * MAXR * ffi.sizeof'int32_t'
  -- Zero the counts
  ffi.fill(count_d, b_size32)
  ffi.fill(line_sum_d, b_size32)
  -- Fill up the min/max lines
  ffi.fill(line_min_d, b_size32, 0x7F)
  ffi.fill(line_max_d, b_size32, 0xFF)
end

-- TODO: Respect the integer method, since since lua converts back to double
-- NOTE: Maybe have two ways - one in double, and one in int

local function addPixelToRay (i, j, ith)
  local s, c = sin_d[ith], cos_d[ith]
  -- Counts and Line statistics
  local ir, iline = fabs(c * i + s * j), -s * i + c * j
  count_d[ith][ir] = count_d[ith][ir] + 1
  line_sum_d[ith][ir] = line_sum_d[ith][ir] + iline
  if iline > line_max_d[ith][ir] then line_max_d[ith][ir] = iline end
  if iline < line_min_d[ith][ir] then line_min_d[ith][ir] = iline end
end

-- Check our centering...
local function addPixelToRay2 (i, j, ith)
  local s, c, ir, iline
  if flip_center then
    ith = ith + NTH / 2
    if ith > NTH then ith = ith - NTH end
    c, s = sin_d[ith], cos_d[ith]
  else
    s, c = sin_d[ith], cos_d[ith]
  end
  ir = r0 + c * (i - i0) + s * (j - j0)
  -- Counts
  count_d[ith][ir] = count_d[ith][ir] + 1
  -- Line statistics
  iline = -s * i + c * j
  line_sum_d[ith][ir] = line_sum_d[ith][ir] + iline
  if iline > line_max_d[ith][ir] then line_max_d[ith][ir] = iline end
  if iline < line_min_d[ith][ir] then line_min_d[ith][ir] = iline end
end

local function addHorizontalPixel (i, j)
  --for _, ith in ipairs(sin_index_thresh) do addPixelToRay(i, j, ith) end
  for _, ith in ipairs(sin_index_thresh) do addPixelToRay2(i, j, ith) end
end

local function addVerticalPixel (i, j)
  --for _, ith in ipairs(cos_index_thresh) do addPixelToRay(i, j, ith) end
  for _, ith in ipairs(cos_index_thresh) do addPixelToRay2(i, j, ith) end
end

function RadonTransform.radon_lines (edge_t, use_horiz, use_vert, bbox, angle_prior)
  -- Use pixel directions
  local j, i, label_nw, label_ne, label_sw, label_se
  -- Take care of noise with a threshold, relating to the standard deviation
  local THRESH = 2 * std(edge_t)
  local x_sz, y_sz = edge_t:size(2), edge_t:size(1)
  -- Loop is -2 since we do not hit the boundary
  local ni, nj = x_sz - 2, y_sz - 2
  -- Start the pointers
  local e_ptr_l = edge_t:data()
  local e_ptr_r = e_ptr_l + x_sz
  -- Clear out any old transform
  init(x_sz, y_sz, angle_prior)
  for j=0, nj do
    for i=0, ni do
      label_nw = e_ptr_l[0]
      e_ptr_l = e_ptr_l + 1
      label_ne = e_ptr_l[0]
      label_sw = e_ptr_r[0]
      e_ptr_r = e_ptr_r + 1
      label_se = e_ptr_r[0]
      -- Strong zero crossings
      -- TODO: Add both j and j+1 (nw and sw pixels are edges, maybe?)
      if use_horiz and fabs(label_nw - label_sw) > THRESH then
        addHorizontalPixel(i, j+.5)
      end
      if use_vert and fabs(label_nw - label_ne) > THRESH then
        addVerticalPixel(i+.5, j)
      end
    end
    -- Must have one more increment to get to the next line
    e_ptr_l = e_ptr_l + 1
    e_ptr_r = e_ptr_r + 1
  end
  -- Yield the Radon Transform
  return props
end

-- Converts to torch
function RadonTransform.get_population ()
  --Int
  local count_t = torch.IntTensor(NTH, NR)
  local count_s = count_t:storage()
  local tmp_s = torch.IntStorage(
  NTH * NR,
  tonumber(ffi.cast("intptr_t",count_d))
  )
  count_s:copy(tmp_s)
  --
  local line_sum_t = torch.IntTensor(NTH, NR)
  local line_sum_s = line_sum_t:storage()
  local tmp_s = torch.IntStorage(
  NTH * NR,
  tonumber(ffi.cast("intptr_t",line_sum_d))
  )
  line_sum_s:copy(tmp_s)
  --
  local line_min_t = torch.IntTensor(NTH, NR)
  local line_min_s = line_min_t:storage()
  local tmp_s = torch.IntStorage(
  NTH * NR,
  tonumber(ffi.cast("intptr_t",line_min_d))
  )
  line_min_s:copy(tmp_s)
  --
  local line_max_t = torch.IntTensor(NTH, NR)
  local line_max_s = line_max_t:storage()
  local tmp_s = torch.IntStorage(
  NTH * NR,
  tonumber(ffi.cast("intptr_t",line_max_d))
  )
  line_max_s:copy(tmp_s)
  return count_t, line_sum_t, line_min_t, line_max_t
end

return RadonTransform
