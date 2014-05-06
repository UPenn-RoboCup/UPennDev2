local RadonTransform = {}

local ok, ffi = pcall(require,'ffi')
assert(ok, 'No ffi!')
ok = nil

-- These are the integer constants for avoiding floating point precision
-- Max radius: the diagonal of the image
-- NR: number of radii (100 of MAXR with 200 of NR is .5 R precision)
-- TODO: Now just assuming 1 pixel resolution, and just having a constant buffer
-- so that we do not malloc each time
local MAXR, NR = 101

--local NTH = 45 -- Number of angles
local NTH = 36 -- 5 degree resolution
--local NTH = 180 -- Let's try it :)

local count_d = ffi.new("int64_t["..NTH.."]["..MAXR.."]")
local line_sum_d = ffi.new("int64_t["..NTH.."]["..MAXR.."]")
local line_min_d = ffi.new("int64_t["..NTH.."]["..MAXR.."]")
local line_max_d = ffi.new("int64_t["..NTH.."]["..MAXR.."]")
local b_size = NTH*MAXR*ffi.sizeof('int64_t')

-- Save our lookup table discretization
local th = ffi.new('double[?]',NTH)
local sin_d, cos_d = ffi.new('double[?]',NTH), ffi.new('double[?]',NTH)
for i = 0, NTH-1 do
  -- We only need 0 to Pi
  -- TODO: Change based on the prior
  th[i] = math.pi * i / NTH
  cos_d[i] = math.cos(th[i])
  sin_d[i] = math.sin(th[i])
end

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

-- TODO: Make smarter? Seems somewhat ok...
function RadonTransform.init (w, h)
  -- Resize for the image
  NR = math.ceil(math.sqrt(w*w+h*h))
  -- Zero the counts
  ffi.fill(count_d, b_size)
  ffi.fill(line_sum_d, b_size)
  -- Fill up the min/max lines
  ffi.fill(line_min_d, b_size, 0x7F)
  ffi.fill(line_max_d, b_size, 0xFF)
  --
  countMax = 0
end

-- TODO: Respect the integer method, since since lua converts back to double
-- NOTE: Maybe have two ways - one in double, and one in int

local fabs, min, max, floor = math.abs, math.min, math.max, math.floor

function RadonTransform.addPixelToRay (i, j, ith)
  local s, c = sin_d[ith], cos_d[ith]
  -- Counts
  local ir = fabs(c * i + s * j)
  count_d[ith][ir] = count_d[ith][ir] + 1
  -- Line statistics
  local iline = -s * i + c * j
  line_sum_d[ith][ir] = line_sum_d[ith][ir] + iline
  --print(iline,line_max_d[ith][ir],line_min_d[ith][ir])
  if iline > line_max_d[ith][ir] then line_max_d[ith][ir] = iline end
  if iline < line_min_d[ith][ir] then line_min_d[ith][ir] = iline end
end
local addPixelToRay = RadonTransform.addPixelToRay

function RadonTransform.addHorizontalPixel (i, j)
  for _,ith in ipairs(sin_index_thresh) do addPixelToRay(i,j,ith) end
end

function RadonTransform.addVerticalPixel (i, j)
  for _,ith in ipairs(cos_index_thresh) do addPixelToRay(i,j,ith) end
end

-- Find parallel lines in the Radon space
function RadonTransform.get_parallel_lines (threshold, min_width)
  -- Threshold should be 75% of the maxR maybe? We can deal with this later
  threshold = threshold or 60
  min_width = min_width or 7
  local parallel_lines, found = {}, false
  local i_monotonic_max, monotonic_max, val
  for ith=0, NTH-1 do
    local i_arr = {}
    i_monotonic_max = 0
    monotonic_max = 0
    for ir=0, NR-1 do
      val = count_d[ith][ir]
      if val>threshold and val>monotonic_max then
        monotonic_max = val
        i_monotonic_max = ir
      elseif ir>i_monotonic_max+min_width and monotonic_max>0 then
        table.insert(i_arr, i_monotonic_max)
        i_monotonic_max = ir
        monotonic_max = -1
      end
    end
    -- Save the parallel lines
    if #i_arr>1 then
      found = true
      parallel_lines[ith] = i_arr
    end
  end
  -- Yield the parallel lines
  if not found then return end
  
  local lines = {}
  for ith, rs in pairs(parallel_lines) do
    for _, i_r in pairs(rs) do
      local s, c = sin_d[ith], cos_d[ith]
      -- Find the image index
      local iR = i_r * c
      local jR = i_r * s
      --
      local lMean = tonumber(line_sum_d[ith][i_r] / count_d[ith][i_r])
      local lMin = tonumber(line_min_d[ith][i_r])
      local lMax = tonumber(line_max_d[ith][i_r])
      --print(iR, jR, lMax,lMin,lMean)
      --
      table.insert(lines, {
        ith = ith,
        ir = i_r,
        iMean = iR - lMean * s,
        jMean = jR + lMean * c,
        iMin = iR - lMin * s,
        jMin = jR + lMin * c,
        iMax = iR - lMax * s,
        jMax = jR + lMax * c,
      })
    end
  end
  
  -- Return our table of statistics
  return lines
  
end

-- Converts to torch
function RadonTransform.get_population ()
  local torch = require'torch'
  --
  local count_t = torch.LongTensor(NTH, NR)
  local count_s = count_t:storage()
  local tmp_s = torch.LongStorage(
  NTH*NR,
  tonumber(ffi.cast("intptr_t",count_d))
  )
  count_s:copy(tmp_s)
  --
  local line_sum_t = torch.LongTensor(NTH, NR)
  local line_sum_s = line_sum_t:storage()
  local tmp_s = torch.LongStorage(
  NTH*NR,
  tonumber(ffi.cast("intptr_t",line_sum_d))
  )
  line_sum_s:copy(tmp_s)
  --
  local line_min_t = torch.LongTensor(NTH, NR)
  local line_min_s = line_min_t:storage()
  local tmp_s = torch.LongStorage(
  NTH*NR,
  tonumber(ffi.cast("intptr_t",line_min_d))
  )
  line_min_s:copy(tmp_s)
  --
  local line_max_t = torch.LongTensor(NTH, NR)
  local line_max_s = line_max_t:storage()
  local tmp_s = torch.LongStorage(
  NTH*NR,
  tonumber(ffi.cast("intptr_t",line_max_d))
  )
  line_max_s:copy(tmp_s)
  return count_t, line_sum_t, line_min_t, line_max_t
end

return RadonTransform