local RadonTransform = {}

local ok, ffi = pcall(require,'ffi')
assert(ok, 'No ffi!')
ok = nil

-- These are the integer constants for avoiding floating point precision
-- Max radius: the diagonal of the image
-- NR: number of radii (100 of MAXR with 200 of NR is .5 R precision)
local MAXR, NR = 100, 100 

--local NTH = 45 -- Number of angles
--local NTH = 35 -- Number of angles (just over 5 degrees seems ok)
local NTH = 180 -- Let's try it :)

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

-- 2D counts array
local count_d, line_sum_d

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

local BIG = 2147483640

-- TODO: Make smarter? Seems somewhat ok...
function RadonTransform.init (w, h)
  -- Resize for the image
  MAXR = math.ceil(math.sqrt(w*w+h*h))
  NR = MAXR
  count_d = ffi.new("int64_t["..NTH.."]["..NR.."]")
  line_sum_d = ffi.new("int64_t["..NTH.."]["..NR.."]")
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
end
local addPixelToRay = RadonTransform.addPixelToRay

function RadonTransform.addHorizontalPixel (i, j)
  for _,ith in ipairs(sin_index_thresh) do addPixelToRay(i,j,ith) end
end

function RadonTransform.addVerticalPixel (i, j)
  for _,ith in ipairs(cos_index_thresh) do addPixelToRay(i,j,ith) end
end

-- This gives the one best line
function RadonTransform.get_line_stats ()
  -- If no lines
  if countMax == 0 then return end
  -- Find the index
  local iR = rMax * cosTable[thMax]
  local jR = rMax * sinTable[thMax]
  -- Find our counts
  local lMean = lineSum[thMax][rMax] / countMax
  local lMin = lineMin[thMax][rMax]
  local lMax = lineMax[thMax][rMax]
  
  --print('max',thMax, math.floor(rMax+.5) )
  --print('l',lMin,lMean,lMax)
  --print()
  
  -- Return our table of statistics
  return {
    iMean = iR - lMean * sinTable[thMax],
    jMean = jR + lMean * cosTable[thMax],
    iMin  = iR - lMin  * sinTable[thMax],
    jMin  = jR + lMin  * cosTable[thMax],
    iMax  = iR - lMax  * sinTable[thMax],
    jMax  = jR + lMax  * cosTable[thMax],
  }
end

-- Converts to torch
function RadonTransform.get_population ()
  local torch = require'torch'
  local count_t = torch.LongTensor(NTH, NR)
  local count_s = count_t:storage()
  local tmp_s = torch.LongStorage(
  NTH*NR,
  tonumber(ffi.cast("intptr_t",count_d))
  )
  count_s:copy(tmp_s)
  
  local line_sum_t = torch.LongTensor(NTH, NR)
  local line_sum_s = line_sum_t:storage()
  local tmp_s = torch.LongStorage(
  NTH*NR,
  tonumber(ffi.cast("intptr_t",line_sum_d))
  )
  line_sum_s:copy(tmp_s)  
  return count_t, line_sum_t
end

return RadonTransform