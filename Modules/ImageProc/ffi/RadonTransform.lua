local RadonTransform = {}

local ok, ffi = pcall(require,'ffi')
assert(ok, 'No ffi!')
ok = nil

-- These are the integer constants for avoiding floating point precision
-- Max radius: the diagonal of the image
-- NR: number of radii (100 of MAXR with 200 of NR is .5 R precision)
local MAXR, NR = 100, 100 

--local NTH = 45 -- Number of angles
local NTH = 35 -- Number of angles

-- a horizontal pixel could be part of a 45 degree line
-- NOTE: This we can change based on the prior
local DIAGONAL_THRESHOLD = math.sqrt(2) / 2

-- Save our lookup table discretization
local th = torch.DoubleTensor(NTH)
local sinTable, cosTable = torch.DoubleTensor(NTH), torch.DoubleTensor(NTH)

-- Keep track of the counts
local countMax, thMax, rMax = 0
local count, lineMin, lineMax, lineSum

local BIG = 2147483640

-- Clear the Radon transform
function RadonTransform.clear ()
  -- Clear the transform
  count:zero()
  lineMin:fill(BIG)
  lineMax:fill(-BIG)
  lineSum:zero()
  countMax = 0
end

-- Initialize the lookup table
function RadonTransform.init (w, h)
  -- TODO: Just use apply
  for i = 1, NTH do
    -- We only need 0 to Pi
    -- TODO: Change based on the prior
    th[i] = math.pi * i / NTH
    cosTable[i] = math.cos(th[i])
    sinTable[i] = math.sin(th[i])
  end
  -- Resize for the image
  MAXR = math.ceil(math.sqrt(w*w+h*h))
  NR = MAXR
  count = torch.LongTensor(NTH, NR)
  lineMin = torch.LongTensor(NTH, NR)
  lineMax = torch.LongTensor(NTH, NR)
  lineSum = torch.LongTensor(NTH, NR)
  -- Do the clear initially
  RadonTransform.clear()
end

-- TODO: Respect the integer method, since since lua converts back to double
-- NOTE: Maybe have two ways - one in double, and one in int
function RadonTransform.addPixelToRay (j, i, ith)
  
  ------------
  -- While not in ffi land! i-1 and j-1
  ------------
  -- TODO: Use FFI math, like fabs, etc.
  local ir = math.abs(cosTable[ith] * i + sinTable[ith] * j)
  -- R value: 0 to MAXR-1
  -- R index: 0 to NR-1
  --local ir1 = math.floor(math.max(1,math.min(ir, MAXR))+.5)
  local ir1 = math.floor(ir) % MAXR + 1
  --local ir1 = math.max(1,math.min(ir, MAXR))
  count[ith][ir1] = count[ith][ir1] + 1
  if count[ith][ir1] > countMax then
    thMax = ith
    rMax  = ir1
    countMax = count[ith][ir1]
  end

  -- Line statistics:
  local iline = -sinTable[ith] * i + cosTable[ith] * j
  lineSum[ith][ir1] = lineSum[ith][ir1] + iline
  if iline > lineMax[ith][ir1] then
    lineMax[ith][ir1] = iline
  end
  if iline < lineMin[ith][ir1] then
    lineMin[ith][ir1] = iline
  end
  
end

function RadonTransform.addHorizontalPixel (i, j)
  ------------
  -- While not in ffi land!
  i, j = i+1, j+1
  ------------
  for ith = 1, NTH do
    if math.abs(sinTable[ith]) >= DIAGONAL_THRESHOLD then
      RadonTransform.addPixelToRay(i, j, ith)
    end
  end
end

function RadonTransform.addVerticalPixel (i, j)
  ------------
  -- While not in ffi land!
  i, j = i+1, j+1
  ------------
  for ith = 1, NTH do
    if math.abs(cosTable[ith]) >= DIAGONAL_THRESHOLD then
      RadonTransform.addPixelToRay(i, j, ith)
    end
  end
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
  
  print('max',thMax, math.floor(rMax+.5) )
  print('l',lMin,lMean,lMax)
  print()
  
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

function RadonTransform.get_population ()
  return count, lineSum, lineMin, lineMax
end

return RadonTransform