local RadonTransform = {}

-- These are the integer constants for avoiding floating point precision
-- Max r for 80*60 label
local MAXR = 100 -- Max radius
local NR = 100 -- Number of radii
local NTH = 45 -- Number of angles
local NTRIG = 65536 -- Integer trig normalization
-- a horizontal pixel could be part of a 45 degree line
-- NOTE: This we can change based on the prior
local DIAGONAL_THRESHOLD = NTRIG / math.sqrt(2)

-- Keep track of the counts
local countMax, thMax, rMax = 0
local count   = torch.IntTensor(NTH, NR)
local lineMin = torch.IntTensor(NTH, NR)
local lineMax = torch.IntTensor(NTH, NR)
local lineSum = torch.IntTensor(NTH, NR)

-- Save our lookup table discretization
local th = torch.DoubleTensor(NTH)
local sinTable, cosTable = torch.IntTensor(NTH), torch.IntTensor(NTH)

-- TODO: Define INT_MAX better
local INT_MAX = 4294967296
local INT_MIN = -4294967296
local PI = math.pi

-- Clear the Radon transform
function RadonTransform.clear ()
  -- Clear the transform
  count:zero()
  lineMin:fill(INT_MAX)
  lineMax:fill(INT_MIN)
  lineSum:zero()
  countMax = 0
end

-- Initialize the lookup table
function RadonTransform.init ()
  -- TODO: Just use apply
  for i = 1, NTH do
    -- We only need 0 to Pi
    -- TODO: Change based on the prior
    th[i] = PI * i / NTH
    cosTable[i] = NTRIG * math.cos(th[i])
    sinTable[i] = NTRIG * math.sin(th[i])
  end
  -- Do the clear initially
  RadonTransform.clear()
end

-- TODO: Respect the integer method, since since lua converts back to double
-- NOTE: Maybe have two ways - one in double, and one in int
function RadonTransform.addPixelToRay (i, j, ith)
  
  -- TODO: Use FFI math, like fabs, etc.
  local ir = math.abs(cosTable[ith] * i + sinTable[ith] * j) / NTRIG
  -- R value: 0 to MAXR-1
  -- R index: 0 to NR-1
  local ir1 = (ir + 1) * NR / MAXR - 1
  count[ith][ir1] = count[ith][ir1] + 1
  if count[ith][ir1] > countMax then
    thMax = ith
    rMax  = ir1
    countMax = count[ith][ir1]
  end

  -- Line statistics:
  local iline = (-sinTable[ith] * i + cosTable[ith] * j) / NTRIG
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
  local iR = ((rMax+1)*MAXR/NR-1+.5)*cosTable[thMax]
  local jR = ((rMax+1)*MAXR/NR-1+.5)*sinTable[thMax]
  -- Find our counts
  local lMean = lineSum[thMax][rMax] / countMax
  local lMin = lineMin[thMax][rMax]
  local lMax = lineMax[thMax][rMax]
  
  -- Return our table of statistics
  return {
    iMean = (iR - lMean*sinTable[thMax])/NTRIG,
    jMean = (jR + lMean*cosTable[thMax])/NTRIG,
    iMin = (iR - lMin*sinTable[thMax])/NTRIG,
    jMin = (jR + lMin*cosTable[thMax])/NTRIG,
    iMax = (iR - lMax*sinTable[thMax])/NTRIG,
    jMax = (jR + lMax*cosTable[thMax])/NTRIG,
  }
end

return RadonTransform