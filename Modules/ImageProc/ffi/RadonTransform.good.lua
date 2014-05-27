local RadonTransform = {}

local ok, ffi = pcall(require,'ffi')
assert(ok, 'No ffi!')
local torch = require'torch'

-- These are the integer constants for avoiding floating point precision
-- Max radius: the diagonal of the image
-- NR: number of radii (100 of MAXR with 200 of NR is .5 R precision)
-- TODO: Now just assuming 1 pixel resolution, and just having a constant buffer
-- so that we do not malloc each time
local MAXR, NR = 222

--local NTH = 90 -- Number of angles (2 degree res)
local NTH = 45 -- Number of angles (4 degree res)
--local NTH = 36 -- 5 degree resolution
--local NTH = 180 -- (1 degree res)

local count_d    = ffi.new("int32_t["..NTH.."]["..MAXR.."]")
local line_sum_d = ffi.new("int32_t["..NTH.."]["..MAXR.."]")
local line_min_d = ffi.new("int32_t["..NTH.."]["..MAXR.."]")
local line_max_d = ffi.new("int32_t["..NTH.."]["..MAXR.."]")

-- Export a bit
RadonTransform.count_d = count_d
RadonTransform.NTH = NTH
RadonTransform.NR = NR

-- Save our lookup table discretization
local th = ffi.new('double[?]', NTH)
local sin_d, cos_d = ffi.new('double[?]', NTH), ffi.new('double[?]', NTH)
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
  -- Update the export
  RadonTransform.NR = NR
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

local fabs, min, max, floor = math.abs, math.min, math.max, math.floor

function RadonTransform.addPixelToRay (i, j, ith)
  local s, c = sin_d[ith], cos_d[ith]
  -- Counts
  local ir = fabs(c * i + s * j)
  count_d[ith][ir] = count_d[ith][ir] + 1
  -- Line statistics
  local iline = -s * i + c * j
  line_sum_d[ith][ir] = line_sum_d[ith][ir] + iline
  if iline > line_max_d[ith][ir] then line_max_d[ith][ir] = iline end
  if iline < line_min_d[ith][ir] then line_min_d[ith][ir] = iline end
end
local addPixelToRay = RadonTransform.addPixelToRay

function RadonTransform.addHorizontalPixel (i, j)
  for _, ith in ipairs(sin_index_thresh) do addPixelToRay(i, j, ith) end
end

function RadonTransform.addVerticalPixel (i, j)
  for _, ith in ipairs(cos_index_thresh) do addPixelToRay(i, j, ith) end
end

-- Find parallel lines in the Radon space
function RadonTransform.get_parallel_lines (min_width)
  -- Have a minimum width of the line (in pixel space)
  min_width = min_width or 4
  local i_monotonic_max, monotonic_max, val

  local ithMax, irMax1, irMax2
  local cntMax1, cntMax2 = 0, 0
  local found = false

  for ith=0, NTH-1 do
    i_monotonic_max = 0
    monotonic_max = 0
    local i_arr, c_arr = {}, {}
    for ir=0, NR-1 do
      val = count_d[ith][ir]
      if val > monotonic_max then
        monotonic_max = val
        i_monotonic_max = ir
      else
        -- End of monotonicity
        if #c_arr<2 then
          table.insert(i_arr, i_monotonic_max)
          table.insert(c_arr, monotonic_max)
          i_monotonic_max = ir
          monotonic_max = 0
        elseif (ir-i_monotonic_max)>min_width then
          if monotonic_max>c_arr[1]  then
            c_arr[1] = monotonic_max
            i_arr[1] = i_monotonic_max
          elseif monotonic_max>c_arr[#c_arr]  then
            c_arr[#c_arr] = monotonic_max
            i_arr[#i_arr] = i_monotonic_max
          end
          i_monotonic_max = ir
          monotonic_max = 0
        end
      end
    end
    -- Save the parallel lines
    if #i_arr==2 then
      --print(ith,'ith',unpack(c_arr))
      -- Dominate the previous maxes
      if c_arr[1]>cntMax1 and c_arr[2]>cntMax2 then
        irMax1, irMax2, ithMax = i_arr[1], i_arr[2], ith
        cntMax1, cntMax2 = c_arr[1], c_arr[2]
        found = true
      elseif c_arr[1]>cntMax1 and c_arr[2]>cntMax2 then
        irMax1, irMax2, ithMax = i_arr[2], i_arr[1], ith
        cntMax1, cntMax2 = c_arr[2], c_arr[1]
        found = true
      end
    end
  end
  -- Yield the parallel lines
  if not found then return end

  --print('PARALLEL',ithMax,irMax1-irMax2)

  local s, c = sin_d[ithMax], cos_d[ithMax]
  -- Find the image indices
  local iR1 = irMax1 * c
  local iR2 = irMax2 * c
  local jR1 = irMax1 * s
  local jR2 = irMax2 * s
  local lMean1 = line_sum_d[ithMax][irMax1] / count_d[ithMax][irMax1]
  local lMin1 = line_min_d[ithMax][irMax1]
  local lMax1 = line_max_d[ithMax][irMax1]
  local lMean2 = line_sum_d[ithMax][irMax2] / count_d[ithMax][irMax2]
  local lMin2 = line_min_d[ithMax][irMax2]
  local lMax2 = line_max_d[ithMax][irMax2]

  return {
      iMean = iR1 - lMean1 * s,
      jMean = jR1 + lMean1 * c,
      iMin = iR1 - lMin1 * s,
      jMin = jR1 + lMin1 * c,
      iMax = iR1 - lMax1 * s,
      jMax = jR1 + lMax1 * c,
    },
    {
      iMean = iR2 - lMean1 * s,
      jMean = jR2 + lMean1 * c,
      iMin = iR2 - lMin1 * s,
      jMin = jR2 + lMin1 * c,
      iMax = iR2 - lMax1 * s,
      jMax = jR2 + lMax1 * c,
    },
    {
      ith = ithMax,
      ir1 = irMax1,
      ir2 = irMax2,
      c1 = cntMax1,
      c2 = cntMax2,
      NTH = NTH,
      NR = NR,
    }

end

-- Converts to torch
function RadonTransform.get_population ()
  local torch = require'torch'
  --Int
  local count_t = torch.IntTensor(NTH, NR)
  local count_s = count_t:storage()
  local tmp_s = torch.IntStorage(
  NTH*NR,
  tonumber(ffi.cast("intptr_t",count_d))
  )
  count_s:copy(tmp_s)
  --
  local line_sum_t = torch.IntTensor(NTH, NR)
  local line_sum_s = line_sum_t:storage()
  local tmp_s = torch.IntStorage(
  NTH*NR,
  tonumber(ffi.cast("intptr_t",line_sum_d))
  )
  line_sum_s:copy(tmp_s)
  --
  local line_min_t = torch.IntTensor(NTH, NR)
  local line_min_s = line_min_t:storage()
  local tmp_s = torch.IntStorage(
  NTH*NR,
  tonumber(ffi.cast("intptr_t",line_min_d))
  )
  line_min_s:copy(tmp_s)
  --
  local line_max_t = torch.IntTensor(NTH, NR)
  local line_max_s = line_max_t:storage()
  local tmp_s = torch.IntStorage(
  NTH*NR,
  tonumber(ffi.cast("intptr_t",line_max_d))
  )
  line_max_s:copy(tmp_s)
  return count_t, line_sum_t, line_min_t, line_max_t
end

return RadonTransform
