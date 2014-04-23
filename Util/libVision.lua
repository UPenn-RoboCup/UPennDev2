-- libDetect
-- (c) 2014 Stephen McGill
-- General Detection methods
local libVision = {}
local torch  = require'torch'
local bit = require'bit'
local lshift = bit.lshift
local rshift = bit.rshift
local bor = bit.bor
local band = bit.band

function libVision.yuyv_to_labelA(yuyv_t, labelA_t, lut, cc, w, h)
  -- Correct dimensions
  local na, nb, s, idx, i = labelA_t:size(1)-1, labelA_t:size(2)-1, w*4, 0, 0
  -- C variable access
  local lA, yuyv, cc_c = labelA_t:data(), yuyv_t:data(), cc:data()
  -- Reset the color count
  cc:zero()
  -- Temporary variables for the loop
  local y6,u6,v6,index,cdt
  for a=0,na do
    for b=0,nb do
      y6, u6, v6 =
        rshift(yuyv[idx], 2),
        band(yuyv[idx+1],0xFC),
        band(yuyv[idx+2],0xFC)
      index = bor( y6, lshift(u6,4), lshift(v6,10) )
      cdt = lut[index]
      lA[i] = cdt
      cc_c[cdt] = cc_c[cdt] + 1
      -- Stride to next, assume 4
      idx = idx + 4
      i = i + 1
    end
    -- stride to next
    idx = idx + s
  end
  -- Return nothing, since given the tensors
end

return libVision
