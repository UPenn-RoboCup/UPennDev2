local MultiValve = {}
local ImageProc = require'ImageProc'
local ImageProc2 = require'ImageProc.ffi2'
local c_zlib = require'zlib.ffi'.compress
local lA_meta = {
  c = 'zlib',
  id = 'labelA',
}
local wa, ha, wb, hb, colors
local detected = {
	id = 'detect'
}

-- Should have a common API (meta/raw)
function MultiValve.send()
  local lA_raw = c_zlib(ImageProc2.labelA_d, ImageProc2.labelA_n)
  lA_meta.sz = #lA_raw
  return {
  	{lA_meta, lA_raw},
		{detected},
	}
end

function MultiValve.entry(cfg)
	ImageProc2.setup(cfg.w, cfg.h, cfg.vision.scaleA, cfg.vision.scaleB)
	ImageProc2.load_lut(table.concat{HOME, "/Data/", "lut_", cfg.lut, ".raw"})
	local info = ImageProc2.get_info()
	wa, ha, wb, hb = info.wa, info.ha, info.wb, info.hb
	colors = cfg.vision.colors
  lA_meta.w = wa
  lA_meta.h = ha
end

function MultiValve.update(rgb, depth)
  ImageProc2.rgb_to_labelA(rgb.data)
  ImageProc2.block_bitor()
	for color, index in pairs(colors) do
	  detected[color] = ImageProc.connected_regions(
			tonumber(ffi.cast('intptr_t', ffi.cast('void *', ImageProc2.labelB_d))),
			wb,
			hb,
			index
		)
	end
end

function MultiValve.exit()
end

return MultiValve
