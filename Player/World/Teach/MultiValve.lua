local MultiValve = {}
require'hcm'
local ImageProc2 = require'ImageProc.ffi2'
local c_zlib = require'zlib.ffi'.compress
local ImProc, colors
local lA_meta = {
  c = 'zlib',
  id = 'labelA',
}
local detected
--local ptable = require'util'.ptable

hcm.set_guidance_color('GRAB ANY VALVE')

local discrete_prior_mt = {}
function discrete_prior_mt.__call(t, k)
	return t[k]
end

-- Autonomous Prior
local color_auto_priors = setmetatable({
	cyan = 1,
	magenta = 1,
	yellow = 1,
	}, discrete_prior_mt)
-- Normalize
local color_auto_priors_sum = 0
for k,v in pairs(color_auto_priors) do color_auto_priors_sum = color_auto_priors_sum + v end
for k,v in pairs(color_auto_priors) do color_auto_priors[k] = color_auto_priors[k] / color_auto_priors_sum end
-- Human prior
local color_human_priors = {
	cyan = setmetatable({cyan = 0.9, magenta = 0.1, yellow = 0.1,}, discrete_prior_mt),
	magenta = setmetatable({cyan = 0.1, magenta = 0.9, yellow = 0.1,}, discrete_prior_mt),
	yellow = setmetatable({cyan = 0.1, magenta = 0.1, yellow = 0.9,}, discrete_prior_mt),
}
-- Normalize
for kk, pr in pairs(color_human_priors) do
	local color_auto_priors_sum = 0
	for k,v in pairs(pr) do color_auto_priors_sum = color_auto_priors_sum + v end
	for k,v in pairs(pr) do pr[k] = v / color_auto_priors_sum end
end
-- Effective prior is the autonomous one
local color_prior = setmetatable({}, discrete_prior_mt)
for k,v in pairs(color_auto_priors) do
	color_auto_priors[k] = color_auto_priors[k] / color_auto_priors_sum
	-- Effective prior is the autonomous one
	color_prior[k] = color_auto_priors[k]
end


-- Human prior
local size_human_priors = {}
function size_human_priors.small(area)
	if area < 50 then return .7
	elseif area < 150 then return .6
	elseif area < 250 then return .5
	elseif area < 400 then return .4
	else return .1
	end
end
function size_human_priors.large(area)
	if area > 750 then return .7
	elseif area > 500 then return .6
	elseif area > 250 then return .5
	else return .1
	end
end
-- Autonomous Prior wants larger valves
local size_auto_priors = size_human_priors.large
-- Default
local size_prior = size_auto_priors

-- Should have a common API (meta/raw)
function MultiValve.send()
  local lA_raw = c_zlib(ImProc.labelA_d, ffi.sizeof(ImProc.labelA_d))
  lA_meta.sz = #lA_raw
  return {
  	{lA_meta, lA_raw},
		{detected},
	}
end

function MultiValve.entry(cfg)
	colors = cfg.vision.colors
	ImProc = ImageProc2.new(cfg.w, cfg.h, cfg.vision.scaleA, cfg.vision.scaleB)
  lA_meta.w = ImProc.wa
  lA_meta.h = ImProc.ha
	ImageProc2.load_lut(ImProc, table.concat{HOME, "/Data/", "lut_", cfg.lut, ".raw"})
end

function MultiValve.update(rgb, depth)
  ImageProc2.rgb_to_labelA(ImProc, rgb.data)
  ImageProc2.block_bitor(ImProc)
	-- Nothing detected yet
	detected = {
		id = 'detect',
	}
	local tracked, t_sum = {}, 0
	for color, color_index in pairs(colors) do
		----[[
	  local color_detected = ImageProc.connected_regions(
			tonumber(ffi.cast('intptr_t', ffi.cast('void *', ImProc.labelB_d))),
			ImProc.wb,
			ImProc.hb,
			color_index
		)
		--]]
		--[[
	  local color_detected = ImageProc2.connected_regions(
			ImProc.labelB_d,
			ImProc.wb,
			ImProc.hb,
			color_index
		)
		--]]
		if color_detected then
			local bbox_stats
			for i, d in ipairs(color_detected) do
				bbox_stats = ImageProc2.color_stats(ImProc.labelB_d, ImProc.wb, ImProc.hb, color_index, d.boundingBox)
				--for k, v in pairs(bbox_stats) do d[k] = d[k] or v end
				for k, v in pairs(d) do bbox_stats[k] = v or bbox_stats[k] end
				bbox_stats.color = color
				bbox_stats.p = 1
				t_sum = t_sum + bbox_stats.p
				table.insert(tracked, bbox_stats)
			end
		end
	end
	-- Normalize tracks
	for i,track in ipairs(tracked) do track.p = track.p / t_sum end
	
	-- Robot only guess based on area alone
	table.sort(tracked)
	--[[
	local ntracked = #tracked
	local t_sum = 0
	for i,track in ipairs(tracked) do
		--track.p = 1 / (2^i)
		--track.p = 1 / i
		track.p = track.area
		t_sum = t_sum + track.p
	end
	-- Normalize
	--io.write('Stage 2:')
	for i,track in ipairs(tracked) do
		track.p = track.p / t_sum
		--io.write(' ', track.p)
	end
	--io.write('\n')
	--]]
	
	-- Add human guess to color
	local g_t = hcm.get_guidance_t()
	if g_t > 0 then
		hcm.set_guidance_t({g_t})
		local g_color = hcm.get_guidance_color():match('GRAB%s([%s|%w]*)%sVALVE')
		g_color = g_color and g_color:lower() or ''
		-- Grab the prior for this vocabulary word
		local human_color_prior = color_human_priors[g_color]
		-- If does not exist, then no replacement
		if human_color_prior then
			-- Full replace the prior
			for k,v in pairs(color_prior) do color_prior[k] = human_color_prior[k] or color_prior[k] end
		elseif g_color:find'any color' or g_color=='any' then
			-- Go back to auto
			for k,v in pairs(color_prior) do color_prior[k] = color_auto_priors[k] end
		end
		local human_size_prior = g_color and size_human_priors[g_color]
		if human_size_prior then
			-- Full replace the prior
			size_prior = human_size_prior
		elseif g_color:find'any size' or g_color=='any' then
			-- Go back to auto
			size_prior = size_auto_priors
		end
	end
	-- Update tracked probabilities based on effective priors
	local t_sum = 0
	for i,track in ipairs(tracked) do
		track.p = track.p * color_prior(track.color) * size_prior(track.area)
		t_sum = t_sum + track.p
	end
	-- Normalize
	for i,track in ipairs(tracked) do
		track.p = track.p / t_sum
		--print(track.area, track.p)
	end
	-- Save
	detected.tracked = tracked
end

function MultiValve.exit()
end

return MultiValve
