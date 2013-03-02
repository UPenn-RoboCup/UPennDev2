require('Motion_state')

--------------------------------------------------------------------------
-- Walk Stop Controller
--------------------------------------------------------------------------

walk_stop = Motion_state.new('walk_stop')
walk_stop:set_joint_access(0, 'all')
walk_stop:set_joint_access(1, 'lowerbody')
local dcm = walk_stop.dcm

-- default parameters
walk_stop.parameters = {
}

function walk_stop:entry()
end

function walk_stop:update()
end

function walk_stop:exit()
end

return walk_stop
