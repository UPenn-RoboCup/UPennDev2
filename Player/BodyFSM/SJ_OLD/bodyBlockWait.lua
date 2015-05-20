local state = {}
state._NAME = ...

require'mcm'

function state.entry()
  print(state._NAME..' Entry' )
end

function state.update()
  if mcm.get_motion_state()==2 then --stopped
    return "done"
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
