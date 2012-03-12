module(..., package.seeall)

require('CommManager')
require('controller')
require('unix')

get_time = controller.wb_robot_get_time

function entry()
  -- initialize shared memory
  CommManager.entry()
  CommManager.update()
end

function update()
  CommManager.update()
end

function exit()
  CommManager.exit()
end
