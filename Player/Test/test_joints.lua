--Prints dynamically changing joint positions in real time
module(... or "", package.seeall)

require('init')
require('Body')

function update()
  Body.get_all_positions()
end
