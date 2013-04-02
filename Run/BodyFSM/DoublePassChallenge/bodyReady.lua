module(..., package.seeall);

require('Body')
require('walk')

function entry()
end

function update()
  walk.stop();  --Don't move at ready stage
end

function exit()
end

