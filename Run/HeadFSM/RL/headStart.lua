module(..., package.seeall);

local Body = require('Body')

function entry()
  print(_NAME.." entry");

end

function update()
  return 'done';
end

function exit()
end
