module(..., package.seeall);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end

function event(e)
  sm:add_event(e);
end
