require('unix')

Body = {}

Body.get_time = unix.time

function Body.entry()
end

function Body.update()
end

function Body.exit()
end

return Body
