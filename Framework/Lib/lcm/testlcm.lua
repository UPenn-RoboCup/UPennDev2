local lcm = require('lualcm')

for k, v in pairs(lcm) do
  if type(v) == 'function' then
    print(k)
  end
end

