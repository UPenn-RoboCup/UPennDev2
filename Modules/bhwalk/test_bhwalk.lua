bhwalk = require'bhwalk'

for k, v in pairs(bhwalk) do
  print(k,v)
  if type(v)=='function' then v() end
end

