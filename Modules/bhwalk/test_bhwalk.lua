bhwalk = require'bhwalk'

for k, v in pairs(bhwalk) do
  if type(v)=='function' then
    print(k,v())
  else
    print(k,v)
  end
end

