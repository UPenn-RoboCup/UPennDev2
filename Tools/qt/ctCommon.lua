
splitPath = function(str)
  local ptr = 1
  while str:find('/', ptr) ~= nil do
    ptr = ptr + 1
  end
  path = str:sub(1, ptr - 1)
  filename = str:sub(ptr, #str)
--  print(path, filename)
  return path, filename
end


