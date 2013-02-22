----------------------------------------------------------------------
-- vector : variable length vector
----------------------------------------------------------------------

vector = {}
vector.__index = vector
vector.__mtstring = 'vector'

function vector.new(v)
  v = v or {}
  return setmetatable(v, vector)
end

function vector.ones(n)
  n = n or 1
  local v = {}
  for i = 1, n do
    v[i] = 1
  end
  return setmetatable(v, vector)
end

function vector.zeros(n)
  n = n or 1
  local v = {}
  for i = 1, n do
    v[i] = 0
  end
  return setmetatable(v, vector)
end

function vector.copy(v1)
  if not v1 then return nil end
  local v = {}
  for i = 1,#v1 do
    v[i] = v1[i]
  end
  return setmetatable(v, vector)
end

function vector.slice(v1, istart, iend)
  local v = {}
  iend = iend or #v1
  for i = 1,iend-istart+1 do
    v[i] = v1[istart+i-1]
  end
  return setmetatable(v, vector)
end

function vector.norm(v1)
  local s = 0
  for i = 1, #v1 do
    s = s + v1[i] * v1[i]
  end
  return math.sqrt(s)
end

function vector.mulnum(v1, a)
  local v = {}
  for i = 1, #v1 do
    v[i] = a * v1[i]
  end
  return setmetatable(v, vector)
end

function vector.divnum(v1, a)
  local v = {}
  for i = 1, #v1 do
    v[i] = v1[i]/a
  end
  return setmetatable(v, vector)
end

function vector.__add(v1, v2)
  if getmetatable(v1) == getmetatable(v2) then 
    local v = {}
    for i = 1, #v1 do
      v[i] = v1[i] + v2[i]
    end
    return setmetatable(v, vector)
  else
    error("attempt to add vector to non-vector type")
  end
end

function vector.__sub(v1, v2)
  if getmetatable(v1) == getmetatable(v2) then 
    local v = {}
    for i = 1, #v1 do
      v[i] = v1[i] - v2[i]
    end
    return setmetatable(v, vector)
  else
    error("attempt to subtract non-vector type from vector")
  end
end

function vector.__mul(v1, v2)
  if type(v2) == "number" then
    return vector.mulnum(v1, v2)
  elseif type(v1) == "number" then
    return vector.mulnum(v2, v1)
  else
    local s = 0
    for i = 1, #v1 do
      s = s + v1[i] * v2[i]
    end
    return s
  end
end

function vector.__div(v1, v2)
  if type(v2) == "number" then
    return vector.divnum(v1, v2)
  else
    error("attempt to divide vector by non-number type")
  end
end

function vector.__unm(v1)
  return vector.mulnum(v1, -1)
end

function vector.__tostring(v1)
  return "{"..table.concat(v1, ', ').."}"
end

return vector
