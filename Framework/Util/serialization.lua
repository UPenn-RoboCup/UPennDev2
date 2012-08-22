module(..., package.seeall);

function serialize(o)
  local str = "";
  if type(o) == "number" then
    str = tostring(o);
  elseif type(o) == "string" then
    str = string.format("%q",o);
  elseif type(o) == "table" then
    str = "{";
    for k,v in pairs(o) do
      str = str..string.format("[%s]=%s,",serialize(k),serialize(v));
    end
    str = str.."}";
  else
    str = "nil";
  end
  return str;
end

function deserialize(s)
  --local x = assert(loadstring("return "..s))();
  local x = loadstring("return "..s)();
  if (not x) then
    print(string.format("Could not deserialize: %s",s));
  end
  return x;
end
