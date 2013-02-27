
function createVRML(header)
  local vrml = {}
  vrml.__type = 'vrml'
  vrml.__header = header
  return vrml
end

function createNode(nodeName, nodeType)
  local node = {}
  node.__name = nodeName
  node.__nodeType = nodeType
  node.__type = 'node'

  return node
end

function createField(fieldName, value)
  local field = {}
  if type(value) == 'table' then
    field = value
  end
  field.__name = fieldName
  field.__type = 'field'

  return field
end

function createMultiField(fieldName, value)
  local field = {}
  if type(value) == 'table' then
    field = value
  end
  field.__name = fieldName
  field.__type = 'multifield'

  return field
end



function createProto(protoName)
  local proto = {}
  proto.__name = protoName
  proto.__type = 'field'
  return proto
end

function indentSpace(indent)
  local str = ''
  for i = 1, indent do
    str = str..'  '
  end
  return str
end



function writenode(file, node, indent)
  if node.__name then
    file:write(indentSpace(indent)..'DEF '..node.__name..' ')
    file:write(node.__nodeType..' {\n')
  else
    file:write(indentSpace(indent)..node.__nodeType..' {\n')
  end
  for k, v in ipairs(node) do
    _G['write'..v.__type](file, v, indent + 1)
  end
  file:write('}\n')
end

function writefield(file, field, indent)
  file:write(indentSpace(indent)..field.__name)
  for k, v in ipairs(field) do
    file:write(' '..v)
  end
  file:write('\n')
end

function writemultifield(file, field, indent)
  file:write(indentSpace(indent)..field.__name..' [\n')
  local numCount = 0
  local maxLineNum = 4
  for k, v in ipairs(field) do
    if type(v) == 'string' then
      file:write(indentSpace(indent+1)..'"'..v..'"\n')
    elseif type(v) == 'number' then
      file:write(indentSpace(indent+1))
      file:write(v..' ')
      numCount = numCount + 1
      if numCount % maxLineNum == 0 then
        file:write('\n')
      end
    end
  end
  file:write(indentSpace(indent)..']\n')
end

function saveVRML(vrml, filename)
  if vrml.__type ~= 'vrml' then
    error('input must be created by createVRML')
  end
  local indent = 0 
  file = assert(io.open(filename, 'w'))

  if vrml.__header then
    file:write(vrml.__header)
    file:write('\n\n')
  end
  for k, v in ipairs(vrml) do
    _G['write'..v.__type](file, v, indent)
  end
end
