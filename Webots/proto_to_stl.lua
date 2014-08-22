local robot = 'THOROP_Visual'
local dir = string.format('protos/%s',robot)
local ls_cmd = string.format('ls "%s"',dir)

local function proto_to_vrml( name )
  local pathname = dir..'/'..name..'.proto'
  local wrl_pathname = dir..'/wrl/'..name..'.wrl'
  local mkdir_cmd = string.format('mkdir -p %s/wrl',dir)
  
  -- Make dir
  local status = os.execute(mkdir_cmd)
  assert(status,'Failed to make wrl dir!')
  
  -- Open and read the proto
  local f = io.open(pathname,'r')
  local text = f:read('*all')
  f:close()
  
  -- Open the WRL file
  f = io.open(wrl_pathname,'w')
  -- Add the correct header
  f:write('#VRML V2.0 utf8\n')
  -- Remove the {} container for the PROTO
  local first_char,e = text:find('{')
  assert(first_char,'Could not find the first {')
  local final_char,e = text:reverse():find('}')
  assert(final_char,'Could not find the last }')
  final_char = #text-final_char+1
  local new_text = text:sub(first_char+1, final_char-1)
  -- Write the WRL file of the VRML format
  f:write(new_text)
  f:close()
end

local function vrml_to_stl(name)
  local wrl_pathname = dir..'/wrl/'..name..'.wrl'
  local stl_pathname = dir..'/stl/'..name
  local mkdir_cmd = string.format('mkdir -p %s/stl',dir)
  local status = os.execute(mkdir_cmd)
  assert(status,'Failed to make stl dir!')
  local convert_cmd = string.format('./meshconv %s -c stl -o %s 1>/dev/null',
  wrl_pathname,stl_pathname)
  local status = os.execute( convert_cmd )
  if status~=0 then print('!! Failed to convert',name) end
end

local protos = {}
for proto_name in io.popen( ls_cmd ):lines() do
  local s, e = proto_name:find('.proto')
  if s then
    local name = proto_name:sub(1,s-1)
    table.insert(protos,name)
    -- Convert to VRML
    print( string.format('Converting %s from proto to VRML.',name) )
    proto_to_vrml(name)
    -- Convert to STL
    print( string.format('Converting %s from VRML to STL.',name) )
    vrml_to_stl(name)
    print()
  end
end