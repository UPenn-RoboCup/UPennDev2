Config = {}

function load_config(config_name)
  local local_config = require(config_name)
  if (type(local_config) == 'table') then
    for k,v in pairs(local_config) do
      Config[k] = local_config[k]
    end
  end
end

function Config.get_field(field)
  local t = Config
  for k in string.gmatch(field, '(%a[%a_%d]*)') do
    if (type(t) ~= 'table') then
      return nil
    end
    t = t[k]
  end
  return t
end

load_config('Config_bias')
load_config('Config_platform')
load_config('Config_devices')
load_config('Config_mechanics')
load_config('Config_motion')

Config.platform = {}
Config.platform.name = 'gazebo_thor_op'

return Config
