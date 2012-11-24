require('lcm')
require('lcm_rpc_request_t')
require('lcm_rpc_response_t')
require('serialization')
require('unix')

----------------------------------------------------------------------
-- rpc : client / server implementation for remote procedure calls
----------------------------------------------------------------------

rpc = {}

local rpc_client_mt = {}
local rpc_server_mt = {}
rpc_client_mt.__index = rpc_client_mt 
rpc_server_mt.__index = rpc_server_mt 

-- utilities
----------------------------------------------------------------------

local function uuidgen()
  local pipe = io.popen('uuidgen', 'r')
  local uuid = pipe:read('*a')
  pipe:close()
  return uuid
end

local function serialize_call(call_string, ...)
  local arg = {...}
  local arg_list = {}
  for i = 1,#arg do
    arg_list[i] = serialization.serialize(arg[i])
  end
  return 'return '..call_string..'('..table.concat(arg_list, ',')..')'
end

local function serialize_return_values(...)
  local arg = {...}
  local arg_list = {}
  for i = 1,#arg do
    arg_list[i] = serialization.serialize(arg[i])
  end
  return 'return '..table.concat(arg_list, ',')
end

local function get_functions(t, searched)
  if (t == package) then return {} end
  local searched = searched or {}
  local functions = {}
  searched[t] = true
  for k,v in pairs(t) do
    if (type(v) == 'function') then
      functions[k] = true
    elseif (type(v) == 'table') then
      if not(searched[v]) then
        local methods = get_functions(v, searched)
        for m in pairs(methods) do
          functions[k..'.'..m] = true
        end
      end
    end
  end
  return functions
end

local function generate_dictionary()
  return get_functions(_G)
end

-- message handlers
----------------------------------------------------------------------

local function handle_rpc_request(channel, msg, o)
  local return_values
  local return_status
  if (msg.request_id == -1) then
    return_values = {o:get_dictionary()}
    return_status = true
  else
    return_values = {pcall(loadstring(msg.eval_string))}
    return_status = table.remove(return_values, 1)
  end
  if (msg.synchronous) then
    local eval_string = serialize_return_values(unpack(return_values))
    local response = {
      client_id      = msg.client_id,
      request_id     = msg.request_id,
      eval_string    = eval_string,
      eval_nbytes    = #eval_string,
      return_status  = return_status,
    }
    o.lcm:rpc_response_t_publish(o.response_channel, response)
  end
end

local function handle_rpc_response(channel, msg, o)
  if (msg.client_id == o.client_id)
  and (msg.request_id == o.request_id) then
    local return_values = {pcall(loadstring(msg.eval_string))}
    local return_status = table.remove(return_values, 1)
    o.return_values = return_values 
    o.return_status = return_status and msg.return_status
    o.response = true
  end
end

-- rpc client / server constuctors
----------------------------------------------------------------------

function rpc.new_client(channel, provider)
  local o = {}
  o.client_id = uuidgen() -- 36 character uuid
  o.request_id = 0        -- 16 bit signed integer
  o.mode = 'strict'
  o.timeout = nil
  o.response = false
  o.dictionary = {}
  o.return_values = {}
  o.return_status = false
  o.lcm = lcm.new(provider)
  o.request_channel = channel..'_RPC_REQUEST'
  o.response_channel = channel..'_RPC_RESPONSE'
  o.lcm:rpc_response_t_subscribe(o.response_channel, handle_rpc_response, o)
  return setmetatable(o, rpc_client_mt)
end

function rpc.new_server(channel, provider)
  local o = {}
  o.timeout = 0
  o.clients = 0
  o.blacklist = {}
  o.dictionary = generate_dictionary()
  o.lcm = lcm.new(provider)
  o.request_channel = channel..'_RPC_REQUEST'
  o.response_channel = channel..'_RPC_RESPONSE'
  o.lcm:rpc_request_t_subscribe(o.request_channel, handle_rpc_request, o)
  return setmetatable(o, rpc_server_mt)
end

-- rpc client methods
----------------------------------------------------------------------

function rpc_client_mt.set_strict(o)
  o.mode = 'strict'
end 

function rpc_client_mt.set_lenient(o)
  o.mode = 'lenient'
end

function rpc_client_mt.set_lazy(o)
  o.mode = 'lazy'
end

function rpc_client_mt.set_timeout(o, seconds)
  o.timeout = seconds
end

function rpc_client_mt.get_timeout(o)
  return o.timeout
end

function rpc_client_mt.get_lcm(o)
  return o.lcm
end

function rpc_client_mt.get_dictionary(o)
  return o.dictionary
end

function rpc_client_mt.get_client_id(o)
  return o.client_id
end

function rpc_client_mt.get_request_id(o)
  return o.request_id
end

function rpc_client_mt.connect(o, timeout)
  -- get call dictionary from server 
  local _timeout = o.timeout
  o.timeout = timeout
  o.return_values = {}
  o.return_status = false
  o.response = false
  o.request_id = -1
  local rpc_request = {
    client_id   = o.client_id,
    request_id  = o.request_id,
    eval_string = '',
    eval_nbytes = 0,
    synchronous = true,
  }
  o.lcm:rpc_request_t_publish(o.request_channel, rpc_request)
  local status, dictionary = o:get_results()
  o.timeout = _timeout
  o.dictionary = status and dictionary or o.dictionary
  return status
end

function rpc_client_mt.eval(o, call_string, ...)
  -- non-blocking remote procedure call (TODO perform dictionary check)
  o.return_values = {}
  o.return_status = true
  o.response = true
  o.request_id = (o.request_id + 1) % 32767
  local eval_string = serialize_call(call_string, ...)
  local rpc_request = {
    client_id   = o.client_id,
    request_id  = o.request_id,
    eval_string = eval_string,
    eval_nbytes = #eval_string,
    synchronous = false,
  }
  o.lcm:rpc_request_t_publish(o.request_channel, rpc_request)
end

function rpc_client_mt.call(o, call_string, ...)
  -- blocking remote procedure call (TODO perform dictionary check)
  o.return_values = {}
  o.return_status = false
  o.response = false
  o.request_id = (o.request_id + 1) % 32767
  local eval_string = serialize_call(call_string, ...)
  local rpc_request = {
    client_id   = o.client_id,
    request_id  = o.request_id,
    eval_string = eval_string,
    eval_nbytes = #eval_string,
    synchronous = true,
  }
  o.lcm:rpc_request_t_publish(o.request_channel, rpc_request)
  return o:get_results()
end

function rpc_client_mt.get_results(o)
  -- block for rpc response (avoid calling directly)
  if (not o.timeout) then
    while (not o.response) do
      o.lcm:handle()
    end
  else
    local dt = 0
    local t0 = unix.time()
    local fd = o.lcm:get_fileno()
    while (not o.response) and (dt <= o.timeout) do
      if (unix.select({fd}, o.timeout - dt) > 0) then
        o.lcm:handle()
      end
      dt = unix.time() - t0
    end
  end
  return o.return_status, unpack(o.return_values)
end

-- rpc server methods
----------------------------------------------------------------------

function rpc_server_mt.set_timeout(o, seconds)
  o.timeout = seconds
end

function rpc_server_mt.get_timeout(o)
  return o.timeout
end

function rpc_server_mt.get_lcm(o)
  return o.lcm  
end

function rpc_server_mt.get_dictionary(o)
  return o.dictionary
end

function rpc_server_mt.generate_dictionary(o)
  o.dictionary = generate_dictionary()
end

function rpc_server_mt.blacklist(o)
  -- TODO purge blacklisted functions from dictionary
end

function rpc_server_mt.update(o)
  local fd = o.lcm:get_fileno()
  if (unix.select({fd}, o.timeout) > 0) then
    o.lcm:handle()
  end
end

return rpc
