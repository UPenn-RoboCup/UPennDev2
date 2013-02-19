require('zmq')
require('unix')
require('cmsgpack')

----------------------------------------------------------------------
-- rpc : client / server implementation for remote procedure calls
----------------------------------------------------------------------

rpc = {}
rpc.client = {}
rpc.server = {}
rpc.client.__index = rpc.client
rpc.server.__index = rpc.server
rpc.client.__mtstring = 'rpc.client'
rpc.server.__mtstring = 'rpc.server'

-- utilities
----------------------------------------------------------------------

local function get_functions(t, searched)
  if (t == package) then return {} end
  local searched = searched or {}
  local functions = {}
  searched[t] = true
  for k,v in pairs(t) do
    if (type(k) == 'string') then
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
  end
  return functions
end

local function generate_dictionary()
  return get_functions(_G)
end

-- rpc client
----------------------------------------------------------------------

function rpc.client.new(endpoint, zmq_context)
  local o = {}
  o.request_id = 0
  o.mode = 'strict'
  o.timeout = nil
  o.response = false
  o.dictionary = {}
  o.return_values = {}
  o.return_status = false
  o.context = zmq_context
  o.endpoint = endpoint
  o.sock = o.context:socket(zmq.REQ)
  o.sock:connect(endpoint)
  o.poller = zmq.ZMQ_Poller(1)
  o.poller:add(o.sock, zmq.POLLIN)
  return setmetatable(o, rpc.client)
end

function rpc.client.set_strict(o)
  o.mode = 'strict'
end 

function rpc.client.set_lenient(o)
  o.mode = 'lenient'
end

function rpc.client.set_lazy(o)
  o.mode = 'lazy'
end

function rpc.client.set_timeout(o, seconds)
  o.timeout = seconds
end

function rpc.client.get_timeout(o)
  return o.timeout
end

function rpc.client.get_socket(o)
  return o.sock
end

function rpc.client.get_dictionary(o)
  return o.dictionary
end

function rpc.client.get_request_id(o)
  return o.request_id
end

function rpc.client.connect(o, timeout)
  -- get call dictionary from server 
  o.request_id = -1
  local rpc_request = {
    id = o.request_id,
    procedure = '',
    arguments = nil,
  }
  o.sock:send(cmsgpack.pack(rpc_request))
  local return_status, dictionary = o:receive(timeout or o.timeout)
  o.dictionary = return_status and dictionary or o.dictionary
  return return_status
end

function rpc.client.call(o, procedure_name, ...)
  -- execute remote procedure call
  o.request_id = (o.request_id + 1) % 32767
  local request = {
    id = o.request_id,
    procedure = procedure_name,
    arguments = {...},
  }
  o.sock:send(cmsgpack.pack(request))
  return o:receive(o.timeout)
end

function rpc.client.receive(o, timeout)
  -- block for rpc response
  o.return_values = {'timeout error'}
  o.return_status = false
  o.response = false
  local dt = 0
  local t0 = unix.time()
  local timeout = timeout or math.huge
  while (not o.response and dt <= timeout) do
    if (o.poller:poll(1000*(timeout - dt)) > 0) then
      o:handle_rpc_response(o.sock:recv())
    end
    dt = unix.time() - t0
  end
  if (not o.response) then
    -- reconnect to server
    o.poller:remove(o.sock, zmq.POLLIN)
    o.sock:close()
    o.sock = o.context:socket(zmq.REQ)
    o.sock:connect(o.endpoint)
    o.poller:add(o.sock, zmq.POLLIN)
  end
  return o.return_status, unpack(o.return_values)
end

function rpc.client.handle_rpc_response(o, msg)
  -- handle rpc response
  local success, response = pcall(cmsgpack.unpack, msg)
  if (success and response.id == o.request_id) then
    o.response = true
    o.return_values = response.values
    o.return_status = response.status
  end
end

function rpc.client.close(o)
  o.sock:close()
end

-- rpc server methods
----------------------------------------------------------------------

function rpc.server.new(endpoint, zmq_context)
  local o = {}
  o.timeout = nil
  o.blacklist = {}
  o.dictionary = generate_dictionary()
  o.context = zmq_context
  o.endpoint = endpoint
  o.sock = o.context:socket(zmq.REP)
  o.sock:bind(endpoint)
  o.poller = zmq.ZMQ_Poller(1)
  o.poller:add(o.sock, zmq.POLLIN)
  return setmetatable(o, rpc.server)
end

function rpc.server.set_timeout(o, seconds)
  o.timeout = seconds
end

function rpc.server.get_timeout(o)
  return o.timeout
end

function rpc.server.get_lcm(o)
  return o.lcm  
end

function rpc.server.get_dictionary(o)
  return o.dictionary
end

function rpc.server.generate_dictionary(o)
  o.dictionary = generate_dictionary()
end

function rpc.server.blacklist(o)
  -- TODO purge blacklisted functions from dictionary
end

function rpc.server.update(o)
  local timeout = o.timeout or math.huge
  local count = o.poller:poll(1000*timeout)
  while (count > 0) do
    o:handle_rpc_request(o.sock:recv())
    count = o.poller:poll(0)
  end
end

function rpc.server.handle_rpc_request(o, msg)
  local response
  local return_values
  local return_status
  local success, request = pcall(cmsgpack.unpack, msg)

  if (success) then
    if (request.id == -1) then
      return_values = {o:get_dictionary()}
      return_status = true
    else
      local object = request.procedure:match('^([%a_%d%.]+):[%a_%d]+$')
      local procedure = request.procedure:gsub(':', '.')
      local arguments = request.arguments
      local success, object = 
        pcall(loadstring('return '..tostring(object)))
      local success, procedure = 
        pcall(loadstring('return '..procedure))
      if (success and object) then
        table.insert(arguments, 1, object)
      end
      if (not success) then
        return_values = procedure
        return_status = false
      else
        return_values = {pcall(procedure, unpack(arguments))}
        return_status = table.remove(return_values, 1)
      end
    end
    response = {
      id = request.id,
      values = return_values,
      status = return_status,
    }
  else
    response = {
      id = -1,
      values = {'decoding error'},
      status = false,
    }
  end
  o.sock:send(cmsgpack.pack(response))
end

function rpc.server.close(o)
  o.sock:close()
end

return rpc
