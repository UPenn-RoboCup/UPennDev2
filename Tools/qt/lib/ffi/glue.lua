--glue library, see http://code.google.com/p/lua-files/wiki/glue

local glue = {}

function glue.index(t)
	local dt={} for k,v in pairs(t) do dt[v]=k end
	return dt
end

function glue.keys(t)
	local dt={} for k in pairs(t) do dt[#dt+1]=k end
	return dt
end

function glue.update(dt,...)
	for i=1,select('#',...) do
		local t=select(i,...)
		if t ~= nil then
			for k,v in pairs(t) do dt[k]=v end
		end
	end
	return dt
end

function glue.merge(dt,...)
	for i=1,select('#',...) do
		local t=select(i,...)
		if t ~= nil then
			for k,v in pairs(t) do
				if dt[k] == nil then dt[k]=v end
			end
		end
	end
	return dt
end

function glue.extend(dt,...)
	for j=1,select('#',...) do
		local t=select(j,...)
		if t ~= nil then
			for i=1,#t do dt[#dt+1]=t[i] end
		end
	end
	return dt
end

function glue.append(dt,...)
	for i=1,select('#',...) do
		dt[#dt+1] = select(i,...)
	end
end

function glue.pluck(t,key)
	local dt={}
	for i=1,#t do dt[#dt+1]=t[i][key] end
	return dt
end

function glue.stringort(t,...)
	table.sort(t,...)
	return t
end

function glue.min(t,cmp)
	local n=t[1]
	if cmp then
		for i=2,#t do if cmp(t[i], n) then n=t[i] end end
	else
		for i=2,#t do if t[i] < n then n=t[i] end end
	end
	return n
end

function glue.max(t,cmp)
	local n=t[1]
	if cmp then
		for i=2,#t do if cmp(n, t[i]) then n=t[i] end end
	else
		for i=2,#t do if n < t[i] then n=t[i] end end
	end
	return n
end

function glue.stringum(t,key)
	local n=0
	if key then
		for i=1,#t do n=n+(t[i][key] or 0) end
	else
		for i=1,#t do n=n+(t[i] or 0) end
	end
	return n
end

function glue.reverse(t)
	for i=1,math.floor(#t/2) do
		t[#t-i+1],t[i]=t[i],t[#t-i+1]
	end
	return t
end

glue.string = {}

getmetatable''.__mod = function(s,v)
	if type(v) == 'table' then
		return s:format(unpack(v))
	else
		return s:format(v)
	end
end

function glue.string.gsplit(s, sep, start, plain)
	start = start or 1
	plain = plain or false
	local done = false
	local function pass(i, j, ...)
		if i then
			local seg = s:sub(start, i - 1)
			start = j + 1
			return seg, ...
		else
			done = true
			return s:sub(start)
		end
	end
	return function()
		if done then return end
		if sep == '' then done = true return s end
		return pass(s:find(sep, start, plain))
	end
end

function glue.string.trim(s,charset)
	charset = charset or '%s'
	local from = s:match('^['..charset..']*()')
	return from > #s and '' or s:match('.*[^'..charset..']', from)
end

local function format_ci_pat(c)
	return string.format('[%s%s]', c:lower(), c:upper())
end
function glue.string.escape(s,mode)
	if mode == '*i' then s = s:gsub('[%a]', format_ci_pat) end
	return (s:gsub('%%','%%%%'):gsub('%z','%%z')
				:gsub('([%^%$%(%)%.%[%]%*%+%-%?])', '%%%1'))
end

glue.string.P = glue.string.escape
glue.string.PI = function(s) return glue.string.escape'*i' end

function glue.string.starts(s,prefix)
	return s:find(prefix, 1, true) == 1
end

function glue.string.ends(s,suffix)
	return #suffix==0 or s:find(suffix, 1, true) == #s - #suffix + 1
end

function glue.string.fromhex(s)
	return (s:gsub('..', function(cc)
	  return string.char(tonumber(cc, 16))
	end))
end

function glue.string.tohex(s)
	if type(s) == 'number' then
		return string.format('%08.8x', s)
	end
	return (s:gsub('.', function(c)
	  return string.format('%02x', string.byte(c))
	end))
end

glue.update(glue, glue.string)

local function select_at(i,...)
	return ...,select(i,...)
end
local function collect_at(i,f,s,v)
	local t = {}
	repeat
		v,t[#t+1] = select_at(i,f(s,v))
	until v == nil
	return t
end
local function collect_first(f,s,v)
	local t = {}
	repeat
		v = f(s,v); t[#t+1] = v
	until v == nil
	return t
end
function glue.collect(n,...)
	if type(n) == 'number' then
		return collect_at(n,...)
	else
		return collect_first(n,...)
	end
end

function glue.ipcall(f,s,v)
	local function pass(ok,v1,...)
		v = v1
		return v and ok,v,...
	end
	return function()
		return pass(pcall(f,v))
	end
end

function glue.pass(...) return ... end

function glue.memoize(f)
	local t = {}
	return function(x)
		if t[x] == nil then t[x] = f(x) end
		return t[x]
	end
end

function glue.cache(f)
	return setmetatable({}, {__index = function(t,k)
		t[k] = f(k)
		return rawget(t,k)
	end})
end

local function index_parents(t,k)
	local parents = getmetatable(t).__parents
	for i=1,#parents do
		if parents[i][k] ~= nil then
			return parents[i][k]
		end
	end
end
local function setmeta(t,__index,__parents)
	local meta = getmetatable(t)
	if not meta then
		if not __index and not __parents then return t end
		meta = {}
		setmetatable(t, meta)
	end
	meta.__index = __index
	meta.__parents = __parents
	return t
end
function glue.inherit(t,...)
	local n=select('#',...)
	if n==0 then error('parent expected', 2) end
	if n==1 then return setmeta(t,...,nil) end
	local parents={}
	for i=1,n do
		parents[#parents+1]=select(i,...) --ignore nils
	end
	if #parents < 2 then return setmeta(t,parents[1],nil) end
	setmeta(t,index_parents,parents)
	return t
end

function glue.fileexists(name)
	local f = io.open(name, 'rb')
	if f then f:close() end
	return f ~= nil and name or nil
end

function glue.readfile(name, format)
	local f = assert(io.open(name, format=='t' and 'r' or 'rb'))
	local s = f:read'*a'
	f:close()
	return s
end

function glue.assert(v,err,...)
	if v then return v,err,... end
	err = err or 'assertion failed!'
	if select('#',...) > 0 then err = err:format(...) end
	error(err, 2)
end

function glue.unprotect(ok,result,...)
	if not ok then return nil,result,... end
	if result == nil then result = true end
	return result,...
end

local function pcall_error(e)
	return tostring(e) .. '\n' .. debug.traceback()
end
function glue.pcall(f, ...) --luajit and lua 5.2 only!
	return xpcall(f, pcall_error, ...)
end

function glue.fpcall(f,...) --bloated: 2 tables, 4 closures. can we reduce the overhead?
	local fint, errt = {}, {}
	local function finally(f) fint[#fint+1] = f end
	local function onerror(f) errt[#errt+1] = f end
	local function err(e)
		for i=#errt,1,-1 do errt[i]() end
		for i=#fint,1,-1 do fint[i]() end
		return tostring(e) .. '\n' .. debug.traceback()
	end
	local function pass(ok,...)
		if ok then
			for i=#fint,1,-1 do fint[i]() end
		end
		return glue.unprotect(ok,...)
	end
	return pass(xpcall(f, err, finally, onerror, ...))
end

function glue.fcall(f,...)
	return assert(glue.fpcall(f,...))
end

if not ... then require'glue_test' end

return glue

