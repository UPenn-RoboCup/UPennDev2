--formatting of non-structured types for pretty printing
local glue = require'glue' --index

local escapes = { --don't add unpopular escapes here
	['\\'] = '\\\\',
	['\t'] = '\\t',
	['\n'] = '\\n',
	['\r'] = '\\r',
}
local function escape_byte(c)
	return string.format('\\%03d', c:byte())
end
local function escape_byte_short(c)
	return string.format('\\%d', c:byte())
end
local function quote_string(s, quote)
	s = s:gsub('.', escapes)
	s = s:gsub(quote, '\\%1')
	s = s:gsub('[^\32-\126][0-9]', escape_byte)
	s = s:gsub('[^\32-\126]', escape_byte_short)
	return s
end

local function format_string(s, quote)
	return string.format('%s%s%s', quote, quote_string(s, quote), quote)
end

local function write_string(s, write, quote)
	write(quote); write(quote_string(s, quote)); write(quote)
end

local keywords = glue.index{
	'and',       'break',     'do',        'else',      'elseif',    'end',
	'false',     'for',       'function',  'goto',      'if',        'in',
	'local',     'nil',       'not',       'or',        'repeat',    'return',
	'then',      'true',      'until',     'while',
}
local function is_identifier(v)
	return type(v) == 'string' and not keywords[v]
				and v:match('^[a-zA-Z_][a-zA-Z_0-9]*$') ~= nil
end

local hasinf = math.huge == math.huge - 1
local function format_number(v)
	if v ~= v then
		return '0/0'
	elseif hasinf and v == math.huge then
		return '1/0' --writing 'math.huge' would not make it portable, just wrong
	elseif hasinf and v == -math.huge then
		return '-1/0'
	elseif v == math.floor(v) and v >= -2^31 and v <= 2^31-1 then
		return string.format('%d', v) --printing with %d is faster
	else
		return string.format('%0.17g', v)
	end
end

local function write_number(v, write)
	write(format_number(v))
end

local function is_dumpable(f)
	return type(f) == 'function' and debug.getinfo(f, 'Su').what ~= 'C'
end

local function format_function(f)
	return string.format('loadstring(%s)', format_string(string.dump(f)))
end

local function write_function(f, write, quote)
	write'loadstring('; write_string(string.dump(f), write, quote); write')'
end

local function pformat(v, quote)
	quote = quote or "'"
	if v == nil or type(v) == 'boolean' then
		return tostring(v)
	elseif type(v) == 'number' then
		return format_number(v)
	elseif type(v) == 'string' then
		return format_string(v, quote)
	elseif type(v) == 'function' and is_dumpable(v) then
		return format_function(v)
	end
end

local serializabe_types = glue.index{'nil', 'boolean', 'string', 'number', 'function'}
local function is_serializable(v)
	return serializabe_types[type(v)] and
				(type(v) ~= 'function' or is_dumpable(v))
end

local function pwrite(v, write, quote)
	quote = quote or "'"
	if v == nil or type(v) == 'boolean' then
		write(tostring(v))
	elseif type(v) == 'number' then
		write_number(v, write)
	elseif type(v) == 'string' then
		write_string(v, write, quote)
	elseif type(v) == 'function' then
		write_function(v, write, quote)
	end
end

return {
	is_identifier = is_identifier,
	is_dumpable = is_dumpable,
	is_serializable = is_serializable,

	format_string = format_string,
	format_number = format_number,
	format_function = format_function,

	write_string = write_string,
	write_number = write_number,
	write_function = write_function,

	pformat = pformat,
	pwrite = pwrite,
}
