local glue = require'glue'
require'unit'

test(select(2,pcall(glue.assert,false,'bad %s','dog')), 'bad dog')
test(select(2,pcall(glue.assert,false,'bad dog %s')), 'bad dog %s')
test({pcall(glue.assert,1,2,3)}, {true,1,2,3})

test(glue.index{a=5,b=7,c=3}, {[5]='a',[7]='b',[3]='c'})
test(glue.sort(glue.keys{apple=15,banana=23}), {'apple','banana'})
test(glue.update({a=1,b=2,c=3}, {d='add',b='overwrite'}, {b='over2'}), {a=1,b='over2',c=3,d='add'})
test(glue.merge({a=1,b=2,c=3}, {d='add',b='overwrite'}, {b='over2'}), {a=1,b=2,c=3,d='add'})
test(glue.extend({5,6,8}, {1,2}, {'b','x'}), {5,6,8,1,2,'b','x'})

test(glue.min{5,2,11,3}, 2)
test(glue.max{5,2,11,3}, 11)

test(glue.sum{1,10,100,1000}, 1111)
test(glue.sum({{count=5},{count=8},{count=2}},'count'), 15)

test(glue.reverse{1,2,3},{3,2,1})
test(glue.reverse{1,2,3,4},{4,3,2,1})

local function test1(s,sep,expect)
	local t={} for c in glue.gsplit(s,sep) do t[#t+1]=c end
	assert(#t == #expect)
	for i=1,#t do assert(t[i] == expect[i]) end
	test(t, expect)
end
test1('','',{''})
test1('','asdf',{''})
test1('asdf','',{'asdf'})
test1('', ',', {''})
test1(',', ',', {'',''})
test1('a', ',', {'a'})
test1('a,b', ',', {'a','b'})
test1('a,b,', ',', {'a','b',''})
test1(',a,b', ',', {'','a','b'})
test1(',a,b,', ',', {'','a','b',''})
test1(',a,,b,', ',', {'','a','','b',''})
test1('a,,b', ',', {'a','','b'})
test1('asd  ,   fgh  ,;  qwe, rty.   ,jkl', '%s*[,.;]%s*', {'asd','fgh','','qwe','rty','','jkl'})
test1('Spam eggs spam spam and ham', 'spam', {'Spam eggs ',' ',' and ham'})
t = {} for s,n in glue.gsplit('a 12,b 15x,c 20', '%s*(%d*),') do t[#t+1]={s,n} end
test(t, {{'a','12'},{'b 15x',''},{'c 20',nil}})
--TODO: use case with () capture

test(glue.trim(',  a , x  ,, d ,', '%s,'), 'a , x  ,, d')

test(glue.escape'^{(.-)}$', '%^{%(%.%-%)}%$')
test(glue.escape'%\0%', '%%%z%%')

test(glue.starts('abc', 'x'),false)
test(glue.starts('abc', ''),true)
test(glue.starts('abc', 'ab'),true)
test(glue.starts('abc', 'abc'),true)
test(glue.starts('abc', 'abcd'),false)

test(glue.ends('abc', 'x'),false)
test(glue.ends('abc', ''),true)
test(glue.ends('abc', 'bc'),true)
test(glue.ends('abc', 'abc'),true)
test(glue.ends('abc', 'abcd'),false)

test(glue.collect(('abc'):gmatch('.')), {'a','b','c'})
test(glue.collect(2,ipairs{5,7,2}), {5,7,2})

do
	local i = 0
	local function testiter()
		i = i + 1
		if i % 2 == 0 then error('even',0) end
		if i > 3 then return end
		return i,i^2
	end
	t = {}
	for ok,v1,v2 in glue.ipcall(testiter) do
		t[#t+1] = {ok,v1,v2}
	end
	test(t, {{true,1,1}, {false,'even'}, {true,3,9}, {false,'even'}})
end

