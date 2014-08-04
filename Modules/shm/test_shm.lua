local shm = require('shm')
t = shm.new('test')
print('New...', t)

t:set('a', 3.14 )
print("t.a",t.a)
t.a = 3.14
print("t.a",t.a)
t.b = {1, -2, -3}
if t.b then
	for i, v in ipairs(t.b) do
		print(i,v)
	end
end
