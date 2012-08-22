require('cbuffer')

x = cbuffer.new('raw', 4)
x:set('int16', 1000, 0)
x:set('int8', 128, 2)
x:set('int8', 129, 3)
print(x:get('int16', 0))
print(x:get('int8', 2))
print(x:get('int8', 3))

y = cbuffer.new('string', '\22\245\150\200')
print(y:get('uint8', 0))
print(y:get('uint8', 1))
print(y:get('uint8', 2))
print(y:get('uint8', 3))
print(y:get('float'))

y = cbuffer.new('float', 27.5)
print(y:get('float'))
