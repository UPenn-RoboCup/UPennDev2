

--require("test_estop");
--x2=t:get('t.a')
--print("t.a",x2)
--



local shm = require('shm')
t2 = shm.open('estop')
y=t2:get('a')

--print("hi")
print(y)