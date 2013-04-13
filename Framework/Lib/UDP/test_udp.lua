require 'Udp'
Udp.init('192.168.123.25',54321)
for i=1,10 do
  local ret = Udp.send('hello')
  print('Sent',ret,'bytes')
end
