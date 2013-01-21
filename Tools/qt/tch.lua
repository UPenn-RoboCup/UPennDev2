package.path = '/usr/local/share/torch/lua/?/?.lua;'..package.path
package.path = '/usr/local/share/torch/lua/?/init.lua;'..package.path
package.cpath = '/usr/local/lib/?.so;/usr/local/lib/torch/?.so;'..package.cpath

require 'torch'
x = torch.Tensor(7,7,7)
print(x)


