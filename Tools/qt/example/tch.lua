package.path = '/usr/local/share/torch/lua/?/?.lua;'..package.path
package.path = '/usr/local/share/torch/lua/?/init.lua;'..package.path
--package.cpath = '/usr/local/lib/?.so;/usr/local/lib/torch/?.so;'..package.cpath
package.cpath = '/usr/local/lib/?.dylib;/usr/local/lib/torch/?.dylib;'..package.cpath

require 'torch';

x = torch.Tensor(4,5);
s = x:storage();
for i = 1, s:size() do
  s[i] = 1;
end

print(x)


