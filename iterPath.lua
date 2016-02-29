f = io.open('/tmp/qp.path')
raw = f:read("*all")
f:close()
o = mp.unpack(raw)
q=torch.Tensor(o)
o = nil
nq = 7
np = q:nElement() / nq
print('np', np)
qPath = q:view(7, np):t():clone()
q = nil

local K = require'K_ffi'
local T = require'Transform'
mattorch = require'mattorch'
trs = {}
tr6s = {}
for i=1,np do
  local q = vector.new(qPath[i])
  local tr = K.forward_larm(q)
  table.insert(trs, tr)
  table.insert(tr6s, T.position6D(tr))
end

o=mp.pack(tr6s)
f = io.open('/tmp/fk.mp','w')
raw = f:write(o)
f:close()