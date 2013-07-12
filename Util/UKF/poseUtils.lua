require 'torch'

function QuatCompare(q1, q2)
  local e1 = Quat2Vector(q1)
  local e2 = Quat2Vector(q2)
--  return math.abs(e1:norm() - e2:norm())
  return (e1-e2):norm()
end

function Vector2Quat(W, dt)
  local w = torch.DoubleTensor(3):copy(W) 
  local dq = torch.DoubleTensor(4):fill(0)
  local wNorm = w:norm()
  if wNorm < 1e-6 then
    dq[1] = 1
    return dq
  end
  if dt then
    dAngle = wNorm * dt
  else
    dAngle = wNorm
  end
  dAxis = w:div(wNorm) 
  dq[1] = math.cos(dAngle / 2)
  dq[{{2, 4}}] = dAxis * math.sin(dAngle / 2)
  return dq
end

function Quat2Vector(Q)
  local q = torch.DoubleTensor(4):copy(Q)
--  if q[1] > 1 then q[1] = 1 end
  local alphaW = 2*math.acos(q[1])
  local v = torch.DoubleTensor(3):fill(0)
--  print('q', math.sin(alphaW/2) * alphaW)
--  print(q)
  if alphaW > 1e-6 then
    v[1] = q[2] / math.sin(alphaW/2) * alphaW
    v[2] = q[3] / math.sin(alphaW/2) * alphaW
    v[3] = q[4] / math.sin(alphaW/2) * alphaW
  end
  return v
end

function QuatInv(Q)
  local q = torch.DoubleTensor(4):copy(Q)
  local rt = q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2
  return torch.DoubleTensor({q[1]/rt, -q[2]/rt,
                            -q[3]/rt, -q[4]/rt})
end

function Quat2R(qin)
  local R = torch.DoubleTensor(3,3):fill(0)
  local q = torch.DoubleTensor(4):copy(qin)

  R[1][1] = 1 - 2 * q[3] * q[3] - 2 * q[4] * q[4]
  R[1][2] = 2 * q[2] * q[3] - 2 * q[4] * q[1]
  R[1][3] = 2 * q[2] * q[4] + 2 * q[3] * q[1]
  R[2][1] = 2 * q[2] * q[3] + 2 * q[4] * q[1]
  R[2][2] = 1 - 2 * q[2] * q[2] - 2 * q[4] * q[4]
  R[2][3] = 2 * q[3] * q[4] - 2 * q[2] * q[1]
  R[3][1] = 2 * q[2] * q[4] - 2 * q[3] * q[1]
  R[3][2] = 2 * q[3] * q[4] + 2 * q[2] * q[1]
  R[3][3] = 1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]

  return R
end

function R2Quat(Rin)
  local R = torch.DoubleTensor(3, 3):copy(Rin)
  local q = torch.DoubleTensor(4)
  local tr = R[1][1] + R[2][2] + R[3][3]
  if tr > 0 then
    local S = math.sqrt(tr + 1.0) * 2
    q[1] = 0.25 * S
    q[2] = (R[3][2] - R[2][3]) / S
    q[3] = (R[1][3] - R[3][1]) / S
    q[4] = (R[2][1] - R[1][2]) / S
  elseif R[1][1] > R[2][2] and R[1][1] > R[3][3] then
    local S = math.sqrt(1.0 + R[1][1] - R[2][2] - R[3][3]) * 2
    q[1] = (R[3][2] - R[2][3]) / S
    q[2] = 0.25 * S
    q[3] = (R[1][2] + R[2][1]) / S 
    q[4] = (R[1][3] + R[3][1]) / S
  elseif R[2][2] > R[3][3] then
    local S = math.sqrt(1.0 + R[2][2] - R[1][1] - R[3][3]) * 2
    q[1] = (R[1][3] - R[3][1]) / S
    q[2] = (R[1][2] + R[2][1]) / S 
    q[3] = 0.25 * S
    q[4] = (R[2][3] + R[3][2]) / S
  else
    local S = math.sqrt(1.0 + R[3][3] - R[1][1] - R[2][2]) * 2
    q[1] = (R[2][1] - R[1][2]) / S
    q[2] = (R[1][3] + R[3][1]) / S 
    q[3] = (R[2][3] + R[3][2]) / S
    q[4] = 0.25 * S
  end
  return q
end

function QuatMul2(Q1, Q2)
  local q = torch.DoubleTensor(4)
  local q1 = torch.DoubleTensor(4):copy(Q1)
  local q2 = torch.DoubleTensor(4):copy(Q2)
  q[1] = q2[1]*q1[1]-q2[2]*q1[2]-q2[3]*q1[3]-q2[4]*q1[4]
  q[2] = q2[1]*q1[2]+q2[2]*q1[1]+q2[3]*q1[4]-q2[4]*q1[3]
  q[3] = q2[1]*q1[3]-q2[2]*q1[4]+q2[3]*q1[1]+q2[4]*q1[2]
  q[4] = q2[1]*q1[4]+q2[2]*q1[3]-q2[3]*q1[2]+q2[4]*q1[1]
  return q
end

function QuatMul(Q1, Q2)  -- q = q1 x q2
  local q = torch.DoubleTensor(4)
  local q1 = torch.DoubleTensor(4):copy(Q1)
  local q2 = torch.DoubleTensor(4):copy(Q2)
  local a1 = q1[1]
  local b1 = q1[2]
  local c1 = q1[3]
  local d1 = q1[4]
  local a2 = q2[1]
  local b2 = q2[2]
  local c2 = q2[3]
  local d2 = q2[4]
  q[1] = a1*a2 - b1*b2 - c1*c2 - d1*d2
  q[2] = a1*b2 + b1*a2 + c1*d2 - d1*c2
  q[3] = a1*c2 - b1*d2 + c1*a2 + d1*b2
  q[4] = a1*d2 + b1*c2 - c1*b2 + d1*a2
  return q
end

function R2rpy(Rin)
  -- http://planning.cs.uiuc.edu/node102.html
  local R = torch.DoubleTensor(3, 3):copy(Rin)
  local y = math.atan2(R[2][1], R[1][1])
  local p = math.atan2(-R[3][1], math.sqrt(R[3][2]^2+R[3][3]^2))
  local r = math.atan2(R[3][2], R[3][3])
  local rpy = torch.DoubleTensor({r, p, y})
  return rpy
end

function rpy2Quat(rpyin)
  return R2Quat(rpy2R(rpyin))
end

function Quat2rpy(Qin)
  local q = torch.DoubleTensor(4):copy(Qin)
  local rpy = torch.DoubleTensor(3):fill(0)
  rpy[1] = math.atan2(2*(q[1]*q[2]+q[3]*q[4]), 1-2*(q[2]*q[2]+q[3]*q[3]))
  rpy[2] = math.asin(2*(q[1]*q[3]-q[4]*q[2]))
  rpy[3] = math.atan2(2*(q[1]*q[4]+q[2]*q[3]), 1-2*(q[3]*q[3]+q[4]*q[4]))
  return rpy
end

function rotX(gamma)
  -- http://planning.cs.uiuc.edu/node102.html
  local R = torch.DoubleTensor(3,3):fill(0)
  R[2][2] = math.cos(gamma)
  R[3][2] = math.sin(gamma) 
  R[1][1] = 1
  R[2][3] = -math.sin(gamma)
  R[3][3] = math.cos(gamma)
  return R
end

function rotY(beta)
  -- http://planning.cs.uiuc.edu/node102.html
  local R = torch.DoubleTensor(3,3):fill(0)
  R[1][1] = math.cos(beta)
  R[1][3] = math.sin(beta) 
  R[2][2] = 1
  R[3][1] = -math.sin(beta)
  R[3][3] = math.cos(beta)
  return R
end

function rotZ(alpha)
  -- http://planning.cs.uiuc.edu/node102.html
  local R = torch.DoubleTensor(3,3):fill(0)
  R[1][1] = math.cos(alpha)
  R[2][1] = math.sin(alpha)
  R[1][2] = -math.sin(alpha)
  R[2][2] = math.cos(alpha)
  R[3][3] = 1
  return R
end

function rpy2R(rpy)
  -- http://planning.cs.uiuc.edu/node102.html
  local R = torch.DoubleTensor(3,3):fill(0)
  local alpha = rpy[3]
  local beta = rpy[2]
  local gamma = rpy[1]
  R[1][1] = math.cos(alpha) * math.cos(beta)
  R[2][1] = math.sin(alpha) * math.cos(beta)
  R[3][1] = -math.sin(beta)
  R[1][2] = math.cos(alpha) * math.sin(beta) * math.sin(gamma) - math.sin(alpha) * math.cos(gamma)
  R[2][2] = math.sin(alpha) * math.sin(beta) * math.sin(gamma) + math.cos(alpha) * math.cos(gamma)
  R[3][2] = math.cos(beta) * math.sin(gamma)
  R[1][3] = math.cos(alpha) * math.sin(beta) * math.cos(gamma) + math.sin(alpha) * math.sin(gamma)
  R[2][3] = math.sin(alpha) * math.sin(beta) * math.cos(gamma) - math.cos(alpha) * math.sin(gamma)
  R[3][3] = math.cos(beta) * math.cos(gamma)
  return R
end

function pdcheck(A)
  local e = torch.symeig(A)
  for i = 1, e:size(1) do
    if math.abs(e[i]) < 1e-6 then e[i] = 0; end
    if e[i] < 0 then
      print(A)
      error('Not positive definite matrix'.."("..i..","..e[i]..")")
    end
  end
end
function cholesky(A)
  -- http://rosettacode.org/wiki/Cholesky_decomposition
  pdcheck(A)
  local m = A:size(1)
  local L = torch.DoubleTensor(A:size(1), A:size(1)):fill(0)
  for i = 1, m do
    for k = 1, i do
      local sum = 0
      for j = 1, k do
        sum = sum + L[i][j] * L[k][j]
      end
      if i == k then
        L[i][k] = math.sqrt(A[i][i] - sum)
      else
        L[i][k] = 1 / L[k][k] * (A[i][k] - sum)
      end
    end
  end
  return L
end

function det(A)
  local A = torch.lu(A)
  local dim = A:size(1)
  local d = 1
  for i = 1, dim do
    d = d * A[i][i]
  end
  return d
end

function QuatMean(QMax, qInit)
  local qIter = torch.DoubleTensor(4):copy(qInit)
  local e = torch.DoubleTensor(3, QMax:size(2)):fill(0)
  local iter = 0
  local diff = 0
  repeat
    iter = iter + 1
    for i = 1, QMax:size(2) do
      local ei = e:narrow(2, i, 1):fill(0)
      local qi = QMax:narrow(2, i, 1)
      local eQ = QuatMul(qi, QuatInv(qIter))
      ei:copy(Quat2Vector(eQ))
    end
    local eMean = torch.mean(e,2)
    local qIterNext = QuatMul(Vector2Quat(eMean), qIter)
    diff = QuatCompare(qIterNext, qIter)
    qIter:copy(qIterNext)
  until diff < 1e-6
  return qIter, e
end
