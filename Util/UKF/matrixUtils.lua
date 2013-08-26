require 'torch'

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

function GaussianPDF(x, mean, cov)
--  print(x)
--  print(mean)
--  print(cov)
  local vectorSize = x:size(1)
  local Diff = torch.DoubleTensor(vectorSize,1):copy(x - mean)
  local Cov = torch.DoubleTensor(vectorSize, vectorSize):copy(cov)
  local exp = torch.exp((Diff:t() * torch.inverse(Cov) * Diff):mul(-0.5))
  local detCov = torch.det(Cov)
  local const = (2 * math.pi)^(-vectorSize/2) / math.sqrt(detCov)
  local pdf = exp:mul(const)
  return pdf  
end


