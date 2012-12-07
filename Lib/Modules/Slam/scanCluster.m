function [istart, iend] = scanCluster(x, threshold, nmin)

if nargin < 3,
  nmin = 1;
end

if nargin < 2,
  threshold = 1.0;
end

x = x(:);
x(x==0) = inf;
nx = length(x);

dx = x(2:end) - x(1:end-1);
ix = [0; abs(dx) < threshold; 0];

istart = find(ix(2:end) & ~ix(1:end-1));
iend = find(~ix(2:end) & ix(1:end-1));

% Remove small clusters:
ivalid = (iend-istart >= nmin);
istart = istart(ivalid);
iend = iend(ivalid);
