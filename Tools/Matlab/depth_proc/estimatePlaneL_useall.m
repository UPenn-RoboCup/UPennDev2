function [ A, Inliers ] = estimatePlaneL_useall( x, y, z )
    
persistent required_itr;
A = zeros(4,1);
N = length(x);
itr = 0;
if isempty(required_itr)
    required_itr = ceil(log(1-0.995)/log(1-(1-0.1)^3));  
end

thre = 0.01^2;
maxnum_inliers = 0;
Inliers = [];
ONES = ones(N,1);
while itr < required_itr     
     idx = randperm(N,3);
     
     A_ = [x(idx)' y(idx)' z(idx)' ones(3,1)] ;
     
     [U, S] = svd(A_');
       
     if S(end,end) < 5e-2
         a = U(:,4);

         res = ([x' y' z' ONES]*a);
         sqdist = res.*res;
         inliers = find(sqdist < thre);
         numinliers = length(inliers);

         if numinliers > maxnum_inliers
             A = a;
             maxnum_inliers = numinliers;
             Inliers = inliers;
         end    
     end
     itr = itr + 1;
end

% refinement 
A_ = [x(Inliers)' y(Inliers)' z(Inliers)' ones(maxnum_inliers,1)] ;
[A, S] = svd(A_');
A = A(:,4);

end

