function [ A, Inliers ] = estimatePlane_useall( x, y, z_ )
    
persistent required_itr;
A = zeros(4,1);
N = length(x);
itr = 0;
if isempty(required_itr)
    required_itr = ceil(log(1-0.99)/log(1-(1-0.15)^3));  
end

thre = 0.013^2;
maxnum_inliers = 0;
Inliers = [];
ONES = ones(N,1);
while itr < required_itr     
     idx = randperm(N,3);
     
     A_ = [x(idx)' y(idx)' ones(3,1)] ;
     b_ = z_(idx)';
       
     if cond(A_) < 1e3
         a = A_\b_;

         dist = ([x' y' ONES ]*a - z_');
         sqdist = dist.*dist;
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
A_ = [x(Inliers)' y(Inliers)' ones(maxnum_inliers,1)] ;
b_ = z_(Inliers)';       
A = A_\b_;

end

