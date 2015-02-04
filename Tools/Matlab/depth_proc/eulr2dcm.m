% by Bhoram Lee 
%
% Reference
% Titterton & Weston, "Strapdown Inertial Navigation Technology", 2nd. ed.,
% 2004.
function [ dcm ] = eulr2dcm( eulr )

N = size(eulr,2);
dcm = zeros(3,3,N);
for k=1:N
   dcm(1,1,k) = cos(eulr(2))*cos(eulr(3));
   dcm(1,2,k) = cos(eulr(2))*sin(eulr(3));
   dcm(1,3,k) = -sin(eulr(2));
   
   dcm(2,1,k) = sin(eulr(1))*sin(eulr(2))*cos(eulr(3)) - cos(eulr(1))*sin(eulr(3));
   dcm(2,2,k) = sin(eulr(1))*sin(eulr(2))*sin(eulr(3)) + cos(eulr(1))*cos(eulr(3));
   dcm(2,3,k) = sin(eulr(1))*cos(eulr(2));
   
   dcm(3,1,k) = cos(eulr(1))*sin(eulr(2))*cos(eulr(3)) + sin(eulr(1))*sin(eulr(3));
   dcm(3,2,k) = cos(eulr(1))*sin(eulr(2))*sin(eulr(3)) - sin(eulr(1))*cos(eulr(3));
   dcm(3,3,k) = cos(eulr(1))*cos(eulr(2));  
   
end

end

