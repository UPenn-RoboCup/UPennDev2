function flag = plane_dist( taskNum, normal, position, size )

% plane models 
persistent Mground
persistent Mwall
persistent Mtable
persistent Mstepable

if isempty(Mground)
    loadPlaneModels;
end

flag = true;
dist = 0;
dist_ = 0;
if taskNum < 1
    return;
end
if nargin >= 2 % consider normal 
    if taskNum ==1, % ground        
        dist_ = 1 - Mground.mu_normal*normal;
        flag = (dist_ < Mground.sig_normal);
    elseif taskNum == 2, % wall
        dist_ = 1 - Mwall.mu_normal_z*normal(3);
        flag = (dist_ < Mwall.sig_normal);
    elseif taskNum == 3, % table
        dist_ = 1 - Mtable.mu_normal*normal;
        flag = (dist_ < Mtable.sig_normal);
    elseif taskNum == 4, % stepable
        dist_ = 1 - Mstepable.mu_normal*normal;
        flag = (dist_ < Mstepable.sig_normal);
    end
end

if nargin >= 3 &&  flag ==1 % consider location 
    dist_ = 0;
    if taskNum ==1, % ground        
        dist_ = Mground.mu_height - position(3);  
        flag = abs(dist_) < Mground.sig_height; 
    elseif taskNum == 3, % table
        dist_ = Mtable.mu_height - position(3);  
    end
  
end

% if nargin == 4
%     
% end    

end

