function [distance, yaw, idx] = localizeDoor_v1L(Planes)
% should be exactly same to localizeDoor_v1K
Np = numel(Planes);
distance = []; % error
yaw = [];
idx = 0;
if Np < 1
    return;
elseif Np == 1
    if abs(Planes{1}.Normal(3)) < 0.3
        distance = abs(Planes{1}.Center'*Planes{1}.Normal);
        idx = 1;
        yaw = -atan2(Planes{1}.Normal(2),-Planes{1}.Normal(1))
   
    end
else
    D = [];
    L = [];
    Yaw = [];
    n = 0;
    for k=1:Np
        if abs(Planes{k}.Normal(3)) < 0.3 && (abs(Planes{k}.Normal(1)) > abs(Planes{k}.Normal(2))) 
            n = n +1;
            D(n) = abs(Planes{k}.Center'*Planes{k}.Normal);
            Yaw(n) = -atan2(Planes{k}.Normal(2),-Planes{k}.Normal(1));
            L(n) = norm(Planes{k}.Center(1:2));
        end
    end  
    
    if n == 1
        distance = D;
        yaw = Yaw;
    elseif n > 1 % take the two closest ones
        [~,idx] = sort(L,'ascend');    
        distance = D(idx(1:2));
        yaw = Yaw(idx(1:2));
    end
    
end
   
end
            