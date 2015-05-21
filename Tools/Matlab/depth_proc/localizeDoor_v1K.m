function [distance, yaw, idx] = localizeDoor_v1K(Planes)
% exactly same to localizeDoor_v1L
Np = numel(Planes);
distance = -1.0; % error
yaw = 0;
ori = 0;
if Np < 1
    return;
elseif Np == 1
    if abs(Planes{1}.Normal(3)) < 0.1
        distance = abs(Planes{1}.Center'*Planes{1}.Normal);
        idx = 1;
        yaw = -atan2(Planes{1}.Normal(2),-Planes{1}.Normal(1))*180/pi
   
    end
else
    D = zeros(1,Np);
    l = zeros(1,Np);
    for k=1:Np
        D(k) = abs(Planes{k}.Center'*Planes{k}.Normal);
        l(k) = norm(Planes{k}.Center(1:2));
    end
    [~,idx] = min(l);
    distance = D(idx);
    yaw = -atan2(Planes{idx}.Normal(2),-Planes{idx}.Normal(1))*180/pi
                  
end
   
end
            