% function res = depth_proc(data, meta, ui)
% 
% INPUT 
%  - data 
%  - meta 
%  - ui (user input) See uisetting.m
%    
% OUTPUT 
%  - res (result)
%
% by Bhoram Lee 
function res = depth_proc(data, meta, ui)

if isempty(data)
    return
end

switch ui.runningMode 
    case 1, % logging 
        %logDepthData(data, meta);
    case 2, 
        res = detectPlanes(data, meta, ui);
end
    
end