% function res = depth_proc(data, meta, ui)
% 
% INPUT 
%  - data 
%  - meta 
%  - ui (user input) 
%       ui.runningMode   log(1)  |  execute(2)
%       ui.taskMode      Ground(1) | Wall(2) | Table(3) | Step-able planes(4)
%       ui.click         [x,y] on 2D image
%       ui.figures       fig1: raw depth 
%                        fig2: med filt depth
%                        fig3: ... 
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
        detectPlanes(data, meta, ui);
end
    
end