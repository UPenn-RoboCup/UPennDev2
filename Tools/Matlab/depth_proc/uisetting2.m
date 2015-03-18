%  - ui (user input) 
%       ui.runningMode   log(1)  |  execute(2)
%       ui.taskMode      Ground(1) | Wall(2) | Table(3) | Step-able planes(4) |
%                        Large Planes(11) 
%                        Manual (101)
%       ui.clickType     2d(2)   | 3D(3)
%       ui.clickxy       [x,y] on 2D image
%       ui.figures       [1]fig1:raw depth(1) | med filt depth(2) 
%                        [2]fig2: 
%                        [3]fig3: subsampled points in 3D ... 
ui.runningMode = 2;
ui.reset = 0;
ui.taskMode = 101;
ui.clickType = 2;
ui.clickxy = [];
ui.figures = [0 0 1];
ui.figures_range = [-2 2 -2 2 -1 4];
ui.undistortDepth = 0;


