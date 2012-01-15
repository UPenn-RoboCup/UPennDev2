function [] = plot_freespace( free, scale )
% TODO: Show freespace boundary in labelB
hold on;
if (scale == 4)
    X = free.Bx;
    Y = free.By;
else
    X = free.Ax;
    Y = free.Ay;
end
plot(X,Y,'m--','LineWidth',2);
%plot(free.hDirX,free.hDirY+free.h,'r--','LineWidth',2);
hold off;
end