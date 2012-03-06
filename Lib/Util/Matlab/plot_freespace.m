function [] = plot_freespace( free, scale )
% TODO: Show freespace boundary in labelB
hold on;
if (scale == 4)
    X = free.Bx;
    Y = free.By;
    plot(X,Y,'m--','LineWidth',2);
else
    %X = free.Ax;
    %Y = free.Ay;
end

hold off;
end