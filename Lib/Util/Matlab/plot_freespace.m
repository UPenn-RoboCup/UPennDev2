function [] = plot_freespace( free, scale )
% TODO: Show freespace boundary in labelB
hold on;
plot(free.x,free.y,'m--','LineWidth',2);
hold off;
end