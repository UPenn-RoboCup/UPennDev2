function [] = plot_horizon( horizon, scale )
hold on;
if (scale == 4)
    % labelB
    plot(horizon.hXB,horizon.hYB,'m--');
else
    % labelA
    plot(horizon.hXA,horizon.hYA,'m--');
end
hold off;
end