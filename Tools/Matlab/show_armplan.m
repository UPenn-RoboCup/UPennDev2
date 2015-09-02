%% Plot the joint trajectories in time
qq = reshape(q, [nq, np])';
qq0 = reshape(qPath0, [nq, np])';
figure(1);
plot(rad2deg(qq0));
xlim([1, np]);

figure(2);
plot(rad2deg(qq));
xlim([1, np]);

figure(3);
plot(rad2deg(qq-qq0));
xlim([1, np]);
title('Difference in Degrees');

drawnow;