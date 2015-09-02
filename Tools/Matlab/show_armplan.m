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

figure(4);
b = bar(rad2deg(abs([qq0(end,:)' - qGoal, qq(end,:)' - qGoal])));
b(1).FaceColor = 'r';
b(2).FaceColor = 'b';
title('qEnd vs qGoal (Absolute difference)');

drawnow;