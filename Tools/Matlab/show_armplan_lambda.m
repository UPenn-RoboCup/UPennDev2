%% Plot
dlambda0 = reshape(dlambda0, [nNull, np]);
dlambda = reshape(dlambda, [nNull, np]);
figure(1);
plot(dlambda0);
figure(2);
plot(dlambda);
figure(3);
plot(dlambda - dlambda0);

drawnow;