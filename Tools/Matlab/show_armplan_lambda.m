%% Plot
dlambda0 = reshape(dlambda0, [nNull, np]);
dlambda = reshape(dlambda, [nNull, np]);
figure(11);
plot(dlambda0);
title('Original lambda');

figure(12);
plot(dlambda);
title('Optimized lambda');

%figure(13);
%plot(dlambda - dlambda0);

drawnow;