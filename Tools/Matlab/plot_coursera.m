%% Meta parameters
mu = 0;
sigma = 1;

%% Generate
pd = makedist('Normal',mu,sigma);
x = -4*sigma : sigma/10 : 4*sigma;
y = pdf(pd,x);

%% Plot
figure(1);
clf;
plot(x, y, 'b-', 'LineWidth', 3);
ax = gca;
%ax.Box = 'off';
%ax.Color = 'none';
ax.Visible = 'off';

saveas(gcf, '~/Desktop/gaussian', 'png');