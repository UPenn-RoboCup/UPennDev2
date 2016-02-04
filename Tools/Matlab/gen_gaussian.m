mu = [0,0]; %// data

sigma = [1 0; 0 .5]; %// data
x = -5:.1:5; %// x axis
y = -4:.1:4; %// y axis

[X, Y] = meshgrid(x,y); %// all combinations of x, y
Z = mvnpdf([X(:) Y(:)],mu,sigma); %// compute Gaussian pdf
Z = reshape(Z,size(X)); %// put into same size as X, Y
%// contour(X,Y,Z), axis equal  %// contour plot; set same scale for x and y...
%surf(X,Y,Z) %// ... or 3D plot

theta = 90*pi/180;
rotate = [cos(theta), -sin(theta);
    sin(theta), cos(theta)];
C = contour(X, Y, Z, 1);
% Strange first coord...
C1 = rotate * C(:,2:end);
%axis equal;
%plot(C1(1,:), C1(2,:))

%% Plot the path of the ball
th = linspace(-pi, pi);
x = cos(th);
y = sin(th);
c = [x; y];

n = size(ball, 2);
sel = 1:n;

figure(4);
clf;
plot(ball(1, sel), ball(2, sel), 'ro',...
    'MarkerEdgeColor', [0.5 0 0], 'MarkerFaceColor', 'r');
hold on;
for i=sel
    xb = ball(1, i);
    yb = ball(2, i);
    [theta, rho] = cart2pol(xb, yb);
    sc = [rho; max(min(0.1, rho*tan(theta)), 1)];
    e = 0.05 * sc .* c;
    rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    e = rot * e;
    ex = e(1, :);
    ey = e(2, :);
    plot(ex + xb, ey + yb, 'b--');
end
hold off;
title('Ball Position tracks');
xlabel('X (meters)');
ylabel('Y (meters)');
axis equal;