ob_x = obser(1:80);
ob_y = obser(81:160);
ob_theta = zeros(1,80);
for i = 1 : 80
  ob_theta(1,i) = atan2(ob_x(i), ob_y(i));
end

plot(ob_x, ob_y, '*');

grid on;
%set(gca, 'xtick', [-0.2:0.02:0.8]);
%set(gca, 'ytick', [-0.5:0.02:0.5]);
axis([-0.2,0.8,-0.5,0.5]);
hold on;

sigma_x = 0.01;
sigma_y = 0.01;
A = 1;
ob_a = cos(ob_theta).^2./2./sigma_x.^2 + sin(ob_theta).^2./2./sigma_y.^2;
ob_b = -sin(2*ob_theta)./4./sigma_x.^2 + sin(2*ob_theta)./4./sigma_y.^2 ;
ob_c = sin(ob_theta).^2./2./sigma_x.^2 + cos(ob_theta).^2./2./sigma_y.^2;

pt_idx = 1;
for x = -0.2 : 0.02 : 0.8 
  for y = -0.5 : 0.02 : 0.5
    max_p = 0;
    for k = 1 : 80
      Z = A*exp( - (ob_a(k)*(x-ob_x(k)).^2 + ...
                2*ob_b(k)*(x-ob_x(k)).*(y-ob_y(k)) + ...
                ob_c(k)*(y-ob_y(k)).^2));
      max_p = max(max_p, Z);
    end
    if max_p > 0.5001
      point(1, pt_idx) = y;
      point(2, pt_idx) = x;
      point(3, pt_idx) = max_p;
      pt_idx = pt_idx + 1;
    end
  end
end

plot(point(2,:), point(1,:), 'o');


%{
A = 1;
x0 = 0; y0 = 0;
 
sigma_x = 0.5;
sigma_y = 1;
 
for theta = 0:pi/100:pi
  a = cos(theta)^2/2/sigma_x^2 + sin(theta)^2/2/sigma_y^2;
  b = -sin(2*theta)/4/sigma_x^2 + sin(2*theta)/4/sigma_y^2 ;
  c = sin(theta)^2/2/sigma_x^2 + cos(theta)^2/2/sigma_y^2;
   
  [X, Y] = meshgrid(-0.5:0.02:0.5, -0.5:.02:0.5);
  Z = A*exp( - (a*(X-x0).^2 + 2*b*(X-x0).*(Y-y0) + c*(Y-y0).^2)) ;
  surf(X,Y,Z);shading interp;view(-36,36);axis equal;drawnow
end
%}
