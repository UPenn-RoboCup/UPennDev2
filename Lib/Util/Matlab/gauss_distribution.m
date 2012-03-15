function f = gauss_distribution(x, mu, s)
p1 = -.5 * ((x - mu)/s) .^ 2;
p2 = (s * sqrt(2*pi));
f = exp(p1) ./ p2; 

a = -100; b = 100;
x = a + (b-a) * rand(1, 500);
m = (a + b)/2;
s = 30; 