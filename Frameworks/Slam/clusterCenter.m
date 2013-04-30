function [xc, yc] = clusterCenter(r, th, istart, iend)

xr = r.*cos(th);
yr = r.*sin(th);

i1 = floor(.5*(istart+iend));
i2 = ceil(.5*(istart+iend));

xc = .5*(xr(i1) + xr(i2));
yc = .5*(yr(i1) + yr(i2));

