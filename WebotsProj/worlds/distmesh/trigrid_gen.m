pv = [-2 3; 2 3; 2 -3; -2 -3; -2 3];
distance = 1.0
[p,t]=distmesh2d(@dpoly,@huniform,distance,[-2,-3; 2,3],pv,pv);
save(strcat('triangle_grid',num2str(distance),'.txt'), 'p', '-ASCII')
