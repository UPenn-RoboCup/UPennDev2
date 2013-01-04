
% (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
% ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
% University of Pennsylvania

function t = trans(v)

t = [1 0 0 v(1)
     0 1 0 v(2)
     0 0 1 v(3)
     0 0 0  1  ];
   