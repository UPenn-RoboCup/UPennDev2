
% (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
% ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
% University of Pennsylvania

function r = rotz(t)

% Homogeneous transformation representing a rotation of theta
% about the Z axis.

ct = cos(t);
st = sin(t);
r =    [ct	-st	0	0
        st	ct	0 0
        0	   0	1	0
        0	   0	0	1];
