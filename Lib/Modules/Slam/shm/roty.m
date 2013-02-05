function r = roty(t)

% Homogeneous transformation representing a rotation of theta
% about the y axis.

ct = cos(t);
st = sin(t);
r =    [ct	0	 st	 0
        0	  1	 0	 0
        -st	0  ct	 0
        0	  0	 0	 1];
