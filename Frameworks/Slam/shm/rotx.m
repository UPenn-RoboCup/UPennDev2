function r = rotx(t)

% Homogeneous transformation representing a rotation of theta
% about the X axis.

ct = cos(t);
st = sin(t);
r =    [1	0	 0	 0
        0	ct -st 0
        0	st ct	 0
        0	0	 0	 1];
