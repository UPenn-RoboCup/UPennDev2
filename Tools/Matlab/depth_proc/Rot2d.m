function R = Rot2d( theta )
c = cos(theta);
s = sin(theta);
R = [c -s; s c];
end

