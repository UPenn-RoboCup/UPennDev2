%% Test the iKine and fKine within the ranges
a_u = .5;
a_l = .5;
all_diffs = [];
bad = 0;
good = 0;
% y must be positive
for y=0:.1:1
    for x=-1:.1:1
        for z=-1:.1:1
            theta = iKine( [x y z] );
            coords = fKine( theta );
            diff = [x y z] - coords;
            if( ~isnan(coords) )
                all_diffs = [all_diffs ; diff];
                c = sqrt(x^2+y^2+z^2);
                if( ~isreal(coords) || ~isreal(theta) )
                    bad = bad+1;
                    initc = [x y z]
                    theta
                    %t_1 = (c^2-a_u^2-a_l^2)/(-2*a_u*a_l)
                    %t_2 = a_u+a_l*cos(theta(3))
                    coords
                else
                    good = good+1;
                end
            else
                %disp('unreachable')
            end
        end
    end
end

bad_ratio = bad / (bad+good)

figure(1);
clf;
plot(all_diffs.^2,'*')
