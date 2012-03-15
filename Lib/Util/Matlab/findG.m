[~,mDis] = size(flag);
idx = 1;
nGau = 0;
inGau = false;

% first item

while idx <= mDis
    if (flag(idx)==1) 
        disp('find 1');
        for cnt = (idx+1):mDis
            if (flag(cnt)~=1)
                disp('condition2');
                break;                
            end
        end
        if (cnt == mDis) 
            cnt = cnt + 1;
        end
        disp(cnt-idx);
        idx = cnt;
    else
        idx = idx + 1;
    end
end

%%

a = -100; b = 100;
x = a + (b-a) * rand(1, 200);
m = (a + b)/2;
s = 5; 

f = gauss_distribution(x, m, s);
x = x./100;
f = 1 - f./max(f);
figure;plot(x,f,'o');
figure;polar(x.*(pi/4),f,'o');
grid on;
title('Bell Curve');
xlabel('Randomly produced numbers');
ylabel('Gauss Distribution');




%Then, we plot this information using our bell curve: 




