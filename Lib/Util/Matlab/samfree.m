%%
seq = {};
seq.x = Ax;
seq.y = Ay;
seq.r = sqrt(Ax.^2+Ay.^2);
seq.a = atan(Ay./Ax);

interval = 2.5 * pi / 180;

pts = seq.a(1)/abs(seq.a(1))*ceil(abs(seq.a(1))/interval);
pte = seq.a(end)/abs(seq.a(end))*floor(abs(seq.a(end))/interval);

%%
nseq = 80;

idx = nseq;
r = zeros(1,pts-pte+1);
a = zeros(1,pts-pte+1);
ridx = 1;
while (pte <= pts)
    for i = idx : -1 : 2
        rad = pte * 2.5 * pi /180;
        if (seq.a(i) <= rad) && (seq.a(i-1) >= rad)
            break;
        end
    end
    midval = (seq.r(i-1) + seq.r(i))/2;
    disp([i,seq.a(i-1),rad,seq.a(i),midval]);
    a(ridx) = rad;
    r(ridx) = midval;
    ridx = ridx + 1;
    pte = pte + 1;
    idx = i - 1;
end