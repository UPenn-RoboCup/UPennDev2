% sampling frequency
Fs = 16000;
% sample time
T = 1/Fs;
% length of signal
L = 512;
%L = 341;
% time vector
t = [0:L-1]*T;

% touch tone frequencies
symbols = [ '1' '2' '3' 'A'; ...
            '4' '5' '6' 'B'; ...
            '7' '8' '9' 'C'; ...
            '*' '0' '#' 'D'];

fRow = [ 697  770  852  941];  
fCol = [1209 1336 1477 1633];

symbol = '5';
[r c] = find(symbols == symbol);
h1 = fRow(r);
h2 = fCol(c);
% sum of a h1 Hz sinusoid and a h2 Hz sinusoid
x = 100 * sin(2*pi*h1*t) + 100 * sin(2*pi*h2*t);

%plot(gca, Fs*t(1:50),x(1:50));

% Next power of 2 from length of y
NFFT = 2^nextpow2(L); 
Y = fft(x, NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2+1);


[xr xi] = myfft(x);
plot(gca, xr);
mydtmf(x, x);

