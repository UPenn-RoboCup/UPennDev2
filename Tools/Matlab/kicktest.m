%X coordinates for kicks

breakTX = [0  0.1   0.3 0.6 0.7 0.8   0.9 1];
breakX = [0 -0.2    -1  2  2.1  2    1.3 1];
breakTY = [0  0.1   0.3   0.5    0.7 0.8   0.9   1];
breakY = [0   0.7    1     1     0.9   0.7   0.3     0];


%For walkkick
breakTX = [0   0.3 0.6 0.7 0.8   0.9 1];
breakX = [0     0.7   1.5  1.5  1.45   1.15 1];
breakTY = [0    0.3   0.5    0.7 0.8   0.9   1];
breakY = [0      0.9     1     0.9   0.7   0.4     0];



%For walking
breakTX = [0 0.2 0.5 0.75 0.8 1];
breakX = [0 0.09  0.5 0.9 0.945 1];
breakTY = [0    0.3   0.5      0.7 0.8  0.9  1];
breakY = [0      0.9   1       0.7 0.35  0.085  0];

%walking #2
breakTX = [0 0.15 0.35 0.6 0.8 1];
breakX = [0 0.09  0.4 0.8 0.97 1];

breakTY = [0    0.3   0.6   0.7  0.8  0.9  1];
breakY = [0      0.9   1    0.9  0.65  0.22  0];


breakTY = [0 0.2  0.4  0.5 0.6   0.8  0.9 1];
breakY = [0  0.38 0.95   1  0.9   0.25  0.07 0];






p=csapi(breakTX,breakX);
coefX = p.coefs;

q=csapi(breakTY,breakY);
coefY = q.coefs;

%Print in lua syntax
str1='local breaksTX={';
for i=2:size(p.breaks,2) str1=[str1 sprintf('%f,',p.breaks(i))]; end
str1 = [str1 '}'];
disp(str1);

str1='local breaksTY={';
for i=2:size(q.breaks,2) str1=[str1 sprintf('%f,',q.breaks(i))]; end
str1 = [str1 '}'];
disp(str1);


disp('local coefsX={');
for i=1:size(coefX,1)  
  str2='  {';
  for j=1:size(coefX,2) str2=[str2 sprintf('%f,', coefX(i,j))]; end
  str2=[str2 '},'];
  disp(str2);
end
disp('}');

disp('local coefsY={');
for i=1:size(coefY,1)  
  str2='  {';
  for j=1:size(coefY,2) str2=[str2 sprintf('%f,', coefY(i,j))]; end
  str2=[str2 '},'];
  disp(str2);
end
disp('}');

subplot(2,2,1);
fnplt(p);
ylabel('x');
xlabel('t');

subplot(2,2,2);
fnplt(q);
ylabel('y');
xlabel('t');


subplot(2,2,3);
px = fnval(p,[0:0.01:1]);
py = fnval(q,[0:0.01:1]);
plot(px,py);