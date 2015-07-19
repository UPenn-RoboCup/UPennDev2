close all;




%walkkick #2
breakTX = [0      0.10     0.30 0.34   0.5  1];
breakX = [-16.72  -16.22   25 25        0 -21.95]

breakTY = [0     0.14    0.45  0.6  0.8  1];
breakY = [33.61   52  52       48   15 18.04];

breakTA = [0      0.12  0.20  0.50  0.75     1];
breakA = [-16.89 -40   -40    0      2 4];





p=csapi(breakTX,breakX);
coefX = p.coefs;

q=csapi(breakTY,breakY);
coefY = q.coefs;

r=csapi(breakTA,breakA);
coefA = r.coefs;



%Print in lua syntax
str1='local breaksTX={';
for i=2:size(p.breaks,2) str1=[str1 sprintf('%f,',p.breaks(i))]; end
str1 = [str1 '}'];
disp(str1);

str1='local breaksTY={';
for i=2:size(q.breaks,2) str1=[str1 sprintf('%f,',q.breaks(i))]; end
str1 = [str1 '}'];
disp(str1);

str1='local breaksTA={';
for i=2:size(r.breaks,2) str1=[str1 sprintf('%f,',r.breaks(i))]; end
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

disp('local coefsA={');
for i=1:size(coefA,1)  
  str2='  {';
  for j=1:size(coefA,2) str2=[str2 sprintf('%f,', coefA(i,j))]; end
  str2=[str2 '},'];
  disp(str2);
end
disp('}');

figure(1);
subplot(2,2,1);
fnplt(p);
ylabel('x');
xlabel('t');

subplot(2,2,2);
fnplt(q);
ylabel('y');
xlabel('t');

subplot(2,2,3);
fnplt(r);
ylabel('a');
xlabel('t');
%
figure(2);
subplot(2,2,1);
plot(kick(:,1),kick(:,2)*180/pi)

subplot(2,2,2);
plot(kick(:,1),kick(:,3)*180/pi)

subplot(2,2,3);
plot(kick(:,1),kick(:,4)*180/pi)
%}