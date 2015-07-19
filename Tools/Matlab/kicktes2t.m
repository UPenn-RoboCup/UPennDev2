close all;




%walkkick #2
breakTX = [0     0.07    0.1   0.4  0.75   1];
breakX = [-16.8 -21.47  -20    18    -30 -27];

breakTY = [0    0.2  0.8    1];
breakY = [33.11  60  15  33];

breakTA = [0  0.10     0.2 0.27  0.8    1];
breakA = [-16.2 -27.66 -37 -39    0     5];





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

figure(2);
subplot(2,2,1);
plot(kick(:,1),kick(:,2)*180/pi)

subplot(2,2,2);
plot(kick(:,1),kick(:,3)*180/pi)

subplot(2,2,3);
plot(kick(:,1),kick(:,4)*180/pi)