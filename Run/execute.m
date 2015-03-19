function Rleg_U = execute(s,rpy)
file = fopen(s,'r');
file2 = fopen(rpy,'r');

data = fscanf(file,'%f');
rpy  = fscanf(file2,'%f');

n = length(data);
n2 = length(rpy);
rownum = 67;
Rleg_U= zeros(n/rownum, 4);
Rleg_L= zeros(n/rownum, 4);
RFoot= zeros(n/rownum, 4);
Lleg_U= zeros(n/rownum, 4);
Lleg_U= zeros(n/rownum, 4);
LFoot= zeros(n/rownum, 4);
Rarm_U= zeros(n/rownum, 4);
Rarm_AB= zeros(n/rownum, 4);
Rarm_L= zeros(n/rownum, 4);
Rarm_Wr= zeros(n/rownum, 4);
Larm_U= zeros(n/rownum, 4);
Larm_AB= zeros(n/rownum, 4);
Larm_L= zeros(n/rownum, 4);
Larm_Wr= zeros(n/rownum, 4);
Torso= zeros(n/rownum, 4);
Pelvis= zeros(n/rownum, 4);


stage = zeros(n2/4,1);
r = zeros(n2/4,1);
p = zeros(n2/4,1);
y = zeros(n2/4,1);

for i=1:n/rownum
    for j=1:3
        Rleg_U(i,j) = data(rownum*(i-1)+22*(j-1)+1+1);
        Rleg_L(i,j) = data(rownum*(i-1)+22*(j-1)+2+1);
        RFoot(i,j)  = data(rownum*(i-1)+22*(j-1)+3+1);
        Lleg_U(i,j) = data(rownum*(i-1)+22*(j-1)+7+1);
        Lleg_U(i,j) = data(rownum*(i-1)+22*(j-1)+8+1);
        LFoot(i,j)  = data(rownum*(i-1)+22*(j-1)+9+1);
        Rarm_U(i,j) = data(rownum*(i-1)+22*(j-1)+13+1);
        Rarm_AB(i,j)= data(rownum*(i-1)+22*(j-1)+14+1);
        Rarm_L(i,j) = data(rownum*(i-1)+22*(j-1)+15+1);
        Rarm_Wr(i,j)= data(rownum*(i-1)+22*(j-1)+16+1);
        Larm_U(i,j) = data(rownum*(i-1)+22*(j-1)+17+1);
        Larm_AB(i,j)= data(rownum*(i-1)+22*(j-1)+18+1);
        Larm_L(i,j) = data(rownum*(i-1)+22*(j-1)+19+1);
        Larm_Wr(i,j)= data(rownum*(i-1)+22*(j-1)+20+1);
        Torso(i,j)  = data(rownum*(i-1)+22*(j-1)+21+1);
        Pelvis(i,j) = data(rownum*(i-1)+22*(j-1)+22+1);
    end
    j=4;
    Rleg_U(i,j) = data(rownum*(i-1)+1);
    Rleg_L(i,j) = data(rownum*(i-1)+1);
    RFoot(i,j)  = data(rownum*(i-1)+1);
    Lleg_U(i,j) = data(rownum*(i-1)+1);
    Lleg_U(i,j) = data(rownum*(i-1)+1);
    LFoot(i,j)  = data(rownum*(i-1)+1);
    Rarm_U(i,j) = data(rownum*(i-1)+1);
    Rarm_AB(i,j)= data(rownum*(i-1)+1);
    Rarm_L(i,j) = data(rownum*(i-1)+1);
    Rarm_Wr(i,j)= data(rownum*(i-1)+1);
    Larm_U(i,j) = data(rownum*(i-1)+1);
    Larm_AB(i,j)= data(rownum*(i-1)+1);
    Larm_L(i,j) = data(rownum*(i-1)+1);
    Larm_Wr(i,j)= data(rownum*(i-1)+1);
    Torso(i,j)  = data(rownum*(i-1)+1);
    Pelvis(i,j) = data(rownum*(i-1)+1);
end

for i=1:(n2/4)
    
    stage(i) = rpy(4*(i-1)+1);
    r(i) = rpy(4*(i-1)+2)/pi*180;
    p(i) = rpy(4*(i-1)+3)/pi*180;
    y(i) = rpy(4*(i-1)+4)/pi*180;
end
stageN = max(Rleg_U(:,4));
m = zeros(1,stageN);
sum = 0;

figure(1)
for i=1:stageN
    subplot(5,2,i)
    m(i) = length(find(Rleg_U(:,4)==i));
    plot3(Rleg_U(sum+1:sum+m(i),1),Rleg_U(sum+1:sum+m(i),2),Rleg_U(sum+1:sum+m(i),3),'*','color',rand(1,3));
    sum = sum + m(i);
end

grid on
xlabel('x')
ylabel('y')
zlabel('z')
hold off
sum = 0;
figure(2)
for i=1:stageN
    m(i) = length(find(Rleg_U(:,4)==i));
    plot3(Rleg_U(sum+1:sum+m(i),1),Rleg_U(sum+1:sum+m(i),2),Rleg_U(sum+1:sum+m(i),3),'*','color',rand(1,3));
    sum = sum + m(i);
    hold on
end
grid on
xlabel('x')
ylabel('y')
zlabel('z')
hold off

figure(3)
subplot(3,1,1)
plot(1:length(r),r,'r*')
subplot(3,1,2)
plot(1:length(p),p,'g*')
subplot(3,1,3)
plot(1:length(y),y,'b*')

hold off
    

    