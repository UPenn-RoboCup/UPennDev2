function plot_goalpost( postStats, v, rollAngle, scale, scaleB)

x0=postStats.x;
y0=postStats.y;
w0=postStats.a/2;
h0=postStats.b/2;
a0=postStats.o;
x0=x0/scale;y0=y0/scale;
w0=w0/scale;h0=h0/scale;
r=[cos(a0) sin(a0);-sin(a0) cos(a0)];
x11=[x0 y0]+(r*[w0 h0]')';
x12=[x0 y0]+(r*[-w0 h0]')';
x21=[x0 y0]+(r*[w0 -h0]')';
x22=[x0 y0]+(r*[-w0 -h0]')';

gbx1=(postStats.gbx1)/scale*scaleB;
gbx2=(postStats.gbx2+1)/scale*scaleB;
gby1=(postStats.gby1)/scale*scaleB;
gby2=(postStats.gby2+1)/scale*scaleB;

xskew = tan(rollAngle);
gbx11=gbx1+gby1*xskew;
gbx12=gbx1+gby2*xskew;
gbx21=gbx2+gby1*xskew;
gbx22=gbx2+gby2*xskew;

if overlay_level
    strgoalpos = sprintf('%.2f %.2f',v.x,v.y);
    b_name=text(x0,y0, strgoalpos,'BackGroundColor',[.7 .7 .7]);
    set(b_name,'FontSize',8);
end

goalcolor='r';
goalwidth=2;
bbcolor='w--';
bbwidth=1;

%{
plot([x11(1) x12(1)],[x11(2) x12(2)],goalcolor,'LineWidth',goalwidth);
plot([x21(1) x22(1)],[x21(2) x22(2)],goalcolor,'LineWidth',goalwidth);
plot([x12(1) x22(1)],[x12(2) x22(2)],goalcolor,'LineWidth',goalwidth);
plot([x11(1) x21(1)],[x11(2) x21(2)],goalcolor,'LineWidth',goalwidth);

plot([gbx11 gbx21],[gby1 gby1],bbcolor,'LineWidth',bbwidth);
plot([gbx12 gbx22],[gby2 gby2],bbcolor,'LineWidth',bbwidth);
plot([gbx11 gbx12],[gby1 gby2],bbcolor,'LineWidth',bbwidth);
plot([gbx21 gbx22],[gby1 gby2],bbcolor,'LineWidth',bbwidth);
%}

end