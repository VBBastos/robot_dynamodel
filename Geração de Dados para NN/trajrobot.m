function [xres,yres,psires,ures,wres,urefres,wrefres,xsres,ysres,usres,wsres] = trajrobot(target,tolerance)

%target=[1,0,1,1,0,1,1,1];
%tolerance=0.1;
n=length(target);
traj(1)=0;
traj(2)=0;
traj(3:n+2)=target;
c=1;
ini=1;
lpsi=0;
lpsis=0;
x0=0;
y0=0;
xs0=0;
ys0=0;

while(c-1~=n)
[x,y,psi,i,u,w,uref,wref,xs,ys,psis,us,ws] = genrobotdata(x0,y0,xs0,ys0,lpsi,lpsis,traj(c+2),traj(c+3),tolerance);
lpsi=psi(i);
lpsis=psis(i);
xres(ini:ini+i-1)=x;
yres(ini:ini+i-1)=y;
psires(ini:ini+i-1)=psi;
ures(ini:ini+i-1)=u;
wres(ini:ini+i-1)=w;
urefres(ini:ini+i-2)=uref;
wrefres(ini:ini+i-2)=wref;
xsres(ini:ini+i-1)=xs;
ysres(ini:ini+i-1)=ys;
usres(ini:ini+i-1)=us;
wsres(ini:ini+i-1)=ws;

ini=ini+i;
c=c+2;
x0=x(i);
y0=y(i);
xs0=xs(i);
ys0=ys(i);
%plot(x,y);
%hold on;

end
hold off;
%figure;
%plot(1:ini-1,psires*180/pi);
end