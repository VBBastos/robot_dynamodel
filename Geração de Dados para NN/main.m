clc, clear, close all
% 
% [x,y,psi,i] = genrobotdata(0,0,pi/2,1,3,0.0001);
% [optx,opty] = optivectorv(x,y);
% 
% plot(x,y)
% figure
% plot(1:i,psi*180/pi)
% figure

%PLOT PROP
LSIZE = 18; %LEGEND FONT SIZE

target = randn(1,50)*20;
% target = [-14.6834   -0.6163    4.6469    8.5278   -7.4562   -4.7291   40.4738  -45.1671   44.5889    6.7513 20.0012  -33.2833  -11.8007   -5.5613    8.4543  -33.4040    9.4327  -24.2569    1.3238   13.0471 ...
%    6.5412   21.6527   20.1215  -13.0182    5.1411  -18.8876  -26.4358   18.4965    0.0010   -1.0984 18.2225   11.8917    7.0040   25.0050   18.5958    4.7953  -13.8072  -13.0311   23.8420  -32.2366 ...
%    -0.4892  -38.9769   20.4100   17.2343    0.0232   -1.4167  -49.7257   11.6234  -43.8487  -46.3856];

 %Big Complicated

target = [5,0,5,5,-5,5,-5,-5,5,-5,5,-2] %Trajetória em Quadrado
% target = [5,0,5,5] %Trajetória em L
% target = [0,1,1,1,-1,-3,-1,-2,-1,-1,0,0] %Short

[xres,yres,psires,ures,wres,urefres,wrefres,xsres,ysres,usres,wsres] = trajrobot(target,0.001);

urefres = [0 urefres];


wrefres = [0 wrefres];
l=length(xres);
save('data','xres','yres','psires','ures','wres','urefres','wrefres');

%COMPARISON

%plot(xres,yres,xsres,ysres,'r')
plot(xres,yres,'r','LineWidth',2)

hold on
plot(xsres,ysres,'g--','LineWidth',2);
plot(0,0,'--gs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5])

    text(.03,.03 , num2str(0),'Color','k','FontSize',14)

for i=1:2:(length(target)-1)
    plot(target(i),target(i+1),'--gs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5])

    text(target(i)+0.02, target(i+1), num2str((i+1)/2),'Color','k','FontSize',11)
    
    
end

axis([min(xsres)*1.1 max(xsres)*1.1 min(ysres)*1.1 max(ysres)*1.1]);

%title("Trajetória Percorrida")
xlabel('X (m)')
ylabel('Y (m)')
lgd1 = legend('Analytical model', 'SINDy model')
lgd1.FontSize =  LSIZE;


%JUST TRAJ
figure 
%plot(xres,yres,xsres,ysres,'r')
plot(xres,yres,'r','LineWidth',2)

hold on
%plot(xsres,ysres,'g--','LineWidth',2);
plot(0,0,'--gs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5])

    text(.03,.03 , num2str(0),'Color','k','FontSize',14)

for i=1:2:(length(target)-1)
    plot(target(i),target(i+1),'--gs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5])

    text(target(i)+0.02, target(i+1), num2str((i+1)/2),'Color','k','FontSize',11)
    
    
end

axis([min(xres)*1.1 max(xres)*1.1 min(yres)*1.1 max(yres)*1.1]);


%title("Trajetória Percorrida")
xlabel('X (m)')
ylabel('Y (m)')
lgd2 = legend('Trajectory')
lgd2.FontSize =  LSIZE;


%Change DAta
ures(ures<0)=0;
usres(usres<0)=0;



hold off
figure
subplot(221),   plot((1:l)*.01,ures,(1:l)*.01, usres, 'r--')
title("a) Linear Velocity (analytical vs SINDy)")
lgd3 = legend("Analytical", "SINDy", "Location", "Best")
lgd3.FontSize =  LSIZE;
subplot(222), plot ((1:l)*.01,wres,(1:l)*.01, wsres, 'r--')
title("b) Angular Velocity (analytical vs SINDy)")
legend("Analytical", "SINDy", "Location", "Best")
subplot(223), plot((1:l)*.01,urefres)
title("c) Input u_{ref}")
subplot(224), plot((1:l)*.01,wrefres)
title("d) Input \omega_{ref}")
figure
erro = sqrt((xres-xsres).^2 + (yres-ysres).^2);
plot((1:l)*0.01,erro,'LineWidth',2);
%title("Erro de trajetória")
lgd4 = legend("Trajectory Error")
lgd4.FontSize =  LSIZE;
ylabel('meters')
xlabel('seconds')


%[optx,opty] = optivectorv(xres,yres);



figure
errov = ures-usres;
subplot(211), plot((1:l)*0.01,errov,'k');
title("a) Linear Error")
lgd5 = legend("Linear Vel. Error")
lgd5.FontSize =  LSIZE;
ylabel('m/s')
xlabel('seconds')


errow = abs(wres)-abs(wsres);
subplot(212), plot((1:l)*0.01,errow,'k');
lgd6= legend("Angular Vel. Error")
lgd6.FontSize =  LSIZE;
title("b) Angular Error")
ylabel('rad/s')
xlabel('seconds')




figure
plot(1:l, ures,1:l,urefres,':')
figure
subplot(222), plot((1:l)*.01,ures,'LineWidth',2)
xlabel('seconds')
ylabel('m/sec')
title('Control Output (u_r_e_f)')
subplot(221), plot((1:l)*.01,urefres,'g','LineWidth',2)
xlabel('seconds')
ylabel('m/sec')
title('Dynamic Output (u)')
subplot(224), plot((1:l)*.01,wres,'LineWidth',2)
title('Control Output (\omega_r_e_f)')
xlabel('seconds')
ylabel('rad/sec')
subplot(223), plot((1:l)*.01,wrefres,'g','LineWidth',2)
title('Dynamic Output (\omega)')
xlabel('seconds')
ylabel('rad/sec')



