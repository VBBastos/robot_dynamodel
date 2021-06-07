clc, clear, close all
%COM E SEM RESAMPLE
%PLOT PROP
LSIZE = 18; %LEGEND FONT SIZE

%Get and plot Trajectory
[x,y] = poses();
len = length(x);
figure (1),
plot(x,y,'LineWidth',2);
hold on
plot(x(1),y(1),'gs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
plot(x(len),y(len),'rs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5])

lgd1 = legend("Trajectory", "Start", "End", "Location", "Best")
lgd1.FontSize = LSIZE;
xlabel('X (meters)');
ylabel('Y (meters)');



%Get Linear and Angular Speeds
[uxl,wzl,time] = speeds();
size = length(time);
rate = 10;

%Correct time
time = time - ones(size,1)*time(1);

ux = resample(uxl,time,rate);
wz = resample(wzl,time,rate);




time = 0:1/rate:(floor(length(ux)/rate)-1);
size=length(time);

ux=ux(1:length(time))';
wz=wz(1:length(time))';

%Plot Speeds
figure (2),
subplot(3,2,1), plot(time,ux);
title("ux");

subplot(3,2,6), plot(time,wz);
title("wz");


%Modelo P3DX
theta=[0.2604 0.2509 -0.000499 0.9965 0.00263 1.0768];
ctheta = [0.2664    0.2689         0   1.0148         0   1.0475];



%Calculate Acceleration
u=ux;
w=wz;


%Testes
%u(0.47<u) = 0.5;
%%%%

du(1) = 0;
dw(1) = 0;
dt = time(2:size)-time(1:size-1);
du(2:size) = (u(2:size)-u(1:size-1))./dt;
dw(2:size) = (w(2:size)-w(1:size-1))./dt;


%uref(1:size-1) = theta(1)*du(2:size) + theta(4)*u(1:size) - theta(3)*(w(1:size).^2);
ws(1)=0;
us(1)=0;
cws(1)=0;
cus(1)=0;
psis(1)=0;
xs(1)=0;
ys(1)=0;
cpsis(1)=0;
cxs(1)=0;
cys(1)=0;


%%%%CTHETA%%%%

ctheta = [0.2749    0.3708         0   0.9740   0.2647   1.0130]
ctheta = [0.2664    0.2689         0   1.0148         0   1.0475];


for i=1:size-1
    
    
   %du(i+1) = u(i+1)-u(i))
   uref(i,1) = theta(1)*du(i+1) + theta(4)*u(i) - theta(3)*(w(i)^2);
   wref(i,1) = theta(2)*dw(i+1) + theta(6)*w(i) + theta(5)*u(i)*w(i);
   
   dus(i+1)=(1/theta(1))*uref(i)+(theta(3)/theta(1))*(ws(i)^2)-...
         (theta(4)/theta(1))*us(i); %Aceleraçao Linear
   dws(i+1)=(1/theta(2))*wref(i)-(theta(6)/theta(2))*ws(i)-...
         (theta(5)/theta(2))*us(i)*ws(i); %Aceleração Angular
     
   cdus(i+1) = (1/ctheta(1))*uref(i)+(ctheta(3)/ctheta(1))*(cws(i)^2)-...
         (ctheta(4)/ctheta(1))*cus(i); %Aceleraçao Linear
     
   cdws(i+1)=(1/ctheta(2))*wref(i)-(ctheta(6)/theta(2))*cws(i)-...
         (ctheta(5)/ctheta(2))*cus(i)*cws(i); %Aceleração Angular
     
   us(i+1)=dus(i+1)*dt(i) + us(i);
   ws(i+1)=dws(i+1)*dt(i) + ws(i);
   
   cus(i+1)=cdus(i+1)*dt(i) + cus(i);
   cws(i+1)=cdws(i+1)*dt(i) + cws(i);
   
   psis(i+1) = ws(i+1)*dt(i) + psis(i);
   xs(i+1) = us(i+1)*dt(i)*sin(psis(i+1))+xs(i);
   ys(i+1) = us(i+1)*dt(i)*cos(psis(i+1))+ys(i);
   
   cpsis(i+1) = cws(i+1)*dt(i) + cpsis(i);
   cxs(i+1) = cus(i+1)*dt(i)*sin(cpsis(i+1))+cxs(i);
   cys(i+1) = cus(i+1)*dt(i)*cos(cpsis(i+1))+cys(i);
   
    
    
end

uref(size) = 0;
wref(size) = 0;
mi = 1;


ures=u(mi:size);
wres=w(mi:size);
wrefres=wref(mi:size)';
urefres=uref(mi:size)';


%Plot Speeds and Acceleration

figure (3),
subplot(2,2,1), plot(time(mi:size),u(mi:size));
hold on
plot(time(mi:size),us(mi:size),'g--');
title("u");
hold off
subplot(2,2,2), plot(time,du);
hold on
plot(time(mi:size),dus(mi:size),'g--');
hold off
title("du");
subplot(2,2,3), plot(time(mi:size),w(mi:size));
hold on
plot(time(mi:size),ws(mi:size),'g--');
title("w");
subplot(2,2,4), plot(time,dw);
hold on
plot(time(mi:size),dws(mi:size),'g--');
hold off
title("dw");
hold off

figure (4),
subplot(2,1,1), plot(time, uref);
title("a) Linear Speed");
hold on
plot(time,us,'g--');
hold off
lgd4 = legend("Reference Speed", "Output Speed", "Location", "Best")
lgd4.FontSize = LSIZE;

subplot(2,1,2), plot(time, wref);
title("b) Angular Speed");
hold on
plot(time,ws,'g--');
hold off
legend("Reference Speed ", "Output Speed", "Location", "Best")
save('data','x','y','ures','wres','urefres','wrefres');

figure(5),

angle = 119;
angle = angle*pi/180;


module = (x.^2 + y.^2).^0.5;
anglexy = atan(y./x);
anglexy = anglexy + ones(length(anglexy),1)*angle;
x = module.*cos(anglexy);
y = module.*sin(anglexy);

plot(x,y)


hold on

plot(-xs,ys,'g--')
plot(-cxs,cys,'r')
hold off
lgd5 = legend("Dataset","Estimated from Vel","Estimated from SINDy","Location","Best");
lgd5.FontSize = LSIZE;
xlabel('meters');
ylabel('meters');

figure (6),
subplot(2,1,1), plot(time, cus);
title("a) Linear Velocity");
hold on
plot(time,us,'g--');
hold off
lgd6 = legend("Estimated from SINDy", "Dataset", "Location", "Best")
lgd6.FontSize = LSIZE;
xlabel('seconds');
ylabel('m/s');

subplot(2,1,2), plot(time, wref);
title("b) Angular Velocity");
hold on
plot(time,ws,'g--');
hold off
lgd6 = legend("Estimated from SINDy", "Dataset", "Location", "Best")
lgd6.FontSize = LSIZE;
xlabel('seconds');
ylabel('rad/s');
save('data','x','y','ures','wres','urefres','wrefres');

%Erros

lenc = length(cus);
MAX_ABS_ERROR_U = max(abs(cus-us))
MAX_ABS_ERROR_w = max(abs(cws-ws))
ERROR_U = cus-us;
ERROR_W = cws-ws;


figure(7),
subplot(2,1,1), plot(time,ERROR_U);
title("a) Linear Speed Error");
xlabel('seconds');
ylabel('meters');
subplot(2,1,2), plot(time,ERROR_W);
title("b) Angular Speed Error");
xlabel('seconds');
ylabel('radians');









