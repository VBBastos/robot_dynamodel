clc, clear, close all

%Get Poses
[x,y,r,t] = poses();
len = length(x);
t = t - ones(len,1)*t(1);
dt = mean(t(2:len) - t(1:len-1));


%Get Speed
u(1) = 0;
u(2:len,1) = (x(2:len) - x(1:len-1))./t(1:len-1);


%Get Robot Model


%Plot

plot(x(1),y(1),'gs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5])
hold on,

plot(x(2:len-1),y(2:len-1),'LineWidth',2)

plot(x(len),y(len),'rs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5])


%title("Trajetória Percorrida")
xlabel('X (m)')
ylabel('Y (m)')
legend('Start', 'Path', 'End')

figure,
plot(t,r,'LineWidth',2);

xlabel('time')
ylabel('rad')