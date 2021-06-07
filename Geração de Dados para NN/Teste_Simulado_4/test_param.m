clc, clear, close all

%% Initial Conditions 
    %x0=0;
    %y0=0;
    %psi0=2*pi;
    x(1)=0;
    y(1)=0;
    psi(1)=0;
    u(1)=0.0;
    w(1)=0;
    du(1)=0;
    dw(1)=0;
    i=1;
    a=0;
    
    xs(1)=x(1);
    ys(1)=y(1);
    psis(1)=0;
    ws(1)=0;
    us(1)=0.0;
    dus(1)=0;
    dws(1)=0;
    s(1) = 0;
    
%% Load Experiment Data
    
DATA = loadSimData();

time_w_ref = DATA(:,1);
w_ref = DATA(:,2);
time_u_ref = DATA(:,3);
u_ref = DATA(:,4);
time_x_sim = DATA(:,5);
x_sim = DATA(:,6);
time_y_sim = DATA(:,7);
y_sim = DATA(:,8);
time_psi_sim = DATA(:,9);
psi_sim = DATA(:,10);


%% Variables
R=0.49; %Distance between the rear wheels of the robot
j=1;
ts=0.1;
time_offset_begin = 0;
time_offset_end = 0;
min_time = min([time_psi_sim' time_y_sim' time_x_sim' time_u_ref' time_w_ref'])+time_offset_begin;
max_length = length(DATA(:,1));

%% Ajust Times

time_w_ref = time_w_ref-ones(length(time_w_ref),1)*min_time;
time_u_ref = time_u_ref-ones(length(time_u_ref),1)*min_time;
time_x_sim = time_x_sim-ones(length(time_x_sim),1)*min_time;
time_y_sim = time_y_sim-ones(length(time_y_sim),1)*min_time;
time_psi_sim = time_psi_sim-ones(length(time_psi_sim),1)*min_time;
Tmax = max([time_w_ref' time_w_ref' time_w_ref' time_w_ref'])-time_offset_end;
Tmax = 185
%Tmax = 85;
xtt=0:ts:Tmax;



%% Load Parameters
    
%[1 us(i) ws(i) urefs(i) wrefs(i) us(i)^2 us(i)*ws(i) ws(i)^2 us(i)*urefs(i)...
%ws(i)*urefs(i) urefs(i)^2 us(i)*wrefs(i) ws(i)*wrefs(i) urefs(i)*wrefs(i) wrefs(i)^2]';
% 
% Param = [[  0.        ,   0.        ,   0.        ,   0.        ],
%        [ -1.03013291,   2.08660338,   0.        ,   0.57777575],
%        [  0.        ,  -4.36478753,   0.        ,   0.        ],
%        [  3.19360829,  -4.73167799,   0.        ,  -1.53290063],
%        [  0.        ,  -1.18560475,   0.        ,   0.        ],
%        [  0.61632756,  12.36003576,   0.        ,   0.        ],
%        [  0.        ,  -4.41481929,   0.        ,   0.        ],
%        [  0.        ,   0.        ,   0.        ,   0.        ],
%        [ -1.18274649, -88.70818998,   0.        ,  -0.61694828],
%        [  0.        ,  20.39828403,   0.        ,   0.59136368],
%        [ -2.72095254, 149.73456464,   0.        ,   0.9724361 ],
%        [  0.        ,  14.89034654,   0.        ,   1.18302921],
%        [  0.        ,   4.84475667,   0.        ,   0.        ],
%        [  0.        , -95.43530722,   0.        ,  -5.33442894],
%        [  0.56865242,  -7.16868339,   0.        ,   0.        ]];

% 
% Param = [[ 0.        ,  0.        ,  0.        ,  0.        ],
%        [-0.96473193,  0.76703328, -0.43163795,  0.32056479],
%        [ 0.        , -1.06986276,  0.        , -0.53768904],
%        [ 2.98708278, -1.4397644 ,  0.99154609, -0.79199553],
%        [ 0.        ,  4.09743447,  0.        ,  0.88174191],
%        [ 0.        ,  0.        ,  0.        ,  0.91120397],
%        [ 0.        ,  2.34202608,  0.        ,  0.        ],
%        [ 0.        ,  0.        ,  0.        ,  0.        ],
%        [ 1.74885058, -4.67953949,  0.8162456 , -5.83334626],
%        [ 0.        , -5.11219121,  0.        , -1.08380153],
%        [-5.65031015, 10.79669367, -1.47760158,  8.25386515],
%        [-0.37867062,  0.71553625,  0.        ,  1.1516966 ],
%        [ 0.        ,  0.        ,  0.        ,  0.        ],
%        [ 1.00457291,  4.82265272,  0.        , -5.81974781],
%        [ 0.66271609, -0.84434026,  0.        ,  0.        ]];

Param =[[   0.        ,    0.        ,    0.        ,    0.        ],
       [   0.        ,  -28.65671656,    0.        ,    0.        ],
       [   0.        ,   -5.20742845,    0.        ,    0.        ],
       [   0.        ,  -44.0540581 ,    0.        ,    0.        ],
       [   0.        ,    8.38210309,    0.        ,    0.        ],
       [   0.        ,  102.88682204,    0.        ,  -43.8462081 ],
       [   0.        ,    0.        ,    0.        ,    0.        ],
       [   0.        ,    0.        ,    0.        ,    0.        ],
       [  -2.91439332, -139.14286607,    0.        ,  252.09367541],
       [   0.        ,   92.42617184,    0.        ,  -40.94622608],
       [   6.51343759, 1305.4412581 ,    0.        , -334.43300405],
       [   0.        ,  -17.89276963,    0.        ,    8.89925578],
       [   0.        ,    0.        ,    0.        ,    0.        ],
       [   0.        ,  147.29291961,    0.        ,  -49.48660597],
       [   0.        ,    0.        ,    0.        ,    0.        ]];
   
   
   
%% Load data


data = load('data.mat');
dataLin = data.ures;
dataAng = data.wres;
dataCtrlLin = data.urefres;
dataCtrlAng = data.wrefres;


%% Trajectory Estimation Using Sindy Parameters

i =1;
for t=0:ts:Tmax
    
    urefs(i) = dataCtrlLin(i);
    wrefs(i) = dataCtrlAng(i);
    
    
    dus(i+1) = Param(:,1)'*[1 us(i) ws(i) urefs(i) wrefs(i) us(i)^2 us(i)*ws(i) ws(i)^2 us(i)*urefs(i)...
          ws(i)*urefs(i) urefs(i)^2 us(i)*wrefs(i) ws(i)*wrefs(i) urefs(i)*wrefs(i) wrefs(i)^2]';
     
%     if us(i+1)>0.1
%         us(i+1)=0.1;
%     end
%     if us(i+1)<0
%         us(i+1)=0;
%     end
    us(i+1)=dus(i+1)*ts + us(i);
%     
%     if us(i+1)>0.1
%         us(i+1)=0.1;
%     end
%     if us(i+1)<0
%         us(i+1)=0;
%     end
        
    
    dws(i+1) = Param(:,2)'*[1 us(i) ws(i) urefs(i) wrefs(i) us(i)^2 us(i)*ws(i) ws(i)^2 us(i)*urefs(i)...
          ws(i)*urefs(i) urefs(i)^2 us(i)*wrefs(i) ws(i)*wrefs(i) urefs(i)*wrefs(i) wrefs(i)^2]';
     
    ws(i+1)=dws(i+1)*ts + ws(i);
    
%     if (us(i)< .05)
%        us(i)=0;
%    end
%    if (us(i)> .05)
%        us(i)=.1;
%    end
%    if (abs(ws(i))< .25)
%        ws(i)=0;
%    else
%        ws(i)=-1*sign(ws(i))*.35;
%    end
%     if abs(ws(i+1))>.35
%         ws(i+1)=sign(ws(i+1))*.35;
%     end
    s(i+1)=us(i+1)*ts;
    dxs(i+1)=us(i+1)*cos(psis(i))-0.0*ws(i)*sin(psis(i));  %Velocidade x
    dys(i+1)=us(i+1)*sin(psis(i))+0.0*ws(i)*cos(psis(i)); %Velocidade y
    
     psis(i+1)=ws(i+1)*ts+psis(i); %Estado de rotação
     xs(i+1) = xs(i)+s(i+1)*cos(psis(i+1));
     ys(i+1) = ys(i)+s(i+1)*sin(psis(i+1));
     
     i=i+1;
         
end


%% Plots

figure, plot(xtt, us(1:(length(us)-1)))
hold on, plot(xtt, urefs, '--g')
figure, plot(xtt, ws(1:(length(ws)-1)))
hold on, plot(xtt, wrefs,'--g')


figure, subplot(211), plot(xtt, us(1:(length(us)-1)))
hold on, plot(xtt, urefs,'--r')
plot(xtt, dataLin,'--k')
legend('u','u_r_e_f','u_{data}')
xlabel('seconds')
ylabel('m/sec')
subplot(212), plot(xtt, ws(1:(length(ws)-1)))
hold on, plot(xtt, wrefs,'--r')
plot(xtt, dataAng,'--k')
legend('\omega','\omega_r_e_f','\omega_{data}')

xlabel('seconds')
ylabel('rad/sec')


figure, plot(xs,ys)
hold on, 
plot(0,0,'gs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5])


plot(xs(length(xs)),ys(length(ys)),'bs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5])

xlabel('meters')
ylabel('meters')

legend('SINDy','Encoder Odom' ,'Ref Odom','Initial Point', 'End of SINDy', 'Location', 'NorthEastOutside')

dataxtt = 0:ts:Tmax    ;
size_data = length(dataxtt);
    
figure
subplot(211), plot(xtt, dataLin, xtt, dataCtrlLin, 'k--')
subplot(212), plot(xtt, dataAng, xtt, dataCtrlAng, 'k--')


