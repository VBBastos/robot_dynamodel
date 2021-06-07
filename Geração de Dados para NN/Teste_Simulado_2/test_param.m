clc, clear, close all

%% Initial Conditions 
    %x0=0;
    %y0=0;
    %psi0=2*pi;
    x(1)=0;
    y(1)=0;
    psi(1)=0;
    u(1)=0.4;
    w(1)=0;
    du(1)=0;
    dw(1)=0;
    i=1;
    a=0;
    
    xs(1)=x(1);
    ys(1)=y(1);
    psis(1)=0;
    ws(1)=0;
    us(1)=0.4;
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
%Tmax = max([time_w_ref' time_w_ref' time_w_ref' time_w_ref'])-time_offset_end;
Tmax = 75
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
%        [ 0.66271609, -0.84434026,  0.        ,  0.        ]];]

Param =[[-3.04412824e-03, -2.44144597e-03, -4.38608548e-03,         1.82729505e-03],
       [ 4.30377235e-01,  9.46168475e-01, -8.28594051e-01,         2.01645428e-01],
       [ 8.59205578e-02,  1.03062088e-01, -2.50322256e-01,        -3.13937121e-02],
       [-1.75215761e+00, -2.97624886e+00,  2.30089977e+00,        -4.18131725e-01],
       [-2.19447201e-01, -1.65318061e-01,  4.79157186e-01,         1.11445119e-01],
       [ 4.95935637e-01,  4.55359135e+00, -6.16874673e-01,         6.03662300e+00],
       [ 3.58066269e+01,  2.92752771e+01, -8.48988832e+00,         8.78581101e-01],
       [-1.38692347e+01, -4.48486560e+00, -3.00617742e+00,        -4.13996990e+00],
       [ 6.65874977e+00, -1.86558495e+01,  1.25799201e+01,        -2.30410049e+01],
       [-1.01305205e+02, -7.48468490e+01,  2.79377396e+01,        -5.44115757e-01],
       [-1.22582789e+01,  3.04804740e+01, -2.92973503e+01,         1.70974444e+01],
       [-5.85767560e+01, -5.96091333e+01,  1.34500680e+01,        -2.10944719e+00],
       [ 5.54597869e+01,  8.93619574e+00,  1.06594950e+01,         1.42118096e+01],
       [ 1.70725312e+02,  1.44519266e+02, -4.64138543e+01,        -8.78700915e-01],
       [-4.72964850e+01,  1.32706017e+00, -7.88804241e+00,        -1.16544045e+01]];
   
   
   
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


