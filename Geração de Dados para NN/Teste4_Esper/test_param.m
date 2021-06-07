clc, clear, close all

%% Initial Conditions 
    %x0=0;
    %y0=0;
    %psi0=2*pi;
    x(1)=0;
    y(1)=0;
    psi(1)=0;
    u(1)=0;
    w(1)=0;
    du(1)=0;
    dw(1)=0;
    ts=0.1;
    i=1;
    a=0;
    
    xs(1)=x(1);
    ys(1)=y(1);
    psis(1)=0;
    us(1)=0;
    ws(1)=0;
    us(1)=0;
    dus(1)=0;
    dws(1)=0;
    s(1) = 0;
    
%% Load Experiment Data
    
Pose = loadLidarPose();

[Linear_Vel, Ang_Vel] = loadControlData();

Encoder_Vel = loadEncoderVel();


min_time = min([Linear_Vel(:,1)']);


%% Declare constants and atribute variables


R=0.39; %Distance between the rear wheels of the robot
j=1;
ts=0.1;
time_offset_begin = 6;
time_offset_end = 9;
min_time = min([Encoder_Vel(:,1)' Encoder_Vel(:,3)' Encoder_Vel(:,5)' Encoder_Vel(:,7)'])+time_offset_begin;
time_pid_left = Encoder_Vel(:,1);
data_pid_left = Encoder_Vel(:,2);
time_pid_right = Encoder_Vel(:,3);
data_pid_right = Encoder_Vel(:,4);
time_encoder_left = Encoder_Vel(:,5);
data_encoder_left = Encoder_Vel(:,6);
time_encoder_right = Encoder_Vel(:,7);
data_encoder_right = Encoder_Vel(:,8);

%% Correct Time

time_pid_left = Encoder_Vel(:,1)-ones(length(time_pid_left),1)*min_time;
time_pid_right = Encoder_Vel(:,3)-ones(length(time_pid_right),1)*min_time;
time_encoder_left = Encoder_Vel(:,5)-ones(length(time_encoder_left),1)*min_time;
time_encoder_right = Encoder_Vel(:,7)-ones(length(time_encoder_right),1)*min_time;
Tmax = floor(max([time_pid_left' time_pid_right' time_encoder_left' time_encoder_right']))-time_offset_end;
xtt=0:ts:Tmax;

%% Calculate Linear and Angular Values
for i = 1:1:length(Encoder_Vel)
    VLin(j,1)=(data_encoder_left(i)+data_encoder_right(i))/2;
    VLin_ref(j,1)=(data_pid_left(i)+data_pid_right(i))/2;
    VAng(j,1)= -(data_encoder_left(i)-data_encoder_right(i))./R;
    VAng_ref(j,1)= -(data_pid_left(i)-data_pid_right(i))./R;
    j=j+1;
end

%% Load Parameters
    
%[1 us(i) ws(i) urefs(i) wrefs(i) us(i)^2 us(i)*ws(i) ws(i)^2 us(i)*urefs(i)...
%ws(i)*urefs(i) urefs(i)^2 us(i)*wrefs(i) ws(i)*wrefs(i) urefs(i)*wrefs(i) wrefs(i)^2]';


% Param = [[ 0.        ,  0.        ,  0.        ,  0.        ],
%        [ -0.49914546        ,  1.91184578*0,  0.        , -0.98018837],
%        [ 0.        , -1.31425242,  0.        ,  0.        ],
%        [ 0.54684089        ,  0.        ,  0.        ,  0.        ],
%        [ 0.        ,  1.34237693,  0.        ,  0.        ],
%        [ 0.        , -3.85266109*0,  0.        ,  3.50638403],
%        [ 0.        ,  1.8206167*0 ,  0.        ,  0.        ],
%        [ 0.        ,  0.58984202*0,  0.        , -0.9310782 ],
%        [ -0.85279204*0        ,  0.        ,  0.        , -1.54802118],
%        [ 0.        ,  1.56780082*0, -1.30042476, -3.55083494],
%        [ 0.37342044*0       ,  0.        ,  0.        ,  0.        ],
%        [ -0.41710734*0       , -1.21705287*0,  0.        ,  0.        ],
%        [ 0.        , -1.36317502*0,  0.        ,  2.15373567],
%        [ 0.        ,  0.        ,  0.        ,  0.        ],
%        [ 0.        ,  0.75211403*0,  0.        , -1.15583033]];

% Param = [[ 0.        ,  0.        ,  0.14394684,  0.1790591 ],
%        [-1.33565976, -5.04216188,  1.61907177, -3.82269538],
%        [-0.16062056, -0.93340029,  0.47770145, -1.32444837],
%        [ 0.        ,  0.99024936,  3.37859394,  0.        ],
%        [ 0.14557677,  0.78900655, -0.49026001,  1.16719479],
%        [ 2.20919813, 11.19895501, -7.4016021 ,  6.05632175],
%        [-0.53043988,  1.91817643, -4.87491266,  3.88124923],
%        [-0.36205375, -0.7771016 , -0.99924554,  1.10735941],
%        [ 0.        , -1.73412767, -1.55441783,  1.6350709 ],
%        [ 0.73103612,  0.14046252,  0.56021987, -2.48944262],
%        [ 0.6872131 , -0.26489461, -3.54889131,  0.        ],
%        [ 0.54492893, -1.68480254,  1.0129955 ,  0.        ],
%        [ 0.64749334,  0.38897518,  0.51517619, -1.35663312],
%        [-0.50340997,  0.1742406 , -0.24243301, -1.84276311],
%        [-0.31856194,  0.26588312,  0.17532984, -0.2024135 ]];

%%Linear
Param = [[ 0.       ,  0.            ,  0.        ,  0.        ],
       [ -0.34196831,  -0.71630687   ,  1.64181477, -2.44681112],
       [ 0.        ,  -0.69479772    ,  0.        , -1.09224163],
       [ 0.32776072,  0.28518169     ,  3.15043201,  0.        ],
       [ 0         ,  0.6349971      ,  0.        ,  0.94219403],
       [ 0         ,  0.              , -6.85107369,  4.26843251],
       [ 0.        ,  0.             , -1.94112164,  3.67867477],
       [ 0.        ,  0.             ,  0.        ,  1.57011151],
       [ 0.        ,  0.             , -1.75581191,  1.23235082],
       [ 0.        ,  0.             ,  0.        , -2.80244856],
       [ 0.        ,  0.             , -2.87250572,  0.        ],
       [ 0.        ,  0.             ,  0.        ,  0.        ],
       [ 0.        ,  0.             ,  0.        , -1.5552553 ],
       [ 0.        ,  0.             ,  0.        , -1.5206809 ],
       [ 0.        ,  0.             ,  0.        ,  0.        ]];
   
   
%    Param = [[ 0.                 ,  0.            ,  0.        ,  0.        ], %0
%            [ -0.18831067         ,   -0.62450366   ,  1.64181477, -2.44681112], %u
%            [ 0.14559525          ,  -0.72895277    ,  0.        , -1.09224163], %w
%            [  0.21961115         ,  0.32059062     ,  3.15043201,  0.        ], %uref
%            [ -0.1271808          ,  0.66210786      ,  0.        ,  0.94219403], %wref
%            [ 0.                   ,  0.              , -6.85107369,  4.26843251], %u2
%            [ -0.20783675         ,  0.52815684             ,  0.        ,  1.57011151], %uw
%            [ 0.                  ,  0.14042704             , -1.75581191,  1.23235082], %w2
%            [ 0.                  ,  0.             ,  0.        , -2.80244856],
%            [ 0.                  ,  0.             , -2.87250572,  0.        ],
%            [ 0.                  ,  0.             ,  0.        ,  0.        ],
%            [ 0.                  ,  0.             ,  0.        , -1.5552553 ],
%            [ 0.                  ,  0.             ,  0.        , -1.5206809 ],
%            [ 0.                  ,  0.             ,  0.        ,  0.        ]
%            [ 0.                  ,  0.             ,  0.        ,  0.        ]];
%    
   
   
%% Load data


data = load('data.mat');
dataLin = data.ures;
dataAng = data.wres;
dataCtrlLin = data.urefres;
dataCtrlAng = data.wrefres;


%% Interpolate

ind = 1;
id_n = 0;

for t=0:ts:Tmax
    
    [Ye,Ie] = min(abs(time_pid_left-ones(length(time_pid_left),1)*t));
    if Ie==1
        VAngF(ind,1) = VAng(Ie);
        VLinF(ind,1) = VLin(Ie);
        VLinF_ref(ind,1) = VLin_ref(Ie);
        VAngF_ref(ind,1) = VAng_ref(Ie);
    elseif (Ye<0.05) 
        VAngF(ind,1) = VAng(Ie);
        VLinF(ind,1) = VLin(Ie);
        VLinF_ref(ind,1) = VLin_ref(Ie);
        VAngF_ref(ind,1) = VAng_ref(Ie);
    elseif(Encoder_Vel(Ie,1)-t>0)
        VAngF(ind,1) = VAng(Ie-1);
        VLinF(ind,1) = VLin(Ie-1);
        VLinF_ref(ind,1) = VLin_ref(Ie-1);
        VAngF_ref(ind,1) = VAng_ref(Ie-1);
    else
        VAngF(ind,1) = VAng(Ie);
        VLinF(ind,1) = VLin(Ie);
        VLinF_ref(ind,1) = VLin_ref(Ie);
        VAngF_ref(ind,1) = VAng_ref(Ie);
    end  
    ind = ind + 1;

end
AngRefF = VAngF_ref;
LinRefF = VLinF_ref;


%% Trajectory Estimation Using Sindy Parameters

i =1;
for t=0:ts:Tmax
    
    urefs(i) = LinRefF(i);
    wrefs(i) = AngRefF(i);
    
    
    dus(i+1) = Param(:,1)'*[1 us(i) ws(i) urefs(i) wrefs(i) us(i)^2 us(i)*ws(i) ws(i)^2 us(i)*urefs(i)...
          ws(i)*urefs(i) urefs(i)^2 us(i)*wrefs(i) ws(i)*wrefs(i) urefs(i)*wrefs(i) wrefs(i)^2]';
     
    
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
    dxs(i+1)=us(i+1)*cos(psis(i))-0*ws(i)*sin(psis(i));  %Velocidade x
    dys(i+1)=us(i+1)*sin(psis(i))+0*ws(i)*cos(psis(i)); %Velocidade y
    
     psis(i+1)=ws(i+1)*ts+psis(i); %Estado de rotação
     xs(i+1) = xs(i)+s(i+1)*cos(psis(i+1));
     ys(i+1) = ys(i)+s(i+1)*sin(psis(i+1));
     
     i=i+1;
         
end


%% Plots

xtt = 0:ts:Tmax;

figure, plot(xtt, us(1:(length(us)-1)))
hold on, plot(xtt, urefs, '--g')
figure, plot(xtt, ws(1:(length(ws)-1)))
hold on, plot(xtt, wrefs,'--g')


figure, subplot(211), plot(xtt, us(1:(length(us)-1)))
hold on, plot(xtt, urefs,'--r')
plot(xtt, dataLin,'--k')
legend('u','u_r_e_f','u_{enc}')
xlabel('seconds')
ylabel('m/sec')
subplot(212), plot(xtt, ws(1:(length(ws)-1)))
hold on, plot(xtt, wrefs,'--r')
plot(xtt, dataAng,'--k')
legend('\omega','\omega_r_e_f','\omega_e_n_c')

xlabel('seconds')
ylabel('rad/sec')


% maxAng = max(VAngF)/max(Ang_Vel(:,2))/1.5;
% VLinF = VLinF*1.2;
% VAngF = VAngF./maxAng;
% for i = 1:length(xtt)
%    if (VLinF(i)< .2)
%        VLinF(i)=0;
%    end
% %    if (VLinF(i)> .1019)
% %        VLinF(i)=.1019;
% %    end
%    if (abs(VAngF(i))< .2)
%        VAngF(i)=0;
% %    elseif (abs(VAngF(i))< .35)
% %        VAngF(i)=sign(VAngF(i))*.25;
% %    else
% %        VAngF(i)=sign(VAngF(i))*.35;
%    end
%    
%     
% end

% [B,A]=butter(8,.01,'low');
% VLinF = filtfilt(B,A,VLinF);
% VAngF = filtfilt(B,A,VAngF);


psit(1)=0;
dxt(1)=0;
dyt(1)=0;
ind = 1;
xt(1)=0;
yt(1)=0;
for i=0:ts:(Tmax-ts)
    ind=ind+1;
    psit(ind) = VAngF(ind)*ts + psit(ind-1);
    dxt(ind) = VLinF(ind)*cos(psit(ind));
    dyt(ind) = VLinF(ind)*sin(psit(ind));
    xt(ind) = dxt(ind)*ts + xt(ind-1);
    yt(ind) = dyt(ind)*ts + yt(ind-1);
end


psir(1)=0;
dxr(1)=0;
dyr(1)=0;
indr = 1;
xr(1)=0;
yr(1)=0;
for i=0:ts:(Tmax-ts)
    indr=indr+1;
    psir(indr) = VAngF_ref(indr)*ts + psir(indr-1);
    dxr(indr) = VLinF_ref(indr)*cos(psir(indr));
    dyr(indr) = VLinF_ref(indr)*sin(psir(indr));
    xr(indr) = dxr(indr)*ts + xr(indr-1);
    yr(indr) = dyr(indr)*ts + yr(indr-1);
end

figure, plot(xs,ys)
hold on, plot(xt,yt)
plot(xr,yr,'k')
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

    %text(.03,.03 , 'begin','Color','k','FontSize',14)

dataxtt = 0:ts:Tmax    ;
size_data = length(dataxtt);
    
figure
subplot(211), plot(xtt, dataLin, xtt, dataCtrlLin, 'k--')
subplot(212), plot(xtt, dataAng, xtt, dataCtrlAng, 'k--')

