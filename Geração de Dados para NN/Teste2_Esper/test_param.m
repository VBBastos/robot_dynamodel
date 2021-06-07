clc, clear, close all

%Condições Iniciais
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

%% Load Parameters
    
%[1 us(i) ws(i) urefs(i) wrefs(i) us(i)^2 us(i)*ws(i) ws(i)^2 us(i)*urefs(i)...
%ws(i)*urefs(i) urefs(i)^2 us(i)*wrefs(i) ws(i)*wrefs(i) urefs(i)*wrefs(i) wrefs(i)^2]';

%Nice

% Param = [[ 0.        ,  0.        ,  0.        ,  0.        ],
%        [-0.49914546,  1.71222505,  0.        , -1.26329446],
%        [ 0.        , -1.12970458,  0.        ,  0.        ],
%        [ 0.74684089, -0.64744663,  0.        ,  0.36955142],
%        [ 0.        ,  1.23608646,  0.        ,  0.        ],
%        [ 0.        , -3.41632084,  0.        ,  4.6684057 ],
%        [ 0.        ,  1.73821105,  0.        ,  0.        ],
%        [ 0.        ,  0.4661397 ,  0.        , -1.03899238],
%        [-0.85279204,  1.31964219,  0.        , -2.98975504],
%        [ 0.        ,  0.45364335, -1.23910296, -3.40287723],
%        [ 0.37342044,  0.        ,  0.        ,  0.        ],
%        [-0.41710734, -0.90550503,  0.47327053,  0.43094981],
%        [ 0.        , -1.18778193,  0.        ,  2.36861057],
%        [ 0.        ,  0.        ,  0.        ,  0.        ],
%        [ 0.        ,  0.68759528,  0.        , -1.23225562]];
   
% Param = [[-1.28288634e-03, -1.47856330e-01,  5.58117860e-03,        -1.03732691e-02],
%        [-4.58890476e-01,  1.71427020e+00, -5.50621097e-01,        -1.07266741e+00],
%        [-1.72479462e-01, -1.04751097e+00, -5.21753084e-03,         5.48884172e-02],
%        [ 6.00683734e-01, -2.88143391e-01,  1.42286723e-01,         3.33116473e-01],
%        [ 2.22272260e-01,  1.14364566e+00, -6.54409467e-02,        -2.26739536e-01],
%        [-1.04575666e+00, -3.53187416e+00,  1.11917812e+00,         5.18082854e+00],
%        [ 1.90240812e-01,  1.58535650e+00,  1.46422345e-01,         1.70734863e-01],
%        [ 2.35869591e-01,  5.87857465e-01, -5.62176691e-02,        -1.06924692e+00],
%        [ 4.68196103e-01,  1.44533643e+00, -6.11547421e-01,        -3.91009015e+00],
%        [ 4.65818455e-01,  4.85925612e-01, -1.32100200e+00,        -3.59281920e+00],
%        [ 3.00341867e-01, -1.44071696e-01,  7.11433617e-02,         1.66558237e-01],
%        [-8.47877285e-01, -7.19379976e-01,  4.29600025e-01,         9.81961203e-01],
%        [-7.28910565e-01, -1.37339725e+00,  3.18736161e-01,         2.69213957e+00],
%        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,         0.00000000e+00],
%        [ 4.78615837e-01,  1.01043378e+00, -2.03858351e-01,        -1.57965290e+00]];
%    
%    

Param = [[ 0.        ,  0.        ,  0.        ,  0.        ],
       [ -0.49914546        ,  1.91184578*0,  0.        , -0.98018837],
       [ 0.        , -1.31425242,  0.        ,  0.        ],
       [ 0.54684089        ,  0.        ,  0.        ,  0.        ],
       [ 0.        ,  1.34237693,  0.        ,  0.        ],
       [ 0.        , -3.85266109*0,  0.        ,  3.50638403],
       [ 0.        ,  1.8206167*0 ,  0.        ,  0.        ],
       [ 0.        ,  0.58984202*0,  0.        , -0.9310782 ],
       [ -0.85279204*0        ,  0.        ,  0.        , -1.54802118],
       [ 0.        ,  1.56780082*0, -1.30042476, -3.55083494],
       [ 0.37342044*0       ,  0.        ,  0.        ,  0.        ],
       [ -0.41710734*0       , -1.21705287*0,  0.        ,  0.        ],
       [ 0.        , -1.36317502*0,  0.        ,  2.15373567],
       [ 0.        ,  0.        ,  0.        ,  0.        ],
       [ 0.        ,  0.75211403*0,  0.        , -1.15583033]];
   
   
   
%% Start Algorithm

%Pegando posições
xi = Pose(:,2)-ones(length(Pose(:,2)),1)*Pose(1,2);
tx = Pose(:,1)-ones(length(Pose(:,1)),1)*Pose(1,1);
yi = Pose(:,4)-ones(length(Pose(:,4)),1)*Pose(1,4);
ty = Pose(:,3)-ones(length(Pose(:,3)),1)*Pose(1,3);

Linear_Vel(:,1)=Linear_Vel(:,1)-ones(length(Linear_Vel),1)*min_time;
Encoder_Vel(:,1)=Encoder_Vel(:,1)-ones(length(Encoder_Vel),1)*min_time;
Ang_Vel(:,1)=Ang_Vel(:,1)-ones(length(Ang_Vel),1)*min_time;
n=1;

data = load('data.mat');
dataLin = data.ures;
dataAng = data.wres;
dataCtrlLin = data.urefres;
dataCtrlAng = data.wrefres;




%Realizando sub-Sampling para melhor amostragem do sinal
for l = 1:length(Ang_Vel)
    for m = 0:14 
        LinRef(n+m, 1) = Linear_Vel(l,2);
        AngRef(n+m, 1) = Ang_Vel(l,2);
        if (l<length(Ang_Vel))
            Linear_Vel_Mod(n+m,1) = Linear_Vel(l,1)+(Linear_Vel(l+1,1)-Linear_Vel(l,1))*(m+1)/15;
            Ang_Vel_Mod(n+m,1) = Ang_Vel(l,1)+(Ang_Vel(l+1,1)-Ang_Vel(l,1))*(m+1)/15;
        end
    end
    n=n+15;
end

Encoder1 = Encoder_Vel(:,2);
Encoder2 = Encoder_Vel(:,4);
R=0.39; %Distância entre as duas rodas do robo
j=1;
ofs = 0;
Tmax = floor(max([Encoder_Vel(:,1)' Linear_Vel(:,1)']));

for i = 1:1:length(Encoder_Vel)
    VLin(j,1)=(Encoder1(i)+Encoder2(i))/2;
    %VLin(j,1)=(Encoder_Vel(i,2)+Encoder_Vel(i,4))/2;
    VAng(j,1)= -(Encoder1(i)-Encoder2(i))/R;
    %VAng(j,1)=(Encoder_Vel(i,2)-Encoder_Vel(i,4))*2/R;
    j=j+1;
end



%Sincronizando os dados
count = 1;
count2 = 1;
ind = 1;
for t=0:ts:Tmax
    [Y,I] = min(abs(Linear_Vel(:,1)-ones(8,1)*t));
    if I==1
        LinRefF(ind,1) = Linear_Vel(I,2);
    elseif (Y<0.05) 
        LinRefF(ind,1) = Linear_Vel(I,2);
    elseif( Linear_Vel(I,1)-t>0)
        LinRefF(ind,1) = Linear_Vel(I-1,2);
    else
        LinRefF(ind,1) = Linear_Vel(I,2);
    end
 
    
    
    [Ya,Ia] = min(abs(Ang_Vel(:,1)-ones(8,1)*t));
    if Ia==1
        AngRefF(ind,1) = Ang_Vel(Ia,2);
    elseif (Y<0.05) 
        AngRefF(ind,1) = Ang_Vel(Ia,2);
    elseif( Linear_Vel(I,1)-t>0)
        AngRefF(ind,1) = Ang_Vel(Ia-1,2);
    else
        AngRefF(ind,1) = Ang_Vel(Ia,2);
    end
    
    [Ye,Ie] = min(abs(Encoder_Vel(:,1)-ones(320,1)*t));
    if Ie==1
        VAngF(ind,1) = VAng(Ie);
        VLinF(ind,1) = VLin(Ie);
    elseif (Ye<0.05) 
        VAngF(ind,1) = VAng(Ie);
        VLinF(ind,1) = VLin(Ie);
    elseif(Encoder_Vel(Ie,1)-t>0)
        VAngF(ind,1) = VAng(Ie-1);
        VLinF(ind,1) = VLin(Ie-1);
    else
        VAngF(ind,1) = VAng(Ie);
        VLinF(ind,1) = VLin(Ie);
    end
    
    ind = ind+1;

end
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

xtt = 0:ts:Tmax;

figure, plot(xtt, us(1:(length(us)-1)))
hold on, plot(xtt, urefs, '--g')
figure, plot(xtt, ws(1:(length(ws)-1)))
hold on, plot(xtt, wrefs,'--g')


figure, subplot(211), plot(xtt, us(1:(length(us)-1)))
hold on, plot(xtt, urefs,'--r')
legend('u','u_r_e_f')
xlabel('seconds')
ylabel('m/sec')
subplot(212), plot(xtt, ws(1:(length(ws)-1)))
hold on, plot(xtt, wrefs,'--r')
legend('\omega','\omega_r_e_f')

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

figure, plot(xs,ys)
hold on, plot(xt,yt)
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

legend('SINDy','Encoder Odom' ,'Initial Point', 'End of SINDy', 'Location', 'NorthEastOutside')

    %text(.03,.03 , 'begin','Color','k','FontSize',14)

dataxtt = 0:ts:Tmax    ;
size_data = length(dataxtt);
    
figure
subplot(211), plot(xtt, dataLin, xtt, dataCtrlLin, 'k--')
subplot(212), plot(xtt, dataAng, xtt, dataCtrlAng, 'k--')

