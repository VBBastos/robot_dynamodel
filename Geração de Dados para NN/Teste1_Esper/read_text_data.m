clc, clear, close all

Pose = loadLidarPose();

[Linear_Vel, Ang_Vel] = loadControlData();

Encoder_Vel = loadEncoderVel();

min_time = min([Ang_Vel(:,1)']);




%%%%% NEW %%%%%%%%%%%%%%%%

R=0.39; %Distância entre as duas rodas do robo
%%%% Encoder %%%%%
rate = 100;
mi = 13301;

time1 = Encoder_Vel(:,1);
llen = length(time1);
time1 = time1 - ones(llen,1)*min(time1);
Enc1 = Encoder_Vel(:,2);
Enc2 = Encoder_Vel(:,4);

Enc1 = resample(Enc1,time1,rate);
Enc2 = resample(Enc2,time1,rate);

v_lin = (Enc1 + Enc2)./2;
v_ang = (Enc2 - Enc1)./R;

v_lin = v_lin(1:mi)';
v_ang = v_ang(1:mi)';




ttime = 0:(1/rate):(1/rate)*(length(Enc1)-1);


%%%% REF %%%%%%%%

time2 = Linear_Vel(:,1);
time2 = time2 - ones(length(time2),1)*min(time2);
u_ref = Linear_Vel(:,2);
u_ref = resample(u_ref,time2,rate);

time3 = Ang_Vel(:,1);
time3 = time3 - ones(length(time2),1)*min(time3);
w_ref = Ang_Vel(:,2);
w_ref = resample(w_ref,time3,rate);





%%%%%%%%%%%%%%%%%%%%%%%%%%








%Pegando posições
xi = Pose(:,2)-ones(length(Pose(:,2)),1)*Pose(1,2);
tx = Pose(:,1)-ones(length(Pose(:,1)),1)*Pose(1,1);
yi = Pose(:,4)-ones(length(Pose(:,4)),1)*Pose(1,4);
ty = Pose(:,3)-ones(length(Pose(:,3)),1)*Pose(1,3);
psi(1) = 0;

for i=1:(length(Pose(:,1))-1)
    psi(i+1)= atan((yi(i+1)-yi(i))/(xi(i+1)-xi(i)));
    
end

%Calculando Velocidades

for i=1:(length(Pose(:,1))-1)
    dxi(i)=(xi(i+1)-xi(i))/(tx(i+1)-tx(i));
    dyi(i)=(yi(i+1)-yi(i))/(ty(i+1)-ty(i));
    ui(i)=abs(dxi(i)*cos(psi(i))+dyi(i)*sin(psi(i)));
    wi(i) = (psi(i+1)-psi(i))/(tx(i+1)-tx(i));
    
    tt(i) = tx(i);    
end

Linear_Vel(:,1)=Linear_Vel(:,1)-ones(length(Linear_Vel),1)*min(Linear_Vel(:,1));
Encoder_Vel(:,1)=Encoder_Vel(:,1)-ones(length(Encoder_Vel),1)*min(Encoder_Vel(:,1));
Ang_Vel(:,1)=Ang_Vel(:,1)-ones(length(Ang_Vel),1)*min(Ang_Vel(:,1));


subplot(211), plot(Linear_Vel(:,1),Linear_Vel(:,2),'*')
axis([-10 150 -0.15 0.15])
xlabel('seconds')
ylabel('m/sec')
subplot(212), plot(Ang_Vel(:,1),Ang_Vel(:,2), '*')
xlabel('seconds')
ylabel('rad/sec')
%title('Linear Vel')
figure
plot(Encoder_Vel(:,1),Encoder_Vel(:,2),'--')
hold on 
plot(Encoder_Vel(:,1),Encoder_Vel(:,4))
xlabel('seconds')
ylabel('m/sec')
legend('Left', 'Right');

%title('Encoder_Vel')

figure
plot(Ang_Vel(:,1),Ang_Vel(:,2), '*')
xlabel('seconds')
ylabel('rad/sec')

%title('Ang_Vel')
R=0.39; %Distância entre as duas rodas do robo
j=1;
ofs=0;
%ofs = 11.5; %Offset
Tmax = floor(max([Encoder_Vel(:,1)' Linear_Vel(:,1)']));

% %Calculando as Velocidades Linear e Angular do Robo
% [B,A]=butter(8,.2);

Encoder1 = Encoder_Vel(:,2);
Encoder2 = Encoder_Vel(:,4);


% Encoder1 = filtfilt(B,A,Encoder_Vel(:,2));
% 
% Encoder2 = filtfilt(B,A,Encoder_Vel(:,4));

for i = 1:1:length(Encoder_Vel)
    VLin(j,1)=(Encoder1(i)+Encoder2(i))/2;
    %VLin(j,1)=(Encoder_Vel(i,2)+Encoder_Vel(i,4))/2;
    VAng(j,1)= -(Encoder1(i)-Encoder2(i))./R;
    %VAng(j,1)=(Encoder_Vel(i,2)-Encoder_Vel(i,4))*2/R;
    j=j+1;
end
figure
subplot(211), plot(Encoder_Vel(:,1), VLin)
xlabel('seconds')
ylabel('m/sec')
subplot(212), plot(Encoder_Vel(:,1), VAng)
xlabel('seconds')
ylabel('rad/sec')

figure
plot(Encoder_Vel(:,1), VLin)
hold on
plot(Linear_Vel(:,1),Linear_Vel(:,2),'*')
title('VLin')
figure
plot(Encoder_Vel(:,1), VAng)
hold on
plot(Ang_Vel(:,1),Ang_Vel(:,2),'*')
title('VAng')
n=1;

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

tcount = 1;

%     for t = 0:0.25:125
%         for l=1:length(Linear_Vel)
%             
%         end
%         TestLin(tcount)
%         tcount = tcount + 1;
%     end
    
    



figure
plot(Encoder_Vel(:,1), LinRef(1:670,1))
title('LinRef')
figure
plot(Linear_Vel_Mod(1:660,1), LinRef(1:660,1))
title('LinRef Diff X')

%Sincronizando os dados
count = 1;
count1 = 1;
count2 = 1;
ind = 1;
ts = 0.01;
id_n = 0;

% % % % delay=0.0;
% % % % tref_=[Ang_Vel(2:end,1) Ang_Vel(2:end,1)+1e-6]';
% % % % tref_=[ Ang_Vel(1,1); tref_(:)+delay];
% % % % 
% % % % wref_=[Ang_Vel(:,2) Ang_Vel(:,2)]';
% % % % wref_=wref_(:);
% % % % wref_=wref_(1:end-1);
% % % % 
% % % % vref_=[Linear_Vel(:,2) Linear_Vel(:,2)]';
% % % % vref_=vref_(:);
% % % % vref_=vref_(1:end-1);
% % % % 
% % % % vel_=VLin;
% % % % rot_=VAng;
% % % % tenc_=Encoder_Vel(:,1);
% % % % 
% % % % tmax=max([tenc_;tref_]);
% % % % 
% % % % tabs=0:ts:tmax;
% % % % vel=interp1(tenc_,vel_,tabs);
% % % % rot=interp1(tenc_,rot_,tabs);
% % % % vref=interp1(tref_,vref_,tabs);
% % % % wref=interp1(tref_,wref_,tabs);
% % % % tabs(2:end)=tabs(2:end)+0.9;

for t=0:ts:Tmax 
    [Y,I] = min(abs(Linear_Vel(:,1)-ones(45,1)*t));
    if I==1
        LinRefF(ind,1) = Linear_Vel(I,2);
    elseif (Y<0.05) 
        LinRefF(ind,1) = Linear_Vel(I,2);
    elseif( Linear_Vel(I,1)-t>0)
        LinRefF(ind,1) = Linear_Vel(I-1,2);
    else
        LinRefF(ind,1) = Linear_Vel(I,2);
    end
 
    
    
    [Ya,Ia] = min(abs(Ang_Vel(:,1)-ones(45,1)*t));
    if Ia==1
        AngRefF(ind,1) = Ang_Vel(Ia,2);
    elseif (Ya<0.05) 
        AngRefF(ind,1) = Ang_Vel(Ia,2);
    elseif(Ang_Vel(Ia,1)-t>0)
        AngRefF(ind,1) = Ang_Vel(Ia-1,2);
    else
        AngRefF(ind,1) = Ang_Vel(Ia,2);
    end
    
    [Ye,Ie] = min(abs(Encoder_Vel(:,1)-ones(670,1)*t));
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

xtt = 0:ts:Tmax;
% maxAng = max(VAngF)/max(Ang_Vel(:,2))/1.5;
% VLinF = VLinF*1.2;
% VAngF = VAngF./maxAng;
% 
% 
% for i = 1:length(xtt)
%    if (VLinF(i)< .001)
%        VLinF(i)=0;
%    end
% %    if (VLinF(i)> .1019)
% %        VLinF(i)=.1019;
% %    end
%    if (abs(VAngF(i))< .01)
%        VAngF(i)=0;
% %    elseif (abs(VAngF(i))< .35)
% %        VAngF(i)=sign(VAngF(i))*.25;
% %    else
% %        VAngF(i)=sign(VAngF(i))*.35;
%    end
%    
%     
% end


figure, plot(xtt, LinRefF), title("LinRefF")
%figure, plot(xtt, VLin), title("VlinF")
figure, plot(xtt, AngRefF), title("AngRefF")
%figure, plot(xtt, VAng), title("VAngF")
%figure, plot(0:.25:125, VLinF-LinRefF ), title 


%Salvando dados em formato .mat
%ures=VLinF';
%wres=VAngF';

xtt = xtt(1:length(v_lin));
LinRefF = LinRefF(1:length(v_lin));
AngRefF = AngRefF(1:length(v_lin));

ures = v_lin;
wres = v_ang;
urefres=LinRefF';
wrefres=AngRefF';

% ures = vel;
% wres = rot;
% wrefres = wref;
% urefres = vref;

save('data','ures','wres','urefres','wrefres');
% save('data', 'vel', 'rot', 'vref', 'wref'); 

psi(1)=0;
dx(1)=0;
dy(1)=0;
ind = 1;
x(1)=0;
y(1)=0;


for i=0:ts:(Tmax -ts)
    ind=ind+1;
    psi(ind) = VAngF(ind)*ts + psi(ind-1);
    dx(ind) = VLinF(ind)*cos(psi(ind));
    dy(ind) = VLinF(ind)*sin(psi(ind));
    x(ind) = dx(ind)*ts + x(ind-1);
    y(ind) = dy(ind)*ts + y(ind-1);
end

figure
subplot(211), plot(xtt, v_lin, xtt, LinRefF, 'k--')
subplot(212), plot(xtt, v_ang, xtt, AngRefF, 'k--')

figure
plot(x,y)

figure
plot(xtt, LinRefF + AngRefF) 

% 
% figure,subplot(211),plot(tabs,vref,tabs,vel)
% subplot(212),plot(tabs,wref,tabs,rot)
