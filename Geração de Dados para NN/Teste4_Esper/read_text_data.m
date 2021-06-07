clc, clear, close all


%% Load Experiment Data

Encoder_Vel = loadEncoderVel();


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


%% Plot Data
figure(1)
plot(time_pid_left, data_pid_left), hold on
plot(time_pid_right, data_pid_right)
plot(time_encoder_left, data_encoder_left)
plot(time_encoder_right, data_encoder_right)

legend('PID_L_E_F_T', 'PID_R_I_G_H_T', 'ENCODER_L_E_F_T', 'ENCODER_R_I_G_H_T');


%% Calculate Linear and Angular Values
for i = 1:1:length(Encoder_Vel)
    VLin(j,1)=(data_encoder_left(i)+data_encoder_right(i))/2;
    VLin_ref(j,1)=(data_pid_left(i)+data_pid_right(i))/2;
    VAng(j,1)= -(data_encoder_left(i)-data_encoder_right(i))./R;
    VAng_ref(j,1)= -(data_pid_left(i)-data_pid_right(i))./R;
    j=j+1;
end

%% Plot Velocities
figure(2)
subplot(221), plot(time_pid_left, VLin_ref)
subplot(222), plot(time_pid_left, VLin)
subplot(223), plot(time_pid_left, VAng_ref)
subplot(224), plot(time_pid_left, VAng)


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

figure(3)
subplot(211), plot(xtt, VLinF, xtt, VLinF_ref,'k--')
subplot(212), plot(xtt, VAngF, xtt, VAngF_ref, 'k--')


psi(1)=0;
dx(1)=0;
dy(1)=0;
ind = 1;
x(1)=0;
y(1)=0;
for i=0:ts:(Tmax-ts)
    ind=ind+1;
    psi(ind) = VAngF(ind)*ts + psi(ind-1);
    dx(ind) = VLinF(ind)*cos(psi(ind));
    dy(ind) = VLinF(ind)*sin(psi(ind));
    x(ind) = dx(ind)*ts + x(ind-1);
    y(ind) = dy(ind)*ts + y(ind-1);
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

figure
plot(x,y,xr,yr)


wwres = VAngF'.*VAngF';
uwres = VLinF'.*VAngF';
ures = VLinF';
wres = VAngF';
urefres = VLinF_ref';
wrefres = VAngF_ref';


save('data','ures','wres','urefres','wrefres', 'wwres', 'uwres');