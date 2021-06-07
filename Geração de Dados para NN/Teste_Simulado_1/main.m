clc, clear, close all
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
ts=0.25;
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
Tmax = floor(max([time_w_ref' time_w_ref' time_w_ref' time_w_ref']))-time_offset_end;
xtt=0:ts:Tmax;

%% Ajust PSi

ajust = 0;
psi_test(1) = psi_sim(1)+ajust;

for i=2:max_length
    if (psi_sim(i)-psi_sim(i-1)>3)
        ajust = -2*pi;        
    end
    if (psi_sim(i)-psi_sim(i-1)<-3)
        ajust = 0;        
    end
    psi_test(i) = psi_sim(i) + ajust;   
    
end

psi_sim = psi_test;


%% Plot data
figure(1)
plot(time_w_ref,w_ref,time_u_ref,u_ref)
figure(2)
plot(x_sim,y_sim)
figure(3)
plot(time_psi_sim, psi_sim,time_psi_sim, psi_test,time_psi_sim, psi_test+ones(1,max_length)*pi)
figure(4)
plot(time_x_sim,x_sim,time_y_sim,y_sim)


%% Calculate Velocities

for i=2:(max_length)
   dx(i)=(x_sim(i) - x_sim(i-1))/(time_x_sim(i)-time_x_sim(i-1)); 
   dy(i)=(y_sim(i) - y_sim(i-1))/(time_y_sim(i)-time_y_sim(i-1)); 
   u_sim(i) = sign(u_ref(i))*sqrt(dx(i)^2+dy(i)^2);
   psi_test(i) = atan(dy(i)/dx(i));
   psi_test(1) = psi_test(2);
   u_sim_y(i) = dy(i)/sin(psi_sim(i));
   w_sim(i) = (psi_sim(i) - psi_sim(i-1))/(time_psi_sim(i)-time_psi_sim(i-1)); 
end
dx(1)=dx(2);
dy(1)=dy(2);
w_sim(1)=w_sim(2);


figure(5)
subplot(211), plot(time_u_ref,u_ref,time_u_ref,u_sim)
subplot(212), plot(time_w_ref,w_ref,time_w_ref,w_sim)


%% Save data
ures = u_sim;
wres = w_sim;
urefres = u_ref';
wrefres = w_ref';
save('data','ures','wres','urefres','wrefres');


