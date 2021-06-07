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
ts=0.01;
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

Pose = loadLidarPose();

[Linear_Vel, Ang_Vel] = loadControlData();

Encoder_Vel = loadEncoderVel();

min_time = min([Ang_Vel(:,1)']);
    
%PLOT PROP
LSIZE = 18; %LEGEND FONT SIZE


%% Parameters
% Param = [[  0.        ,  -0.10760704,   0.        ,   0.        ],
%        [  0.06917788,   4.23369311,   0.4347935 ,  -4.21513845],
%        [  0.        ,  -0.0774988 ,   0.        ,  -0.29828512],
%        [  0.1870344 ,  -0.13378744,   0.18581482,   0.78672804],
%        [  0.        ,   1.02349919,   0.        ,  -0.29630162],
%        [ -5.84110701, -34.35105878,  -8.49267862,  16.12772837],
%        [ -0.49644598,  -1.29089081,  -0.52905752,  -3.74018838],
%        [  0.        ,   0.28605296,   0.        ,   0.        ],
%        [  3.51463779,   4.34022873,   2.01760664,  17.10436647],
%        [  0.46321737,  -1.69164664,   0.15267557,   3.08429836],
%        [  0.        ,   0.        ,   0.        ,   0.08018026],
%        [  1.28673793,  -8.2434806 ,   0.89991718,   7.53105563],
%        [  0.        ,  -0.07756114,   0.        ,  -0.83702762],
%        [  0.        ,   0.        ,   0.        ,   0.        ],
%        [  0.        ,   0.07854544,   0.        ,   0.42593395]];


% Param = [[ 0.        ,  0.        ,  0.        ,  0.        ],
%        [-0.20568214, -0.13542562, -0.17589583, -2.13730078],
%        [ 0.        ,  0.        ,  0.        , -0.51842597],
%        [ 0.32262224,  0.46479525,  0.        ,  0.42155272],
%        [ 0.        ,  0.66857589,  0.        , -0.13761144],
%        [ 0.        ,  0.        ,  0.        , -0.21373008],
%        [-0.6735855 , -3.31261086,  0.        , -2.19680218],
%        [ 0.21309494,  0.23572252,  0.        ,  0.        ],
%        [-1.07951951, -3.77026173,  1.43828856, 16.53812664],
%        [ 1.19953928,  1.33966581, -0.4565925 ,  5.50806456],
%        [ 0.        ,  0.        ,  0.        ,  0.        ],
%        [ 0.81800516, -2.53978921,  0.30697045,  4.46832938],
%        [ 0.        ,  0.46803971,  0.        , -0.61779279],
%        [ 0.        ,  0.        ,  0.        ,  0.        ],
%        [-0.34003271,  0.45489688,  0.        , -0.40979645]];


% %SECOND BEST(maxAng = max(VAngF)/max(Ang_Vel(:,2))/1.7; no changes in
% VLinF cutoff = 0.15

% Param = [[  0.        ,   0.16268234,   0.        ,   0.        ],
%        [  1.3585989 ,  -0.12795017,   0.        ,   5.67073161],
%        [  0.        ,   0.32416064,   0.        ,  -0.43329866],
%        [  0.        ,  -2.55252296,   0.        ,  -0.61293584],
%        [ -0.08417305,   0.30882668,   0.        ,  -0.15431171],
%        [-13.40530708,  -9.05413358,  -1.69336269, -23.9583636 ],
%        [  0.        ,  -6.67509673,   1.23678381,  -2.03137585],
%        [  0.        ,  -1.15316085,   0.        ,  -0.5178325 ],
%        [  0.59553445,  19.47707886,   1.32325552, -25.2896214 ],
%        [ -0.62249625,  -2.27063921,   0.72721709,   5.78574369],
%        [  0.        ,  -0.26014321,   0.        ,   0.        ],
%        [  0.        ,  -0.95157111,   0.        ,  -0.66096823],
%        [  0.16944362,   0.20479355,   0.        ,   1.02462429],
%        [  0.        ,   0.        ,   0.        ,   0.        ],
%        [ -0.21288629,  -0.50770461,   0.        ,   0.        ]];


%[1 us(i) ws(i) urefs(i) wrefs(i) us(i)^2 us(i)*ws(i) ws(i)^2 us(i)*urefs(i)...
%ws(i)*urefs(i) urefs(i)^2 us(i)*wrefs(i) ws(i)*wrefs(i) urefs(i)*wrefs(i) wrefs(i)^2]';

% Param = [[  0.        ,   0.16981114,   0.        ,   0.        ],
%        [  1.39454243,   0.        ,   0.        ,   5.8955229 ],
%        [  0.        ,   0.33017432,   0.        ,  -0.3961858 ],
%        [  0.        ,  -2.68806438,   0.        ,  -0.61989226],
%        [ -0.07646345,   0.32169476,   0.        ,  -0.14155356],
%        [-13.60224605, -11.28628149,  -1.67862231, -26.00269358],
%        [  0.17849673,  -6.94396913,   1.18081446,  -2.16869917],
%        [  0.        ,  -1.0944762 ,   0.        ,  -0.47787301],
%        [  0.        ,  21.03232906,   1.24929888, -25.93893212],
%        [ -0.63400373,  -2.2945244 ,   0.69583279,   5.44466142],
%        [  0.        ,  -0.27395707,   0.        ,   0.        ],
%        [  0.        ,  -0.85396267,  -0.13838766,  -0.83289153],
%        [  0.13775852,   0.20562026,   0.        ,   0.9329239 ],
%        [  0.        ,   0.        ,   0.        ,   0.        ],
%        [ -0.2289089 ,  -0.54230415,   0.        ,   0.        ]];

% Param = [[ 0.00000000e+00,  1.41994180e-01,  0.00000000e+00,        -4.10200371e-02],
%        [ 1.31292966e+00, -4.56831784e-02,  0.00000000e+00,         7.52666184e+00],
%        [ 3.75606630e-02,  3.30279133e-01, -4.32106160e-02,        -4.77357561e-01],
%        [ 0.00000000e+00, -2.24047533e+00,  3.70970131e-02,        -2.91484908e-01],
%        [-7.14730057e-02,  2.68258549e-01,  4.22905768e-02,        -1.47911847e-01],
%        [-1.48655634e+01, -1.13009767e+01, -1.79669950e+00,        -3.72894167e+01],
%        [-3.98147713e-01, -7.70859824e+00,  2.18755650e+00,        -2.30649951e+00],
%        [-4.10938209e-02, -1.31386943e+00,  1.02754286e-01,        -5.64741300e-01],
%        [ 5.73180644e-01,  1.95931980e+01,  6.82608080e-01,        -3.31329879e+01],
%        [-7.76539334e-01, -2.29756634e+00,  8.84983338e-01,         6.17515221e+00],
%        [ 0.00000000e+00, -2.28340531e-01,  0.00000000e+00,        -2.97070080e-02],
%        [ 0.00000000e+00, -7.84074860e-01, -6.34037311e-01,        -7.93952712e-01],
%        [ 1.47478825e-01,  2.04511581e-01, -9.57468110e-02,         1.16576180e+00],
%        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,         0.00000000e+00],
%        [-2.00308476e-01, -4.52539696e-01, -3.42597564e-02,         0.00000000e+00]]

% %%%THE BEST %%% 
Param = [[-4.42397696e-02, -1.05484371e-01,  4.04786134e-02,         1.18449670e-01],
       [-1.06259147e+00, -4.60504235e+00, -1.97793888e+00,        -5.52446053e+00],
       [-1.14468955e-01, -7.28169562e-01,  7.56103758e-03,        -1.05793102e+00],
       [ 8.48726563e-01,  3.28616319e+00,  8.13517393e-01,         6.18893545e-01],
       [ 1.17833904e-01,  9.36231371e-01,  2.54448349e-02,         8.03436645e-01],
       [-7.14433336e+00,  3.95570152e+01, -6.54696697e-01,         1.38560525e+01],
       [-4.95547088e-01, -8.28720109e+00,  1.16180258e+00,        -2.67102667e+00],
       [ 3.25935011e-01, -2.69530099e-01, -6.46974442e-01,        -7.88641554e-01],
       [ 1.38495176e+01, -1.74124217e+01,  7.37872502e+00,         2.17360206e+01],
       [ 2.81427863e-01,  1.05709467e+01, -2.95994058e+00,         3.46588717e+00],
       [ 8.64989101e-02,  3.34912970e-01,  8.29105284e-02,         6.30752229e-02],
       [ 1.92484376e+00,  9.66304042e+00, -3.26201130e+00,        -1.41590632e+01],
       [-4.23408972e-01, -1.33760729e-01,  9.62763389e-01,         1.09093258e+00],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,         0.00000000e+00],
       [ 5.11672986e-01,  1.60596691e+00, -1.86338740e-01,         8.78863531e-01]];

% Param = [[  0.        ,   0.        ,   0.        ,   0.        ],
%        [  0.89691859,   2.79050563,   0.55774547,   6.39802454],
%        [  0.        ,   0.        ,   0.        ,  -0.55873938],
%        [  0.        ,  -1.45904846,   0.        ,  -0.81628563],
%        [  0.        ,   0.1838038 ,   0.        ,   0.        ],
%        [ -8.75270855, -23.95022329,  -8.83341001, -32.80481281],
%        [ -1.10629165,  -3.44875556,   0.71496923,  -1.35692905],
%        [  0.        ,  -0.73735413,   0.        ,  -0.34282002],
%        [  0.        ,  11.52175545,   3.17308042, -21.55428322],
%        [ -0.1816754 ,  -1.55670803,   0.53324682,   6.49983622],
%        [  0.        ,   0.        ,   0.        ,   0.        ],
%        [ -1.52475672,  -0.51816764,   0.        ,   2.48340253],
%        [  0.        ,   0.84506572,   0.        ,   0.43010562],
%        [  0.        ,   0.        ,   0.        ,   0.        ],
%        [  0.        ,   0.        ,   0.        ,  -0.36772538]];


% %BEST BELOW
% Param = [[ 0.        ,  0.        ,  0.        ,  0.        ],
%        [-0.20568214, -0.13542562, -0.17589583, -2.13730078],
%        [ 0.        ,  0.        ,  0.        , -0.51842597],
%        [ 0.32262224,  0.46479525,  0.        ,  0.42155272],
%        [ 0.        ,  0.66857589,  0.        , -0.13761144],
%        [ 0.        ,  0.        ,  0.        , -0.21373008],
%        [-0.6735855 , -3.31261086,  0.        , -2.19680218],
%        [ 0.21309494,  0.23572252,  0.        ,  0.        ],
%        [-1.07951951, -3.77026173,  1.43828856, 16.53812664],
%        [ 1.19953928,  1.33966581, -0.4565925 ,  5.50806456],
%        [ 0.        ,  0.        ,  0.        ,  0.        ],
%        [ 0.81800516, -2.53978921,  0.30697045,  4.46832938],
%        [ 0.        ,  0.46803971,  0.        , -0.61779279],
%        [ 0.        ,  0.        ,  0.        ,  0.        ],
%        [-0.34003271,  0.45489688,  0.        , -0.40979645]];

% 
Param = [[  0.        ,   0.        ,   0.        ,   0.        ],
       [  0.        ,  -0.24003187,   0.43215682,   6.52286352],
       [  0.        ,  -0.08788545,  -0.06056912,  -0.58602672],
       [  0.15762507,  -0.48460663,   0.        ,  -0.7356187 ],
       [  0.        ,   0.3737364 ,   0.        ,  -0.18900661],
       [ -1.23034179,   2.70343974,  -5.50551203, -30.83557656],
       [  0.        ,   0.        ,   2.72678967,   1.64645638],
       [  0.10528189,  -0.1515699 ,   0.07315286,  -0.85753356],
       [  0.        ,   4.00523959,   0.90835078, -25.51457934],
       [ -0.96243796,  -2.42768109,   1.21694149,   6.17144747],
       [  0.        ,   0.        ,   0.        ,   0.        ],
       [ -1.36897299,  -1.98172927,  -0.08938443,   1.02598604],
       [  0.        ,   0.        ,   0.        ,   1.42681384],
       [  0.        ,   0.        ,   0.        ,   0.        ],
       [ -0.09556154,   0.15727783,  -0.06618849,  -0.26685892]];
   
   
   
%% Script

%Pegando posições
xi = Pose(:,2)-ones(length(Pose(:,2)),1)*Pose(1,2);
tx = Pose(:,1)-ones(length(Pose(:,1)),1)*Pose(1,1);
yi = Pose(:,4)-ones(length(Pose(:,4)),1)*Pose(1,4);
ty = Pose(:,3)-ones(length(Pose(:,3)),1)*Pose(1,3);


Ang_Vel(:,1)=Ang_Vel(:,1)-ones(length(Ang_Vel),1)*min(Ang_Vel(:,1));
Encoder_Vel(:,1)=Encoder_Vel(:,1)-ones(length(Encoder_Vel),1)*min(Encoder_Vel(:,1));
Linear_Vel(:,1)=Linear_Vel(:,1)-ones(length(Linear_Vel),1)*min(Linear_Vel(:,1));
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





Encoder1 = Encoder_Vel(:,2);
Encoder2 = Encoder_Vel(:,4);
R=0.39; %Distância entre as duas rodas do robo
j=1;

ofs=0;
%ofs = 11.5; %Offset
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
    elseif (Y<0.05) 
        AngRefF(ind,1) = Ang_Vel(Ia,2);
    elseif( Linear_Vel(I,1)-t>0)
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

figure, plot(0:ts:Tmax, us(1:(length(us)-1)))
hold on, plot(0:ts:Tmax, urefs, '--g')
figure, plot(0:ts:Tmax, ws(1:(length(ws)-1)))
hold on, plot(0:ts:Tmax, wrefs,'--g')



xtt = 0:ts:Tmax;
% maxAng = max(VAngF)/max(Ang_Vel(:,2))/1.5;
% VLinF = VLinF*1.2;
% VAngF = VAngF./maxAng;
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
% end 


psit(1)=-(15/180)*pi;
dxt(1)=0;
dyt(1)=0;
ind = 1;
xt(1)=0;
yt(1)=0;
for i=0:ts:(125-ts)
    ind=ind+1;
    psit(ind) = VAngF(ind)*ts + psit(ind-1);
    dxt(ind) = VLinF(ind)*cos(psit(ind));
    dyt(ind) = VLinF(ind)*sin(psit(ind));
    xt(ind) = dxt(ind)*ts + xt(ind-1);
    yt(ind) = dyt(ind)*ts + yt(ind-1);
end

VLinF = v_lin;
vAngF = v_ang;


figure, subplot(211), plot(0:ts:Tmax, us(1:(length(us)-1)))
hold on, plot(0:ts:Tmax, urefs,'--r')
plot(xtt,VLinF,'k')
legend('u_{SINDy}','u_{ref}','u_{enc}')
xlabel('seconds')
ylabel('m/sec')
subplot(212), plot(0:ts:Tmax, ws(1:(length(ws)-1)))
hold on, plot(0:ts:Tmax, wrefs,'--r')
plot(xtt,VAngF,'k')
lgd1 = legend('\omega_{SINDy}','\omega_{ref}','\omega_{enc}')
lgd1.FontSize = LSIZE;
xlabel('seconds')
ylabel('rad/sec')





figure, plot(-xs,-ys)
hold on, plot(-xi,-yi)
plot(xt,yt)
plot(0,0,'gs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5])

plot(-xi(length(xi)),-yi(length(yi)),'rs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5])

plot(-xs(length(xs)),-ys(length(ys)),'bs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5])

xlabel('meters')
ylabel('meters')

lgd2 = legend('SINDy','LIDAR Data', 'Encoder Odom' ,'Initial Point', 'End of LIDAR', 'End of SINDy', 'Location', 'NorthEastOutside')
lgd2.FontSize = LSIZE;
    %text(.03,.03 , 'begin','Color','k','FontSize',14)
    
figure, plot(-xi,-yi)
hold on, plot(0,0,'gs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5])

plot(-xi(length(xi)),-yi(length(yi)),'rs',...
    'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5])



xlabel('meters')
ylabel('meters')

legend('LIDAR Data', 'Initial Point', 'End of LIDAR', 'Location', 'NorthEastOutside')

figure,
ERROR_LIN = v_lin-us(1:length(VLinF));
ERROR_ANG = v_ang-ws(1:length(VLinF));
max(abs(ERROR_LIN))
max(abs(ERROR_ANG))

subplot(211), plot(xtt,ERROR_LIN,'k');
xlabel('seconds');
ylabel('m/s');
title('a) Linear Error');
subplot(212), plot(xtt,ERROR_ANG,'k');
xlabel('seconds');
ylabel('rad/s');
title('b) Angular Error');



