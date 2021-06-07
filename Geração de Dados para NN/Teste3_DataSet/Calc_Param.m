clc ,clear, close all


% Dataset 1
% Param =[[ 0.        ,  0.        ,  0.        ,  0.        ],
%        [-4.11679328,  0.        , -2.47667934,  0.        ],
%        [ 0.        , -2.7895407 ,  0.        , -3.31435825],
%        [ 4.09720399,  0.        ,  2.96593177,  0.        ],
%        [ 0.        ,  2.74547101,  0.        ,  2.83310155],
%        [ 2.4768517 ,  0.        , -2.53496524, -0.72870733],
%        [-0.33203732,  0.        , -1.49889688, -5.9038168 ],
%        [ 0.        , -0.57233808,  0.        ,  1.31035414],
%        [-1.80448627,  0.        ,  2.63113492,  0.        ],
%        [ 0.34435517,  1.16688652,  1.52737483,  5.68476628],
%        [-0.60559314,  0.        , -1.09461505,  0.65635615],
%        [ 0.        , -0.38392442,  1.03795968,  3.41540037],
%        [ 0.        ,  0.76715955,  0.        , -2.17255588],
%        [ 0.        , -0.54707584, -1.07192877, -4.59314969],
%        [ 0.        , -0.26415697,  0.        ,  1.00853624]];

% Simulated
Param = [ 0         0         0         0;
   -3.9050         0         0         0;
         0   -4.3729         0         0;
    3.9196         0         0         0;
         0    4.0707         0         0;
         0         0         0         0;
         0    0.0137         0         0;
    0.0030         0         0         0;
         0         0         0         0;
         0         0         0         0;
         0         0         0         0;
         0         0         0         0;
         0         0         0         0;
         0         0         0         0;
         0         0         0         0];

Param =  [[ 0.        ,  0.        ,  0.        ,  0.        ];
       [-3.54276788,  0.        , -3.41545629,  0.        ];
       [ 0.        , -2.73165688,  0.        , -3.55926029];
       [ 3.63719855,  0.        ,  3.29583016,  0.        ];
       [ 0.        ,  2.69672038,  0.        ,  2.64962331];
       [ 1.1552429 ,  0.        ,  1.78295883, -0.94001563];
       [ 0.        , -0.71373117, -2.18350641,  0.        ];
       [ 0.        ,  0.        ,  0.        ,  0.        ];
       [ 0.        ,  0.        , -1.55582279,  1.69947737];
       [ 0.        ,  0.93164479,  2.14549132, -0.95202043];
       [-1.31895036,  0.        ,  0.        , -0.71914137];
       [ 0.        ,  0.        ,  1.445174  ,  0.        ];
       [ 0.        ,  0.        ,  0.        , -1.47217603];
       [ 0.        ,  0.        , -1.44372139,  0.        ];
       [ 0.        ,  0.        ,  0.        ,  0.82193704]];
   
   Param = [[  0.        ,   0.        ,   0.        ,   0.        ];
       [ -3.81004728,   0.        ,   2.35766974,   0.        ];
       [  0.        ,  -3.97775455,   0.63087406,  -6.5194274 ];
       [  3.75336131,   0.        ,   2.06817336,   1.84668277];
       [  0.        ,   3.71961397,  -0.66233408,   4.41199409];
       [  0.72609241,   0.        , -16.51401966,  -1.4008918 ];
       [  0.        ,   0.40832466,   0.        , -20.29661407];
       [  0.        ,   0.        ,   1.1870736 ,  -3.70456178];
       [ -0.58465658,   0.        ,  11.1690569 ,   0.        ];
       [  0.        ,  -0.2154387 ,  -1.46882275,  12.21228288];
       [  0.        ,   0.        ,  -3.70197645,  -2.44844762];
       [  0.        ,   0.        ,  -0.29479718,   5.40677456];
       [  0.        ,   0.        ,  -1.65333025,   5.15300084];
       [  0.        ,   0.        ,   1.69898087,  -6.62381366];
       [  0.        ,   0.        ,   0.24100967,   0.        ]];
   
   
   Param = [[  0.        ,   0.        ,   0.        ,   0.        ];
       [ -3.80883257,   0.        ,   2.31346716,   0.        ];
       [  0.        ,  -3.89562609,   0.61473974,  -6.51931094];
       [  3.75338369,   0.        ,   2.12192036,   1.84672187];
       [  0.        ,   3.7190428 ,  -0.66892394,   4.41344328];
       [  0.7237843 ,   0.        , -16.25801748,  -1.40188998];
       [  0.        ,   0.        ,   0.        , -20.22538125];
       [  0.        ,   0.        ,   1.07575831,  -3.70273201];
       [ -0.58460766,   0.        ,  10.7111603 ,   0.        ];
       [  0.        ,   0.        ,  -1.40366909,  12.14235814];
       [  0.        ,   0.        ,  -3.49507948,  -2.4476682 ];
       [  0.        ,   0.        ,   0.        ,   5.40421991];
       [  0.        ,   0.        ,  -1.25682209,   5.1533787 ];
       [  0.        ,   0.        ,   1.38741007,  -6.62455235];
       [  0.        ,   0.        ,   0.        ,   0.        ]];

   
theta=[0.2604 0.2509 -0.000499 -0.9965 -0.00263 -1.0768];


tB = zeros(15,4);
tB(4,1)=1/theta(1);
tB(5,2)=1/theta(2);
tB(2,1)=-theta(4)/theta(1);
tB(3,2)=-theta(6)/theta(2);
tB(8,1)=theta(3)/theta(1);
tB(7,2)=-theta(5)/theta(2);

tB

c_theta(1) = 1/Param(4,1);
c_theta(2) = 1/Param(5,2);
c_theta(4) = Param(2,1)*c_theta(1);
c_theta(6) = Param(3,2)*c_theta(2);
c_theta(3) = Param(8,1)*c_theta(1);
c_theta(5) = Param(7,2)*c_theta(2);

% c_theta = [0.2429    0.3585         0    -0.9952         0    -0.9842];

c_theta

TH = [theta' c_theta']

TB = zeros(15,4);
TB(2,1)=Param(2,1);
TB(3,2)=Param(3,2);
TB(4,1)=Param(4,1);
TB(5,2)=Param(5,2);
TB(8,1)=Param(8,1);
TB(7,2)=Param(7,2);

TB


PARAMETRO(1,1) = 1/c_theta(1);
PARAMETRO(2,1) = 1/c_theta(2);
PARAMETRO(3,1) = c_theta(3)/c_theta(1);
PARAMETRO(4,1) = c_theta(4)/c_theta(1);
PARAMETRO(5,1) = c_theta(5)/c_theta(2);
PARAMETRO(6,1) = c_theta(6)/c_theta(2);


PARAMETRO