clc; clear; 
close all;

%% Par�metros do Rob�

    %Coeficientes

    theta=[0.2604 0.2509 -0.000499 0.9965 0.00263 1.0768];
    a=0.2;
    
    x(1)=1;
    y(1)=1;
    
    xd = 0;
    yd = 1;
    
    tolerance = 0.01;
    
    Param = [[ 0.00000000e+00;  8.31026351e-01; -5.38786311e-01;3.25862499e+01];
       [-1.26137835e+01; -3.61203955e+01; -4.31838275e+02;-1.19268005e+03];
       [ 0.00000000e+00; -6.66738972e+01; -8.35332347e+02;-6.51819415e+02];
       [ 3.02745679e-01;  3.14122713e-01;  6.77550159e+00; 1.04847566e+01];
       [ 0.00000000e+00;  3.84590858e+00;  4.48026786e+01; 3.61994194e+01];
       [ 9.10759805e+01;  2.87075007e+02;  8.25495642e+03; 6.12004664e+03];
       [-2.63454318e+01;  1.43188990e+02; -2.91152801e+03; 2.50392976e+03];
       [ 8.81237137e-01;  4.71211302e+00; -4.41570924e+02; 1.40131212e+03];
       [-2.24504955e+00; -3.28887779e+00; -3.59776564e+02; 1.59920325e+01];
       [ 1.49721473e-01; -2.34596827e+00;  1.74815428e+02;-1.20384420e+02];
       [ 0.00000000e+00;  0.00000000e+00;  3.67577407e+00;-1.25482879e+00];
       [ 7.37500620e-01; -7.35247833e+00;  1.51816814e+02; 7.64094113e+01];
       [ 0.00000000e+00; -3.53328855e-01;  5.55384427e+01;-1.50734534e+02];
       [ 0.00000000e+00;  8.68732924e-02; -8.02092847e+00; 1.61108469e+00];
       [ 0.00000000e+00;  0.00000000e+00; -1.72901045e+00; 3.58065680e+00]];
   
   Param2 = [[ 0.00000000e+00;  2.12451719e+00;  1.10333476e+01; 7.88625968e+01];
       [-1.22674785e+01; -1.11903344e+02; -1.55396849e+03;-4.90901169e+03];
       [ 1.38994980e-01; -8.14926259e+01;  2.02460530e+02;-1.22800218e+03];
       [ 2.49619830e-01;  9.67839379e-01;  1.74329972e+01; 4.34836906e+01];
       [ 0.00000000e+00;  4.99252187e+00; -7.78235764e+00; 7.98049985e+01];
       [ 1.16637523e+02;  8.93871136e+02;  1.30254266e+04; 3.99805776e+04];
       [-1.59002162e+01; -1.85027875e+02; -1.23606359e+03;-9.40650356e+03];
       [ 5.85091942e-01;  3.36715268e+01;  8.01302159e+01; 1.49665543e+03];
       [-2.43904719e+00; -9.50177983e+00; -1.53996962e+02;-3.88517287e+02];
       [ 0.00000000e+00;  2.64754399e-01; -1.16585581e+01; 1.90084711e+01];
       [ 0.00000000e+00;  0.00000000e+00; -7.63868068e-02;-2.52723720e-01];
       [ 6.12828801e-01;  1.59162290e+01;  2.33525620e+01; 6.78670390e+02];
       [ 0.00000000e+00; -2.22284230e+00; -3.09833692e+00;-1.29289445e+02];
       [ 0.00000000e+00; -9.60580481e-02;  5.75598351e-01;-5.03033879e+00];
       [ 0.00000000e+00;  0.00000000e+00;  8.03909586e-02; 1.95230326e+00]];

 Param3 = [[ 0.00000000e+00,  1.78643661e+00,  4.11778565e+00, 5.77697510e+01];
       [ 2.21627521e-01,  1.03021691e+00, -6.67841230e+00,  1.92935865e+01];
       [-4.15736071e+00,  7.96429223e+00,  1.60760589e+02,        -3.50749044e+02];
       [ 4.15321432e-01,  1.17461569e+00,  3.63856821e+01,         3.91391997e+01];
       [-9.63195725e+00, -1.68815990e+02, -1.01371909e+03,        -3.75414461e+03];
       [-2.10738504e+02,  3.66455695e+03,  6.40476961e+03,         5.51913117e+04];
       [-3.13696676e-01,  7.56274517e-01,  3.40515653e+01,        -2.38989758e+02];
       [ 0.00000000e+00,  8.37274734e-02,  2.65349304e-01,         1.43497643e+00];
       [ 0.00000000e+00,  0.00000000e+00,  1.70661815e+00,        -2.86630344e+00];
       [ 0.00000000e+00,  8.75033061e-01, -4.46820014e+01,         1.43186880e+01];
       [ 1.96877155e+00, -4.58944114e+01, -9.96965782e+01,        -8.50948565e+02];
       [-1.88996160e+00,  2.53203714e+01, -1.89862867e+02,         1.06288603e+02];
       [ 0.00000000e+00,  5.06990340e-02, -4.27002617e-01,         6.83055337e+00];
       [ 5.89922857e+00, -1.87365829e+01, -4.79070334e+02,         2.02173682e+03];
       [ 5.14011875e+01, -8.73712711e+02,  4.39943450e+03,        -2.99228895e+03];
       [ 0.00000000e+00,  0.00000000e+00, -1.82814192e+00,        -1.69061583e+01];
       [-1.74209150e-01,  7.71967882e-01,  1.45877223e+01,         1.78717482e+02];
       [ 0.00000000e+00, -6.49867297e-01,  5.03711516e-01,         1.90703506e+01];
       [ 9.01975648e-02, -2.12336321e+00,  4.45053558e+01,         4.31055370e+02];
       [ 1.33969795e+00,  4.18802594e+01, -1.61209324e+01,        -6.39851514e+02];
       [ 0.00000000e+00,  0.00000000e+00, -8.49206548e-02,        -5.48319085e-01];
       [ 0.00000000e+00, -1.29655918e-01, -4.63607960e+00,        -7.25551622e+00];
       [ 0.00000000e+00, -8.67617306e-01, -2.60317647e+00,         1.00209787e+01];
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,        -1.06459303e-01];
       [ 0.00000000e+00,  0.00000000e+00,  2.16753937e+00,         1.46591559e+01];
       [ 0.00000000e+00,  0.00000000e+00,  7.86280070e-02,         1.13843811e-01];
       [ 0.00000000e+00,  1.44418289e+00,  5.72543800e+01,         1.22569571e+02];
       [ 1.98002903e+00, -1.76609038e+01, -2.63595020e+02,        -4.56876067e+03];
       [-6.09281897e+01,  3.46748184e+02, -2.57490119e+03,         3.08655358e+03];
       [-1.16658850e+00,  2.57066194e+01, -3.68861558e+02,        -3.28348540e+03];
       [-3.38131363e+01, -3.80767806e+02,  1.26825967e+03,         2.83088405e+03];
       [-4.19743833e+02,  3.00148117e+03, -1.55222643e+04,         1.95453794e+05];
       [ 3.03705878e+03, -2.16733015e+04,  2.25540352e+04,        -2.09467701e+05];
       [ 2.20556633e+01, -1.31151280e+02,  3.27483467e+02,        -6.50608549e+03];
       [ 0.00000000e+00,  0.00000000e+00,  2.26014314e-02,         1.77092180e-01]];
   
 Param4 = [[ 0.00000000e+00,  0.00000000e+00,  9.23618782e-01,         5.11598550e+00];
       [-3.92860296e+00,  0.00000000e+00, -8.20045125e+01,        -3.29333272e+02];
       [ 0.00000000e+00, -4.43708732e+00, -2.73821558e+01,         4.81793739e+02];
       [ 2.66658503e-01,  0.00000000e+00,  3.75270502e+00,         2.41753284e+01];
       [ 0.00000000e+00,  2.58038310e-01,  4.82143785e-01,        -2.34632339e+01];
       [ 0.00000000e+00,  0.00000000e+00,  2.29718084e+02,        -6.17388308e+02];
       [ 0.00000000e+00,  0.00000000e+00, -3.17947597e+02,         2.92424623e+02];
       [ 0.00000000e+00,  0.00000000e+00, -3.77522736e+01,         4.56516432e+02];
       [ 0.00000000e+00,  0.00000000e+00, -1.92074521e+01,         1.04274178e+02];
       [ 0.00000000e+00,  0.00000000e+00,  3.10428037e+01,        -1.49297938e+02];
       [ 0.00000000e+00,  0.00000000e+00,  4.11539326e-01,        -4.46530366e+00];
       [ 0.00000000e+00,  0.00000000e+00,  4.75807639e+00,        -2.94732441e+00];
       [ 0.00000000e+00,  0.00000000e+00,  2.70630473e+00,        -4.35083201e+01];
       [ 0.00000000e+00,  0.00000000e+00, -1.02771160e+00,         5.65235407e+00];
       [ 0.00000000e+00,  0.00000000e+00, -3.32163017e-02,         1.08373085e+00]];
   
   % du dw duref dwref
   % 1 u0 u1 u2 u3 u0^2 u0u1 u1^2 u0u2 u1u2 u2^2 u0u3 u1u3 u2u3 u3^2
   % u0 = u
   % u1 = w
   % u2 = uref
   % u3 = wref

    %Valores M�ximos
    umax = 0.1;
    wmax = 1.745;
    dumax = 0.3;
    dwmax = 1.745;

    %Condi��es Iniciais
    %x0=0;
    %y0=0;
    %psi0=2*pi;

    psi(1)=0;
    u(1)=0;
    xs(1)=x(1);
    ys(1)=y(1);
    psis(1)=0;
    us(1)=0;
    ws(1)=0;
    us(1)=0;
    w(1)=0;
    du(1)=0;
    dw(1)=0;
    dus(1)=0;
    dws(1)=0;
    
    ts=0.01;
    i=1;
    
    

    %Par�metros Controlador

    kx=10;
    ky=10;
    lx=10;
    ly=10;

    %Destino
    %xd=1;
    %yd=1;
    %tolerance=0.1;

    %% Movimento Rob�
    while(abs(xd-x(i))>tolerance || abs(yd-y(i))>tolerance)
        
        
     %%Controlador

     uref(i)=tanh((yd-y(i))*ky/ly)*ly*sin(psi(i))+...
         tanh((xd-x(i))*kx/lx)*lx*cos(psi(i));
     wref(i)=(1/a)*(tanh((yd-y(i))*ky/ly)*ly*cos(psi(i))-...
         tanh((xd-x(i))*kx/lx)*lx*sin(psi(i)));
     
     urefs(i)=tanh((yd-ys(i))*ky/ly)*ly*sin(psis(i))+...
         tanh((xd-xs(i))*kx/lx)*lx*cos(psis(i));
     wrefs(i)=(1/a)*(tanh((yd-ys(i))*ky/ly)*ly*cos(psis(i))-...
         tanh((xd-xs(i))*kx/lx)*lx*sin(psis(i)));

     %%Modelo Din�mico

     %Velocidade Linear

     du(i+1)=theta(1)*uref(i)+(theta(3)/theta(1))*(w(i)^2)-...
         (theta(4)/theta(1))*u(i); 
     
     %Acelera�ao Linear
      % 1 u0 u1 u2 u3 u0^2 u0u1 u1^2 u0u2 u1u2 u2^2 u0u3 u1u3 u2u3 u3^2
     dus(i+1) = Param4(:,1)'*[1 us(i) ws(i) urefs(i) wrefs(i) us(i)^2 us(i)*ws(i) ws(i)^2 us(i)*urefs(i)...
          ws(i)*urefs(i) urefs(i)^2 us(i)*wrefs(i) ws(i)*wrefs(i) urefs(i)*wrefs(i) wrefs(i)^2]';
     
     u0 = u(i);
     u1 = w(i);
     u2 = uref(i);
     u3 = wref(i);
     
     table= [1;
             u3;
             u1;
             u2;
             u0;
             u0^2;
             u1*u3;
             u2^2;
             u2*u3;
             u1*u2;
             u0*u2;
             u0*u3;
             u3^2;
             u1^2;
             u0*u1;
             u1*u3^2;
             u0*u2*u3;
             u0*u3^2;
             u1^2*u3;
             u0*u1*u3;
             u2^2*u3;
             u1*u2*u3;
             u0*u2^2;
             u2^3;
             u1*u2^2;
             u2*u3^2;
             u1^2*u2;
             u0*u1*u2;
             u0^2*u2;
             u1^3;
             u0*u1^2;
             u0^2*u1;
             u0^3;
             u0^2*u3;
             u3^3];

     %dus(i+1) = Param3(:,1)'*table;
     %dus2(i+1) = Param2(:,1)*[1 u(i) w(i) uref(i) wref(i) u(i)^2 u(i)*w(i) w(i)^2 u(i)*uref(i)...
      %    w(i)*uref(i) uref(i)^2 u(i)*wref(i) w(i)*wref(i) uref(i)*wref(i) wref(i)^2];

     %Limite Acelera��o Linear
     %if du(i+1)>dumax
      %du(i+1)=dumax;
     %end
     %if du(i+1)<-dumax
      %du(i+1)=-dumax;
     %end
     
     u(i+1)=du(i+1)*ts + u(i); %Velocidade Linear
     us(i+1)=dus(i+1)*ts + us(i); 

     %Limite Velocidade Linear
     %if u(i+1)>umax
      %u(i+1)=umax;
     %end
     %if u(i+1)<-umax
      %u(i+1)=-umax;
     %end

     %Velocidade Angular

     dw(i+1)=theta(2)*wref(i)-(theta(6)/theta(2))*w(i)-...
         (theta(5)/theta(2))*u(i)*w(i); %Acelera��o Angular

     dws(i+1) = Param4(:,2)'*[1 us(i) ws(i) urefs(i) wrefs(i) us(i)^2 us(i)*ws(i) ws(i)^2 us(i)*urefs(i)...
          ws(i)*urefs(i) urefs(i)^2 us(i)*wrefs(i) ws(i)*wrefs(i) urefs(i)*wrefs(i) wrefs(i)^2]';
     %dws(i+1) = Param3(:,2)'*table;
     
     %Limite Acelera��o Angular

     %if dw>dwmax
         %dw=dwmax;
     %end
     %if dw<-dwmax
      %   dw=-dwmax;
     %end

     w(i+1)=dw(i+1)*ts + w(i); %Velocidade Angular
     ws(i+1)=dws(i+1)*ts + ws(i); %Velocidade Angular


     %Limite Velocidade Angular

     %if w(i+1)>wmax
      %w(i+1)=wmax;
     %end
     %if w(i+1)<-wmax
      %w(i+1)=-wmax;
     %end

     %%Modelo Cinem�tico

     dx(i+1)=u(i+1)*cos(psi(i))-a*w(i)*sin(psi(i));  %Velocidade x
     dxs(i+1)=us(i+1)*cos(psis(i))-a*ws(i)*sin(psis(i));  %Velocidade x
     x(i+1)=dx(i+1)*ts+x(i);  %Posi��o x
     xs(i+1)=dxs(i+1)*ts+xs(i);  %Posi��o x
     
     dy(i+1)=u(i+1)*sin(psi(i))+a*w(i)*cos(psi(i)); %Velocidade y
     dys(i+1)=us(i+1)*sin(psis(i))+a*ws(i)*cos(psis(i)); %Velocidade y
     
     y(i+1)=dy(i+1)*ts+y(i); %Posi��o y
     ys(i+1)=dys(i+1)*ts+ys(i); %Posi��o y
     
     psi(i+1)=w(i+1)*ts+psi(i); %Estado de rota��o
     psis(i+1)=ws(i+1)*ts+psis(i); %Estado de rota��o
     
     i=i+1;

    end

plot(x,y,'b', xs,ys,'r')
%figure
%plot(1:i,du,1:i,dus, '+');
%figure
%plot(1:i,dw,1:i,dws, '+')
