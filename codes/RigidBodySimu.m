function [T,Y]=RigidBodySimu(Tmax)
% [T,Y]=RigidBodySimu(Tmax)
% This function simulates the 3D motion of a given rigid body. 
% Tmax - Maximum simulation time
% This file is part of a solution to an exercise given in class for the
% module ME518- Rigid Body Mechanics at the Dept. of Mechanical Engineering
% University of Peradeniya 
% copyright Maithripala DHS, University of Peradeniya, Sri Lanka

[XX1,YY1,ZZ1,XX2,YY2,ZZ2,XX3,YY3,ZZ3]=TheObject; %The object shape parametrs for plotting

h=0.1; %Simulation step size.
e1=[1 0 0]'; e2=[0 1 0]'; e3=[0 0 1]'; %Inertial directions



%Nominal system parameters
g=9.806; %Gravitational acceleration
II=[0.003 0 0;0 0.005 0;0  0 0.008]; % Moments of Inertia tensor in X frame
M=0.75; %1.1*M0; %0.85;


%Initial Conditions
th0=pi/2; drn0=[1 0 0]; %Initial angle and direction of the rotation matrix
n0=drn0/norm(drn0);
q00=cos(th0/2); w0=sin(th0/2)*n0; 
q0=[q00 w0]; %Initial quaternions of the rotation matrix

omega0=[1 1 2]; %Initial body angular velocity; 
o0=0*[-10 10 -10]; odot0=1*[1 1 1]; %Initial inertial dispalcement and velocity
x0=[q0 omega0 o0 odot0];

T=0:h:Tmax; %Simulation Time vector

[T,Y]=myode45(T,x0'); %Numerically Solving the Rigid body equations


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function [T,Y]=myode45(T,y0)
        nT=length(T);
        neq=length(y0);
        Y=zeros(nT,neq);
        t00=0;
        
        for i=1:nT
            T(i)=t00;
            
            figure(1)
            RealTimePlot(y0(8:10),y0(1:4)); hold off
            pause(.1)
            
            [y0]=OneIteration(neq,t00,y0); %Calling one step of the Runge-Kutta algorithm
            
            Y(i,:)=y0';
            
            
            t00=t00+h;
            
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ynew] = OneIteration(neq,t,y)
%One iteration step of the Runge-Kutta algorithm. This is stolen from MATLAB ode45
    f0=RigidBodyEqns(t,y);            

% Runge-Kutta parameters.
A = [1/5, 3/10, 4/5, 8/9, 1, 1];
B = [
    1/5         3/40    44/45   19372/6561      9017/3168       35/384
    0           9/40    -56/15  -25360/2187     -355/33         0
    0           0       32/9    64448/6561      46732/5247      500/1113
    0           0       0       -212/729        49/176          125/192
    0           0       0       0               -5103/18656     -2187/6784
    0           0       0       0               0               11/84
    0           0       0       0               0               0
    ];
f = zeros(neq,7);

f(:,1) = f0;

  % ONE STEP.

    hA = h * A;
    hB = h * B;
    
    for i=1:5 
     y_f_hB5=y+f*hB(:,i); 
    y_f_hB5(1:4)=y_f_hB5(1:4)/norm(y_f_hB5(1:4));
    f(:,i+1) = RigidBodyEqns(t+hA(i),y_f_hB5);
    end
   
    ynew = y + f*hB(:,6);
    
    ynew(1:4)=ynew(1:4)/norm(ynew(1:4));
    

    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function Ohat=hat(Omega)
        %The hat isomorphism
        Ohat=[0 -Omega(3) Omega(2);
              Omega(3) 0 -Omega(1);
              -Omega(2) Omega(1) 0];
    end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function xdot=RigidBodyEqns(t,X)
        
        q=X(1:4); %Quatornians
        Omega=X(5:7); %Body angular velocitles
        o=X(8:10); %Center of mass co-ordinates of the body in the inertial frame
        odot=X(11:13); %Velocity of center of mass of in the inertial frame
        
        [fu,Tu]=forceTorqueModel(X); %Forces (inertial) and force moments (body) acting on the body;
        
        %Equations of motion of the system
        qdot=0.5*[0 -Omega(1) -Omega(2) -Omega(3); Omega(1) 0 Omega(3) -Omega(2);
                     Omega(2) -Omega(3) 0 Omega(1); Omega(3) Omega(2) -Omega(1) 0]* q; %Rotation Equation in wuaternions

        Omegadot=inv(II)*(-(hat(Omega)*II*Omega)+Tu); %Omegadot equation
        
        oddot=fu/M; %o double dot equation

        xdot=[qdot; Omegadot; odot; oddot];
        
        
    end %End for Agent

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function[f,T]=forceTorqueModel(X)
        %The force (in the inertial frame) and force moment (in the body frame) model
        q=X(1:4); %Quatornians
        Omega=X(5:7); %Body angular velocitles
        o=X(8:10); %Center of mass co-ordinates of the body in the inertial frame
        odot=X(11:13); %Velocity of center of mass of in the inertial frame
        
        
        %Forces and Torques of the system
        fu=[0;0;0]; %External forces in the inertial frame
        fg=-M*(g)*e3; %Gravity force expressed in the inertial frame
        f=fu+0*fg; %the resultant force acting on the body expressed in the inertial frame
        T=[0;0;0]; %the resultant moment acting on the body expressed in the body frame
        
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    function [XX1,YY1,ZZ1,XX2,YY2,ZZ2,XX3,YY3,ZZ3]=TheObject
    %Object Geometric Parameters Needed only for plotting
    sc=5;
    rs=sc*.005; rd=sc*.025; lz=sc*.250;

    L=0*lz; % L - Lenght from the pivot point to the center of mass. Center of mass is assmed to be on the z-axis.
    PL=0.5*lz; % PL - Length from the bottom to the CM

    lg=(L+PL)/lz;
    sr=1/10;
    rr1=ones(size(0:sr:1));
    nr=length(rr1);
    ng=round((nr-1)*lg);
    rr(1:ng)=rs*rr1(1:ng);
    rr(ng:ng+0.2/sr)=rd*rr1(ng:ng+0.2/sr);
    rr(ng+0.2/sr+1:nr)=rs*rr1(ng+0.2/sr+1:nr);

    [XX,YY,ZZ]=cylinder(rr,40);
    ZZ=(lz*ZZ-PL);
    
    
        th01=pi/2; drn01=[1 0 0]; q01=[cos(th01/2) sin(th01/2)*drn01]; 
        Rb1=eye(3,3)+2*q01(1)*hat(q01(2:4))+2*hat(q01(2:4))^2;
        
        th02=pi/2; drn02=[0 1 0]; q02=[cos(th02/2) sin(th01/2)*drn02];
        Rb2=eye(3,3)+2*q02(1)*hat(q02(2:4))+2*hat(q02(2:4))^2;
        
        
        th03=0; drn03=[1 0 0]; q03=[cos(th03/2) sin(th03/2)*drn03]; %in_Euler_para(pi/4,[0 0 1]);
        Rb3=eye(3,3)+2*q03(1)*hat(q03(2:4))+2*hat(q03(2:4))^2;
        
        XX1=Rb1(1,1)*XX+Rb1(1,2)*YY+Rb1(1,3)*ZZ;
        YY1=Rb1(2,1)*XX+Rb1(2,2)*YY+Rb1(2,3)*ZZ;
        ZZ1=Rb1(3,1)*XX+Rb1(3,2)*YY+Rb1(3,3)*ZZ;
    
        XX2=Rb2(1,1)*XX+Rb2(1,2)*YY+Rb2(1,3)*ZZ;
        YY2=Rb2(2,1)*XX+Rb2(2,2)*YY+Rb2(2,3)*ZZ;
        ZZ2=Rb2(3,1)*XX+Rb2(3,2)*YY+Rb2(3,3)*ZZ;
    
        rr2(1:ng)=rs*rr1(1:ng);
        [XX30,YY30,ZZ30]=cylinder(rr2,40);
        ZZ30=((lz/2)*ZZ30);
    
        XX3=Rb3(1,1)*XX30+Rb3(1,2)*YY30+Rb3(1,3)*ZZ30;
        YY3=Rb3(2,1)*XX30+Rb3(2,2)*YY30+Rb3(2,3)*ZZ30;
        ZZ3=Rb3(3,1)*XX30+Rb3(3,2)*YY30+Rb3(3,3)*ZZ30;
    end %TheObject

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
    function out=RealTimePlot(oQuad,qQuad)
        
        RQ=eye(3,3)+2*qQuad(1)*hat(qQuad(2:4))+2*hat(qQuad(2:4))^2;
        XX1t=oQuad(1)+RQ(1,1)*XX1+RQ(1,2)*YY1+RQ(1,3)*ZZ1;
        YY1t=oQuad(2)+RQ(2,1)*XX1+RQ(2,2)*YY1+RQ(2,3)*ZZ1;
        ZZ1t=oQuad(3)+RQ(3,1)*XX1+RQ(3,2)*YY1+RQ(3,3)*ZZ1;
    
        
        XX2t=oQuad(1)+RQ(1,1)*XX2+RQ(1,2)*YY2+RQ(1,3)*ZZ2;
        YY2t=oQuad(2)+RQ(2,1)*XX2+RQ(2,2)*YY2+RQ(2,3)*ZZ2;
        ZZ2t=oQuad(3)+RQ(3,1)*XX2+RQ(3,2)*YY2+RQ(3,3)*ZZ2;
        
        XX3t=oQuad(1)+RQ(1,1)*XX3+RQ(1,2)*YY3+RQ(1,3)*ZZ3;
        YY3t=oQuad(2)+RQ(2,1)*XX3+RQ(2,2)*YY3+RQ(2,3)*ZZ3;
        ZZ3t=oQuad(3)+RQ(3,1)*XX3+RQ(3,2)*YY3+RQ(3,3)*ZZ3;

        
        surf(XX1t,YY1t,ZZ1t)
        hold on
        surf(XX2t,YY2t,ZZ2t)
        hold on
        surf(XX3t,YY3t,ZZ3t)
        axis([-1+oQuad(1) 1+oQuad(1)  -1+oQuad(2) 1+oQuad(2) -1+oQuad(3) 1+oQuad(3)])
        hold off    
        axis equal
        %axis(1.25*[-100 100  -10 10 -10 10])
        %light('Position',[100 100 100]);
        %material shiny
        
     end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
end %End for RigidBodySimu
%----------------------------------------------------------------------    
