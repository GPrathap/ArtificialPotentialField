function [x,y,z,t] = pot_field_nf( qstart, qgoal )

global infinity;
global epsilon_goal;
global epsilon_gr;

rob_pos = qstart;
h = 0.02;

% Tmax = 30;
% T=0:h:Tmax;
% [T, Y] = myode45(T,rob_pos');
% disp('---------------Y-----------------');
% disp(Y);

t0=0; 
tf=infinity; 
tspan=[t0,tf];
options = odeset('Event',@nf_event);
[T,Y] = ode45(@pot_nf,tspan,rob_pos,options);

x=Y(:,1);
    
function [T,Y]=myode45(T,y0)
        
        nT=length(T);
        neq=length(y0);
        Y=zeros(nT,neq);
        t00=0;
        
        for i=1:nT
            T(i)=t00;
            [y0]=OneIteration(neq,t00,y0); %Calling one step of the Runge-Kutta algorithm
            Y(i,:)=y0';
            t00=t00+h;
        end
end

function [ynew] =OneIteration(neq,t,y)

f0 = pot_nf(t, y);
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
     f(:,i+1) = pot_nf(t+hA(i),y_f_hB5);
    end
    ynew = y + f*hB(:,6); 
end


x=Y(:,1);
y=Y(:,2);
z=Y(:,3);
t=T;
end