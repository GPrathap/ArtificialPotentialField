

function artifical_potential_filed_finder
format long
global arena_r;  
global arena_map;     
global qstart qgoal;  
global epsilon_gr;
global epsilon_gr_nf;
global epsilon_goal;
global epsilon;
global ObsTh;   % Q*
global GoalTh;  % Dgoal*
global infinity;  % Large value to be used as 'infinity'
global k;
global ROT_vector;
global ROT_angle;
global elliptic_scale;
global RT;

% Parameter values to be used for the homework ---
arena_r = 10 ;  % Radius ; x, y, z = Always @ origin
arena_map = [];
infinity = 1e100; % 1e5 eeh... 1e6 baya uzar...
epsilon_gr = 2e-3;      % local minima alg�lamak i�in epsilonun izin verilen de�eri
epsilon_gr_nf = 1e-6;
epsilon_goal = 1e-3;
epsilon = 1e-6;
ObsTh = 1;
GoalTh = 3;
k = 1 ;
ROT_vector = [1 1 1];
ROT_angle = pi/4;
RT = rotationmat3D(ROT_angle,ROT_vector);   % Rotation Matrix...
elliptic_scale = [1 2 3];                   % X Y Z axis scales for Sphere to Elliptic Conversion
init_arena();

tic
[x_ar, y_ar, z_ar, t_ar] = pot_field_ar( qstart, qgoal );     % Path planning with attractive and repulsive forces
exec_time_ar = toc;

tic

[x_nf, y_nf, z_nf, t_nf] = pot_field_nf( qstart, qgoal );     % Path planning with Navigation Function
exec_time_nf = toc;

trf_ar = t_ar(length(t_ar));
trf_nf = t_nf(length(t_nf));

figure(1);
clf;       
draw_arena;     %Draw only Arena for visualization, then followed path.
plot3( x_ar, y_ar, z_ar, 'o', 'MarkerFaceColor','b', ...
            'MarkerEdgeColor','b','MarkerSize',3);
plot3( x_nf, y_nf, z_nf, 'o', 'MarkerFaceColor','r', ...
            'MarkerEdgeColor','r','MarkerSize',3);

x_ar_lin = zeros(length(x_ar),1);   % Make the 2D array into 1D (easier for me to handle)
y_ar_lin = zeros(length(x_ar),1);
z_ar_lin = zeros(length(x_ar),1);
for i = 1:size(x_ar,1)
    for j=1:size(x_ar,2)
        x_ar_lin((i-1)*size(x_ar,2) + j)=x_ar(i,j);% Make the 2D array into 1D (easier for me to handle)
        y_ar_lin((i-1)*size(x_ar,2) + j)=y_ar(i,j);
        z_ar_lin((i-1)*size(x_ar,2) + j)=z_ar(i,j);
    end
end 

x_nf_lin = zeros(length(x_nf),1);
y_nf_lin = zeros(length(x_nf),1);
z_nf_lin = zeros(length(x_nf),1);        
for i = 1:size(x_nf,1)
    for j=1:size(x_nf,2)
        x_nf_lin((i-1)*size(x_nf,2) + j)=x_nf(i,j);
        y_nf_lin((i-1)*size(x_nf,2) + j)=y_nf(i,j);
        z_nf_lin((i-1)*size(x_nf,2) + j)=z_nf(i,j);
    end
end
        
pathl_ar=0; 
for i=1:(length(x_ar_lin)-1)
    pathl_ar = pathl_ar + sqrt((x_ar_lin(i)-x_ar_lin(i+1))^2 + ...
                                (y_ar_lin(i)-y_ar_lin(i+1))^2 + ...
                                (z_ar_lin(i)-z_ar_lin(i+1))^2);
end
pathl_nf=0;        
for i=1:(length(x_nf_lin)-1)
    pathl_nf = pathl_nf + sqrt((x_nf_lin(i)-x_nf_lin(i+1))^2 + ...
                                (y_nf_lin(i)-y_nf_lin(i+1))^2 + ...
                                (z_nf_lin(i)-z_nf_lin(i+1))^2);
end





