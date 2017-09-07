clear all;
clc;

%constants
deg2rad = pi/180;   
rad2deg = 180/pi;

%model parameters
phi = 0 * deg2rad;
theta = 2.0 * deg2rad;
psi = 30.0 * deg2rad;
U = 1.5;
w = 360*U/(2*pi*100) * deg2rad;

%current parameters
U_c = 0.6;
alpha_c = 10*deg2rad;
beta_c = 45*deg2rad;


R_nb = Rzyx(phi,theta,psi); %http://planning.cs.uiuc.edu/node102.html

%init pos
pos = [0 0 0]';

u_nc = U_c*cos(alpha_c)*cos(beta_c);
v_nc = sin(beta_c)*U_c;
w_nc = U*sin(alpha_c)*cos(beta_c);

%currents in NED frame
V_nc_vect = [u_nc v_nc w_nc]';

%currents in BODY frame
V_bc_vect = inv(R_nb) * V_nc_vect;



%simulation parameters
N = 50000;
h = 0.1;

%memory allocation
table = zeros(N+1,8);
angle_table = zeros(N+1, 3);

%% WITHOUT CURRENT
for i = 1:N+1,
    t = (i-1)*h;
    
    %velocity of body in BODY frame, relative to NED
    V_bb = [U*cos(w*t) U*sin(w*t) 0]';
    
    %velocity of body in NED frame, relative to NED
    V_nb = R_nb * V_bb;
    
    %calculation of position at current timestep in BODY frame
    pos = pos+h*V_nb;
    
    %calculation of speed, equal to the norm of velocity in BODY frame
    speed = norm(V_bb);
    
    
    crab_angle = asin(V_nb(2)/speed) *rad2deg;
    sideslip_angle = asin(V_nb(2)/speed)*rad2deg; 
    course_angle = (psi + crab_angle)*rad2deg;
    
    
    
    table(i,:) = [t V_nb' pos' speed];
    
    angle_table(1,:) = [course_angle crab_angle sideslip_angle];
end

t         = table(:,1);
rel_speed = table(:,2:4);
position  = table(:,5:7);
speed     = table(:,8);


figure(1)
plot(position(:,2), position(:,1)),xlabel('East'),ylabel('North'),title('position'),grid







%% WITH CURRENT

%init pos
pos = [0 0 0]';

for i = 1:N+1,
    t = (i-1)*h;
    
    %velocity of body in BODY frame, relative to NED
    V_bb = [U*cos(w*t) U*sin(w*t) 0]' + V_bc_vect;
    
    %velocity of body in NED frame, relative to NED
    V_nb = R_nb * V_bb;
    
    %calculation of position at current timestep in BODY frame
    pos = pos+h*V_nb;
    
    %calculation of speed, equal to the norm of velocity in BODY frame
    speed = norm(V_bb);
    
    
    crab_angle = asin(V_nb(2)/speed) *rad2deg;
    sideslip_angle = asin(V_nb(2)/speed)*rad2deg; 
    course_angle = (psi + crab_angle)*rad2deg;
    
    
    
    table(i,:) = [t V_nb' pos' speed];
    
    angle_table(1,:) = [course_angle crab_angle sideslip_angle];
end

t         = table(:,1);
rel_speed = table(:,2:4);
position  = table(:,5:7);
speed     = table(:,8);


figure(2)
plot(position(:,2), position(:,1)),xlabel('East'),ylabel('North'),title('position'),grid

