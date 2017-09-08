clear all;
clc;
close all;




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


% Namato constants
zeta_p  = 0.1; 
zeta_q  = 0.2; 
omega_p  = 0.1; 
omega_q  = 0.05; 
K = 0.1; %Nomoto gain
T = 50; %Nomoto Time constant

%from FLOW to NED
u_nc = U_c*cos(alpha_c)*cos(beta_c);
v_nc = U_c*sin(beta_c);
w_nc = U_c*sin(alpha_c)*cos(beta_c);

%current vector in NED frame
V_nc_vect = [u_nc v_nc w_nc]';

%currents in BODY frame
V_bc_vect = inv(R_nb) * V_nc_vect;

%w_dot = A*omega+B*Theta*C*delta
A = [-2*zeta_p*omega_p      0                       0;
    0                       -2*zeta_q*omega_q       0;
    0                       0                       -1/T];

B = [-omega_p^2             0                       0;
    0                       -omega_q^2              0;
    0                       0                       0];

C = [0                      0                       K/T]';


%simulation parameters
N = 20000;
h = 0.1;

%memory allocation
table = zeros(N+1,8);
angle_table = zeros(N+1, 3);

%init
t = 0;
pos = [0 0 0]';
Theta = [-1.0 2.0 0.0]*deg2rad';
omega = [0.0 0.0 0.0]';
delta = @deltafunc;

for i = 1:N+1,
    t = (i-1)*h;
    %velocity of body in NED frame, relative to CURRENT
    V_r = [U*cos(omega(3)*t) U*sin(omega(3)*t) 0]';
    
    %velocity of body in NED frame, relative to NED
    V = V_r + V_nc_vect;
    V = V_r;
    
    %calculation of position at current timestep in NED frame
    pos = pos + h*V;
    
    %calculation of speed, equal to the norm of velocity in BODY frame
    speed = norm(V);
    
    %calculation of relative speed, equal to the norm of velocity in BODY frame
    speed_r = norm(V_r);
    
    crab_angle = asin(V(2)./speed) .*rad2deg;
    sideslip_angle = asin(V_r(2)./speed_r).*rad2deg; 
    course_angle = (psi + crab_angle);
 
    [J, J1, J2] = eulerang(Theta(1), Theta(2), Theta(3));
    Theta_dot = J2 * omega;
    Theta = Theta + h * Theta_dot;
    
    omega_dot = A*omega+B*Theta+C*delta(t)*deg2rad;
    omega = omega + h * omega_dot;        
    
    table(i,:) = [t V_r' pos' speed];
    angle_table(i,:) = [course_angle crab_angle sideslip_angle];
end

t         = table(:,1);
rel_speed = table(:,2:4);
position  = table(:,5:7);
speed     = table(:,8);

course_angle = angle_table(:,1);
crab_angle = angle_table(:,2);
sideslip_angle = angle_table(:,3);


figure()
plot(position(:,2), position(:,1)),xlabel('East'),ylabel('North'),title('position'),grid

figure()
plot(t, rel_speed),xlabel('t'),ylabel('m/s'),title('relative velocities'),grid
legend('u', 'v', 'w')

figure()
plot(t, speed),xlabel('t'),ylabel('m/s'),title('speed'),grid


figure()
plot(t, course_angle), xlabel('t'),ylabel('grad'),title('angles'),grid
hold on;
plot(t, crab_angle),xlabel('t'),ylabel('grad'),grid
plot(t, sideslip_angle),xlabel('t'),ylabel('grad'),grid
legend('course angle', 'crab angle', 'sideslip angle')
hold off;


function d = deltafunc(t)
    if t < 700
        d = 5;
    else
        d = 10;
    end  
end



