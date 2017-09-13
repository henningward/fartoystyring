clear all;
clc;
close all;




%constants
deg2rad = pi/180;   
rad2deg = 180/pi;

%init
t = 0;
pos = [0 0 0]';
Theta = [-1.0 2.0 0.0]'*deg2rad;
omega = [0.0 0.0 0.0]';
delta = @deltafunc;


%model parameters
U = 1.5;
w = 360*U/(2*pi*100) * deg2rad;

%current parameters
U_c = 0.6;
alpha_c = 10*deg2rad;
beta_c = 45*deg2rad;


R_nb = Rzyx(Theta(1),Theta(2),Theta(3)); %http://planning.cs.uiuc.edu/node102.html


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
table = zeros(N+1,12);
angle_table = zeros(N+1, 3);


for i = 1:N+1,
    t = (i-1)*h;
    [J, R_nb, J2] = eulerang(Theta(1), Theta(2), Theta(3));
    
    %velocity of body in NED frame, relative to CURRENT
    V_r_n = [U*cos(omega(3)*t) U*sin(omega(3)*t) 0]';
    
    
    %ALTERNATIVE: assuming that the velocity is given in BODY frame, relative to CURRENT
    V_r_n = R_nb * [U*cos(omega(3)*t) U*sin(omega(3)*t) 0]';
    
    
    
    %velocity of body in NED frame, relative to NED
    V_b_n = V_r_n + V_nc_vect;
    V_b_n = V_r_n;
    
    %velocities in BODY frame, relative to NED
    V_b_b  = inv(R_nb) * V_b_n;
    V_r_b = inv(R_nb) * V_r_n;
    
    %calculation of position at current timestep in NED frame
    pos = pos + h*V_b_n;
    
    %calculation of speed, equal to the norm of velocity in BODY frame
    speed = norm(V_b_n);
    
    %calculation of relative speed, equal to the norm of velocity in BODY frame
    speed_r = norm(V_r_n);
    
    crab_angle = asin(V_b_b(2)./speed) .*rad2deg;
    sideslip_angle = asin(V_r_b(2)./speed_r).*rad2deg; 
    course_angle = (Theta(3)*rad2deg + crab_angle);
 
    
    
    
    omega_dot = A*omega+B*Theta+C*delta(t)*deg2rad;
    omega = omega + h * omega_dot; 

    
    Theta_dot = (J2 * omega);
    Theta = Theta + h * Theta_dot;

    
    table(i,:) = [t V_r_n' pos' speed omega' delta(t)];
    angle_table(i,:) = [course_angle crab_angle sideslip_angle];
end

t         = table(:,1);
rel_speed = table(:,2:4);
position  = table(:,5:7);
speed     = table(:,8);
omega     = table(:,9:11);
delta     = table(:,12);
course_angle = angle_table(:,1);
crab_angle = angle_table(:,2);
sideslip_angle = angle_table(:,3);


figure()
plot(position(:,2), position(:,1)),xlabel('East'),ylabel('North'),title('position (velocity in BODY, no current)'),grid

figure()
plot(t, rel_speed),xlabel('t'),ylabel('m/s'),title('relative velocities (no current)'),grid
legend('u', 'v', 'w')

figure()
plot(t, speed),xlabel('t'),ylabel('m/s'),title('speed (no current)'),grid


figure()
plot(t, course_angle), xlabel('t'),ylabel('grad'),title('angles (no current)'),grid
hold on;
plot(t, crab_angle),xlabel('t'),ylabel('grad'),grid
plot(t, sideslip_angle, '-'),xlabel('t'),ylabel('grad'),grid
legend('course angle', 'crab angle', 'sideslip angle')
hold off;

figure()
hold on;
plot(t, omega*rad2deg), xlabel('t'),ylabel('grad'),title('omega (no current)'),grid
plot(t, K*delta), xlabel('t'),ylabel('grad'),title('omega (no current)'),grid
legend('p', 'q', 'r', 'K*delta')
hold off;

function d = deltafunc(t)
    if t < 700
        d = 5;
    else
        d = 10;
    end  
end



