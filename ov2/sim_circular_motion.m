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

%init pos
pos = [0 0 0]';

%from FLOW to NED
u_nc = U_c*cos(alpha_c)*cos(beta_c);
v_nc = U_c*sin(beta_c);
w_nc = U_c*sin(alpha_c)*cos(beta_c);

%current vector in NED frame
V_nc_vect = [u_nc v_nc w_nc]';
    
%currents in BODY frame
V_bc_vect = inv(R_nb) * V_nc_vect;



%simulation parameters
N = 14000;
h = 0.1;

%memory allocation
table = zeros(N+1,11);
angle_table = zeros(N+1, 3);



%% WITHOUT CURRENT
for i = 1:N+1,
    t = (i-1)*h;
    %velocity of body in NED frame, relative to CURRENT
    V_r_n = [U*cos(w*t) U*sin(w*t) 0]';
    
    %velocity of body in NED frame, relative to NED
    V_b_n = V_r_n;
    
    %velocities in BODY frame, relative to NED
    V_b_b  = inv(R_nb) * V_b_n;
    V_r_b = inv(R_nb) * V_r_n;
    
    %{
    %velocity of body in NED frame, relative to NED
    V_nb = R_nb * V;
    
    %relative velocity of body in NED frame, relative to CURRENT
    V_nr = R_nb * V_r;
    %}
    
    
    %calculation of position at current timestep in NED frame
    pos = pos + h*V_b_n;
    
    %calculation of speed, equal to the norm of velocity in BODY frame
    speed = norm(V_b_b);
    
    %calculation of relative speed, equal to the norm of velocity in BODY frame
    speed_r = norm(V_r_n);
    
    psi = psi + h * w;
    R_nb = Rzyx(phi,theta,psi);
    
    crab_angle = asin(V_b_b(2)./speed) .*rad2deg;
    sideslip_angle = asin(V_r_b(2)./speed_r).*rad2deg; 
    course_angle = (psi*rad2deg + crab_angle);
    
    table(i,:) = [t V_r_n' V_r_b' pos' speed];
    
    angle_table(i,:) = [course_angle crab_angle sideslip_angle];
end


t         = table(:,1);
rel_speed_ned   = table(:,2:4);
rel_speed_body  = table(:,5:7);
position        = table(:,8:10);
speed           = table(:,11);

course_angle = angle_table(:,1);
crab_angle = angle_table(:,2);
sideslip_angle = angle_table(:,3);


figure(1)
plot(position(:,2), position(:,1)),xlabel('East'),ylabel('North'),title('position (no current)'),grid

figure(2)
subplot(311), plot(t, rel_speed_ned),xlabel('t'),ylabel('m/s'),title('relative velocities in NED (no current)'),grid
legend('u', 'v', 'w')
subplot(312), plot(t, rel_speed_body),xlabel('t'),ylabel('m/s'),title('relative velocities in BODY (no current)'),grid
legend('u', 'v', 'w')
subplot(313), plot(t, speed),xlabel('t'),ylabel('m/s'),title('speed (no current)'),grid


figure(3)
hold on;
plot(t, course_angle), xlabel('t'),ylabel('grad'),title('Angles (no current)'),grid
plot(t, crab_angle),xlabel('t'),ylabel('grad'),grid
plot(t, sideslip_angle, '--'),xlabel('t'),ylabel('grad'),grid
legend('course angle', 'crab angle', 'sideslip angle')
hold off;





%% WITH CURRENT

%init pos
pos = [0 0 0]';

%model parameters
phi = 0 * deg2rad;
theta = 2.0 * deg2rad;
psi = 30.0 * deg2rad;
U = 1.5;
w = 360*U/(2*pi*100) * deg2rad;


for i = 1:N+1,
    t = (i-1)*h;
    %velocity of body in NED frame, relative to CURRENT
    V_r_n = [U*cos(w*t) U*sin(w*t) 0]';
    
    %velocity of body in NED frame, relative to NED
    V_b_n = V_r_n + V_nc_vect;  
    %velocities in BODY frame, relative to NED
    V_b_b  = inv(R_nb) * V_b_n;
    V_r_b = inv(R_nb) * V_r_n;
    
    %{
    %velocity of body in NED frame, relative to NED
    V_nb = R_nb * V;
    
    %relative velocity of body in NED frame, relative to CURRENT
    V_nr = R_nb * V_r;
    %}
    
    
    %calculation of position at current timestep in NED frame
    pos = pos + h*V_b_n;
    
    %calculation of speed, equal to the norm of velocity in BODY frame
    speed = norm(V_b_b);
    
    %calculation of relative speed, equal to the norm of velocity in BODY frame
    speed_r = norm(V_r_n);
    
    psi = psi + h * w;
    R_nb = Rzyx(phi,theta,psi);
    
    crab_angle = asin(V_b_b(2)./speed) .*rad2deg;
    sideslip_angle = asin(V_r_b(2)./speed_r).*rad2deg; 
    course_angle = (psi*rad2deg + crab_angle);
    
    table(i,:) = [t V_r_n' V_r_b' pos' speed];
    
    angle_table(i,:) = [course_angle crab_angle sideslip_angle];
end


t         = table(:,1);
rel_speed_ned   = table(:,2:4);
rel_speed_body  = table(:,5:7);
position        = table(:,8:10);
speed           = table(:,11);

course_angle = angle_table(:,1);
crab_angle = angle_table(:,2);
sideslip_angle = angle_table(:,3);


figure(4)
plot(position(:,2), position(:,1)),xlabel('East'),ylabel('North'),title('position (current)'),grid

figure(5)
subplot(311), plot(t, rel_speed_ned),xlabel('t'),ylabel('m/s'),title('relative velocities in NED (current)'),grid
legend('u', 'v', 'w')
subplot(312), plot(t, rel_speed_body),xlabel('t'),ylabel('m/s'),title('relative velocities in BODY (current)'),grid
legend('u', 'v', 'w')
subplot(313), plot(t, speed),xlabel('t'),ylabel('m/s'),title('speed (current)'),grid


figure(6)
hold on;
plot(t, course_angle), xlabel('t'),ylabel('grad'),title('Angles (current)'),grid
plot(t, crab_angle),xlabel('t'),ylabel('grad'),grid
plot(t, sideslip_angle, '--'),xlabel('t'),ylabel('grad'),grid
legend('course angle', 'crab angle', 'sideslip angle')
hold off;


