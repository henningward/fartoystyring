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
N = 20000;
h = 0.1;

%memory allocation
table = zeros(N+1,8);
angle_table = zeros(N+1, 3);



%% WITHOUT CURRENT
for i = 1:N+1,
    t = (i-1)*h;
    %velocity of body in NED frame, relative to CURRENT
    V_r = [U*cos(w*t) U*sin(w*t) 0]';
    
    %velocity of body in NED frame, relative to NED
    V = V_r;
    
    
    %{
    %velocity of body in NED frame, relative to NED
    V_nb = R_nb * V;
    
    %relative velocity of body in NED frame, relative to CURRENT
    V_nr = R_nb * V_r;
    %}
    
    
    %calculation of position at current timestep in NED frame
    pos = pos + h*V;
    
    %calculation of speed, equal to the norm of velocity in BODY frame
    speed = norm(V);
    
    %calculation of relative speed, equal to the norm of velocity in BODY frame
    speed_r = norm(V_r);
    
    
    crab_angle = asin(V(2)./speed) .*rad2deg;
    sideslip_angle = asin(V_r(2)./speed_r).*rad2deg; 
    course_angle = (psi + crab_angle);
 
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















%% WITH CURRENT

%init pos
pos = [0 0 0]';

for i = 1:N+1,
    t = (i-1)*h;
    
    %velocity of body in NED frame, relative to CURRENT
    V_r = [U*cos(w*t) U*sin(w*t) 0]';
    
    %velocity of body in NED frame, relative to NED
    V = V_r + V_nc_vect;
    
    
    %{
    %velocity of body in NED frame, relative to NED
    V_nb = R_nb * V;
    
    %relative velocity of body in NED frame, relative to CURRENT
    V_nr = R_nb * V_r;
    %}
    
    
    %calculation of position at current timestep in NED frame
    pos = pos + h*V;
    
    %calculation of speed, equal to the norm of velocity in BODY frame
    speed = norm(V);
    
    %calculation of relative speed, equal to the norm of velocity in BODY frame
    speed_r = norm(V_r);
    
    
    crab_angle = asin(V(2)./speed) .*rad2deg;
    sideslip_angle = asin(V_r(2)./speed_r).*rad2deg; 
    course_angle = (psi + crab_angle);
 
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



