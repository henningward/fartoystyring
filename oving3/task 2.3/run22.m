%% Information 
% This file is only an example of how you can start the simulation. The
% sampling time decides how often you store states. The execution  time
% will increase if you reduce the sampling time.

% Please note that the file "pathplotter.m" (only used in the second part
% of the assignment) shows the ship path during the path following and
% target tracking part of the assignment. It can be clever to adjust the sampling
% time when you use that file because it draws a sketch of the ship in the
% North-East plane at each time instant. Having a small sampling time will
% lead to a lot over overlap in the ship drawing. 

% You should base all of your simulink models on the MSFartoystyring model
% and extend that as you solve the assignment. For your own sake, it is
% wise to create a new model and run file for each task. That is
% especially important in the problems you need to hand in since the files
% you deliver only should create the desired result in that task.

% The msfartoystyring.m file includes the ship model. You are not allowed
% to change anything within that file. You need to include that file in
% every folder where you have a simulink model based on
% "MSFartoystyring.slx". 

% WP.mat is a set of six waypoints that you need to use in the second part of
% the assignment. The north position is given in the first row and the east
% position in the second row. 

clear all
clc
close all


%%

rad2deg = 180/pi;
deg2rad = pi/180;

tstart=0;           % Sim start time
tstop=4000;        % Sim stop time
tsamp=10;           % Sampling time for how often states are stored. (NOT ODE solver time step)
                
p0=[1500 500];      % Initial position (NED)
v0=[6.63 0]';       % Initial velocity (body)
psi0=50*deg2rad;             % Inital yaw angle
r0=0;               % Inital yaw rate
c=1;                % Current on (1)/off (0)


init_controllers

%% task 2.1


%Loading waypoints
load('WP.mat')

wpt_posx = WP(1,:);
wpt_posy  =WP(2,:);
wpt_time = [0 20 40 60 80 100]; 
t = 0:1:max(wpt_time);

% method 1 - cubic Hermite interpolation
x_p = pchip(wpt_time,wpt_posx,t);
y_p = pchip(wpt_time,wpt_posy,t);

% method 2 - spline interpolation
x_s = spline(wpt_time,wpt_posx,t); 
y_s = spline(wpt_time,wpt_posy,t);
%{
%plotting method 1 vs method 2
figure()
subplot(311), plot(wpt_time,wpt_posx,'o',t,[x_p; x_s])
legend ('waypoint-x position', 'hermite-x position', 'spine-x position');
xlabel('time (s)')
ylabel('x- position (m)')
subplot(312), plot(wpt_time,wpt_posy,'o',t,[y_p; y_s])
legend ('waypoint-y position', 'hermite-y position', 'spine-y position');
xlabel('time (s)')
ylabel('y- position (m)')
subplot(313), plot(wpt_posy,wpt_posx,'o',y_p,x_p,y_s,x_s)
legend ('position', 'hermite position', 'spine position');
xlabel('y- position (m)')
ylabel('x- position (m)')

% method 3 - Dubins path

%Need to find maximum turning radius
sim MSFartoystyringtask21

pathplotter(p(:,1),p(:,2),psi,tsamp,100,tstart,tstop,0,zeros(2))
title('Path with maximum rudder and velocity to find turning radius')
%}
turningRadius = 365; %m
n_of_waypoints = length(WP(1, :));
R_bar = zeros(1, n_of_waypoints-2);

%{

figure()
for i = 1:n_of_waypoints-2
    prevWP = WP(:, i);
    currentWP = WP(:, i+1);
    nextWP = WP(:, i+2);
    
    u = (prevWP - currentWP)/(norm(prevWP - currentWP));
    v = (nextWP - currentWP)/(norm(nextWP - currentWP));
    
    w = (u + v)/(norm(u + v));
    a = acos((norm(v)^2 + norm(u+v)^2 - norm(u)^2)/(2*norm(v)*norm(u+v)));
    
    R_bar(i) = turningRadius / sin(a);
    center_pos = currentWP + (R_bar(i) * w);
    viscircles([center_pos(2,1), center_pos(1,1)], turningRadius, 'LineWidth', 1);
 
    hold on;
end
line(wpt_posy,wpt_posx)
plot(wpt_posy,wpt_posx,'o',y_p,x_p,y_s,x_s)
legend ('dubins path','position', 'hermite position', 'spine position');
xlabel('y- position (m)')
ylabel('x- position (m)')

%}




%% Lookahead-based steering with PI-controller simulation
int_on = 1; %integral action on
K_i_guidance = 1/300; %1/300*K_p_guidance


%%
%{
Kp_guidance = 10;
Ki_guidance = 0.1;
sim MSFartoystyring22

pathplotter(p(:,1),p(:,2),psi,tsamp,100,tstart,tstop,0,WP);
%}

%% Path folowing dependacies
global L_pp;    %Ship length
global WP;      %Waypoints
global k;       %Waypoint index
global R;       %Circle of acceptance radius

L_pp = 304.8;
load WP;
k = 1;
R = ones(size(WP, 2))*3*L_pp;



lambda_c = 0.1;

K_p_c = 3*lambda_c^2;
K_i_c = lambda_c^3;
K_d_c = 3*lambda_c;

omega_c = 1;
zeta_c = 1;




%% SIMULATION
sim MSFartoystyring22

%% Path Plot
dec = 20;
track = 0;
tstop = tsamp * (length(p)-1);

pathplotter(p(:,1), p(:,2), psi, tsamp, dec, tstart, tstop, track, WP);




