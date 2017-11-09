clear all
clc
close all


%%

rad2deg = 180/pi;
deg2rad = pi/180;

tstart=0;           % Sim start time
tstop=7000;        % Sim stop time
tsamp=10;           % Sampling time for how often states are stored. (NOT ODE solver time step)
                
p0=[1500 500];      % Initial position (NED)
v0=[6.63 0]';       % Initial velocity (body)
u_r_0 = v0(1);
psi0=150*deg2rad;             % Inital yaw angle
r0=0;               % Inital yaw rate
c=1;                % Current on (1)/off (0)

U_d = v0(1); %desired speed

init_controllers

%% task 2.1


%Loading waypoints
load('WP.mat')

wpt_posx = WP(1,:);
wpt_posy = WP(2,:);
wpt_time = [0 20 40 60 80 100]; 
t = 0:1:max(wpt_time);


%% simulation of target path
U_t = 3;
target_position = zeros(2, tstop);

target_start = WP(:,1);
target_WP = WP(:,2);
target_heading_deg =-(target_WP(2)-target_start(1))/(target_WP(1)-target_start(2));
target_heading_deg = target_heading_deg / norm(target_heading_deg);
target_heading_deg = atan2(WP(2,2)-WP(2,1),WP(1,2)-WP(1,1));
stepsize = U_t;
for i = 1:tstop
    target_position(1, i+1) = target_position(1, i) + stepsize * sin(target_heading_deg);
    target_position(2, i+1) = target_position(2, i) + stepsize * cos(target_heading_deg) * sign(target_heading_deg);
end
%line(wpt_posy, wpt_posx)
%line(target_position(2,:), target_position(1,:))

%% SIMULATION

sim MSFartoystyring27

%% Path Plot

dec = 20;
object_tracking = 1;
pathplotter(p(:,1), p(:,2), psi, tsamp, dec, tstart, tstop, object_tracking, WP);

figure()
plot(t, pos_error)
title('pos error');

figure()
plot(t, dist_error)
title('dist error');

figure()
plot(target_position_2(:,2), target_position_2(:,1)); hold on;
plot(-2500, -3500, 'r*');
title('target');


figure()
plot(t, v(:,1))

