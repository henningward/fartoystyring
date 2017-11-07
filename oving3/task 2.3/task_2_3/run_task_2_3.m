%% This script will run task 2_3
rad2deg = 180/pi;
deg2rad = pi/180;

%% Simulation parameters
tstart=0;                   % Sim start time
tstop=2500;                 % Sim stop time
tsamp=10;                   % Sampling time for how often states are stored. (NOT ODE solver time step)
                
p0 = [1500, 500];           % Initial position (NED)
v0 = [6.63 0]';             % Initial velocity (body)
psi0 = 150*deg2rad;         % Inital yaw angle
r0 = 0;                     % Inital yaw rate
c = 1;                      % Current on (1)/off (0)

%% Course controller
K = -0.0590;
T = 111.5778;

lambda_c = 0.1;

K_p_c = 3*lambda_c^2;
K_i_c = lambda_c^3;
K_d_c = 3*lambda_c;

omega_c = 1;
zeta_c = 1;

%% Speed controller
c_m = 5467;
c_u = -0.0018;
c_uu = 1.0378;

zeta = 1;
omega = 0.01;
lambda = 0.011;
k_p = 2*lambda;
k_i = lambda^2;

%% Generation of reference signals
speed_ref = zeros(tstop, 2);
speed_ref(:, 1) = 0:1:tstop-1;
speed_ref(:, 2) = v0(1);
speed_ref(1:end, 2) = 7;

heading_ref = zeros(tstop, 2);
heading_ref(:, 1) = 0:1:tstop-1;
heading_ref(:, 2) = 0*deg2rad;

%% Path folowing dependacies
global L_pp;    %Ship length
global WP;      %Waypoints
global k;       %Waypoint index
global R;       %Circle of acceptance radius

L_pp = 304.8;
load WP;
k = 1;
R = ones(size(WP, 2))*3*L_pp;

%% SIMULATION
sim model_task_2_3

%% Path Plot
dec = 20;
track = 0;
tstop = tsamp * (length(p)-1);

pathplotter(p(:,1), p(:,2), psi, tsamp, dec, tstart, tstop, track, WP);


