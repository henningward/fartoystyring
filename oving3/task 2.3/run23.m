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
tstop=5000;        % Sim stop time
tsamp=10;           % Sampling time for how often states are stored. (NOT ODE solver time step)
                
p0=zeros(2,1);      % Initial position (NED)
v0=[6.63 0]';       % Initial velocity (body)
psi0=0;             % Inital yaw angle
r0=0;               % Inital yaw rate
c=0;                % Current on (1)/off (0)

%1.2 finding optimal Nomoto-parameters 


amp = -0.3;
omega_d = 0.008;
omega_0 = 10 * omega_d;

run_task_1_2

%computing reference model parameters
zeta = 1;
omega_n = 0.008;


%computing heading control parameters
omega_0 = 10 * omega_d;
w_psi = 0.05;
lambda_heading = 0.25;
Kp = 3*lambda_heading^2;
Kd = 3*lambda_heading;
Ki = lambda_heading^3;



%calculating d1 and d2 for speeed controller

%arbitrary values for nc to obtain two equation sets, needed to calculate
%d1 and d2
%1st equation set
nc = 5;
sim MSFartoystyring16
nc_1 = nc;
u_1 = v(end, 1);
%2nd equation set
nc = 7.3;
sim MSFartoystyring16
nc_2 = nc;
u_2 = v(end, 1);

%estimating d1 and d2
syms d1 d2;
Q1 = d1*u_1+d2*abs(u_1)*u_1 == abs(nc_1)*nc_1;
Q2 = d1*u_2+d2*abs(u_2)*u_2 == abs(nc_2)*nc_2;
sol = solve([Q1 Q2], [d1 d2], 'ReturnConditions', true);
d1 = double(sol.d1);
d2 = double(sol.d2);


%estimating mass by comparing forward_speed_model to MSFartoystyring
m = 5550;
v0=[0.01 0]';
sim Forward_speed_model
sim MSFartoystyring16


%simulating with speed controller
c=1;
v0=[4 0]';
lambda_speed = 0.3;
Kp_speed = 2*lambda_speed;
Ki_speed = lambda_speed^2;


%speed reference parameters
u_r_0 = 4;
u_r_final = 7;
u_r_step_time = 500;

sim MSFartoystyring18

figure()
plot(t,v(:,1), t, u_d, t, u_r);
legend({'u' ,'u_d', 'u_r'}, 'Interpreter','latex')
xlabel('time (s)')
ylabel('rudder input [deg]')
title('Closed loop behaviour of u, $u_d$ and $u_r$','Interpreter','latex','FontSize',16)

figure()
plot(t,u_tilde);
legend({'$u_{tilde}$'}, 'Interpreter','latex')
xlabel('time (s)')
ylabel('speed error [m/s]')
title('Closed loop behaviour of $u_{tilde}$','Interpreter','latex','FontSize',16)

