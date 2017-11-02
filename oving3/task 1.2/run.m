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

run_task_1_2

sim nomotofirstorder
sim MSFartoystyring12_sin_input
%{
figure()
plot(t1,nomoto_output), hold on;
plot(t1,r_sin);
legend('nomoto model','ship model')
%}



%% 1.4
c=1;                % Current on (1)/off (0)


Kp = -50;
Ki = -0.7;
Kd = -350;
sim MSFartoystyring14

figure()
plot(t,psi_tilde), hold on;
legend('psi tilde')

figure()
plot(t,r_tilde);
legend('r tilde')



