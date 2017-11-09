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

figure()
plot(t1,r_sin*rad2deg,'--','LineWidth', 2);
hold on;
plot(t1,nomoto_output*rad2deg, 'LineWidth', 2);

grid on;
legend('Nonlinear ship model', 'Estimated 1st-order Nomoto model')
%title('1st order Nomoto vs system with sine-wave input','Interpreter','latex', 'FontSize',17)
xlabel('time (s)')
ylabel('Yaw rate [deg/s]')



%% 1.4
c=1;                % Current on (1)/off (0)

%computing reference model parameters
zeta_c = 1;
omega_c = 0.02;

%computing heading control parameters
lambda = 0.25;
Kp_heading = 3*lambda^2;
Kd_heading = 3*lambda;
Ki_heading = lambda^3;

sim MSFartoystyring14

figure()
plot(t,psi_tilde*rad2deg, 'LineWidth', 2);
legend({'$\tilde\psi$'}, 'Interpreter','latex','FontSize', 13)
xlabel('time (s)')
ylabel('Heading error [deg]')
%title('Closed loop behaviour of $\tilde{\psi}$','Interpreter','latex', 'FontSize',16)

figure()
plot(t,psi*rad2deg, 'LineWidth', 2, 'Color', 'r');
hold on;
plot(t,psi_d*rad2deg, '--','LineWidth', 3, 'Color', 'b')
legend({'$\psi$', '$\psi_d$'}, 'Interpreter','latex','FontSize', 13)
xlabel('time (s)')
ylabel('Heading [deg]')
%title('Closed loop behaviour of $\psi$ and $\psi_d$','Interpreter','latex', 'FontSize',16)

figure()
plot(t,r_tilde*rad2deg, 'LineWidth', 2);
legend({'$\tilde{r}$'}, 'Interpreter','latex','FontSize', 13)
xlabel('time (s)')
ylabel('Heading error rate [deg/s]')
%title('Closed loop behaviour of $\tilde{r}$','Interpreter','latex','FontSize',16)

figure()
plot(t,r*rad2deg, 'LineWidth', 2, 'Color', 'r');
hold on;
plot(t,r_d*rad2deg, '--', 'LineWidth', 3, 'Color', 'b')
legend({'$r$', '$r_d$'}, 'Interpreter','latex','FontSize', 13)
xlabel('time (s)')
ylabel('Heading rate [deg/s]')
%title('Closed loop behaviour of $r$ and $r_d$','Interpreter','latex', 'FontSize',16)

figure()
satlim = 25*ones(1,length(t));
plot(t,rudder_input(1:501,1)*rad2deg, 'LineWidth', 2);
hold on;
plot(t, satlim, 'LineWidth', 3, 'Color', 'c')
legend({'$\delta_{c}$', 'Saturation at 25 degrees'}, 'Interpreter','latex','FontSize', 12) %kommenter det at dette skjer internt, alts� vil den fortsatt g� forbi eksternt
xlabel('time (s)')
ylabel('rudder input [deg]')
%title('Rudder input $\delta_c$','Interpreter','latex','FontSize',16)