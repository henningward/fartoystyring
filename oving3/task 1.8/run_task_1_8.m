clear all
clc
close all


%%

rad2deg = 180/pi;
deg2rad = pi/180;

tstart=0;           % Sim start time
tstop=5000;         % Sim stop time
tsamp=10;           % Sampling time for how often states are stored. (NOT ODE solver time step)
                
p0=zeros(2,1);      % Initial position (NED)
v0=[6.63 0]';       % Initial velocity (body)
psi0=0;             % Inital yaw angle
r0=0;               % Inital yaw rate
c=0;                % Current on (1)/off (0)

%nomoto parameters
T = 62.6809;
K = -0.0337;

%heading model parameters
zeta_c = 1;
omega_c = 0.02;

w_psi = 0.05;
lambda_heading = 0.25;
Kp_heading = 3*lambda_heading^2;
Kd_heading = 3*lambda_heading;
Ki_heading = lambda_heading^3;


%% Task 1.8

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
omega_n = 0.008;



%speed reference parameters
u_r_0 = 4;
u_r_final = 7;
u_r_step_time = 500;

sim MSFartoystyring18


figure()
plot(t, u_r, 'LineWidth', 2, 'Color', 'g');
hold on;
plot(t, u_d,'--','LineWidth', 3, 'Color', 'b');
hold on;
plot(t,v(:,1),'LineWidth', 1, 'Color', 'r');
legend({'$u_r$' ,'$u_d$', 'u'}, 'Interpreter','latex', 'FontSize', 10)
xlabel('time (s)')
ylabel('speed [m/s]')
%title('Closed loop behaviour of u, $u_d$ and $u_r$','Interpreter','latex','FontSize',16)


figure()
plot(t,u_tilde, 'LineWidth', 3);
legend({'$\tilde{u}$'}, 'Interpreter','latex', 'FontSize', 13)
xlabel('time (s)')
ylabel('speed error [m/s]')
%title('Closed loop behaviour of $u_{tilde}$','Interpreter','latex','FontSize',16)


figure()
plot(t, psi*rad2deg, 'LineWidth', 2);
legend({'$\psi$' }, 'Interpreter','latex', 'FontSize', 13)
xlabel('time (s)')
ylabel('yaw angle [deg]')
%title('','Interpreter','latex','FontSize',16)

figure()
plot(t, r*rad2deg,'LineWidth', 2);
legend({'r'}, 'Interpreter','latex', 'FontSize', 15)
xlabel('time (s)')
ylabel('yaw rate [deg/s]')
%title('','Interpreter','latex','FontSize',16)


figure()
satlim = ((85*2*pi)/60)*rad2deg*ones(1,length(t));
plot(t, satlim, 'LineWidth', 3, 'Color', 'c')
hold on;
plot(t, shaft*rad2deg,'LineWidth', 2);
legend({'Saturation limit','Shaft $n_{c}$'}, 'Interpreter','latex', 'FontSize', 11)
xlabel('time (s)')
ylabel('shaft velocity [degrees/s]')
%title('','Interpreter','latex','FontSize',16)

figure()
satlim = 25*ones(1,length(t));
plot(t, satlim, 'LineWidth', 3, 'Color', 'c')
hold on;
plot(t, rudder_input(1:501,1)*rad2deg,'LineWidth', 2, 'Color', 'b');
legend({'Saturation limit','Rudder input $d_{c}$'}, 'Interpreter','latex', 'FontSize', 11)
xlabel('time (s)')
ylabel('rudder input [degrees]')
%title('','Interpreter','latex','FontSize',16)