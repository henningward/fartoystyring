%Initialize controllers

%Nomoto model
T = 100;
K = -0.0331;
%Heading
omega_n = 0.11;
K_p_heading = omega_n^2*T/K;
K_d_heading = (2*omega_n*T-1)/K;
K_i_heading = omega_n/10*K_p_heading;
delta_c_max = 25*pi/180; % Rudder saturation
%Heading reference model
zeta_heading = 1;
omega_heading = 0.05;
%Forward speed model
d_1 = -0.0634;
d_2 = 1.0463;
m_u = 6000;
%Speed
lambda = 0.23;
K_p_speed = 2*lambda;
K_i_speed = lambda^2;
n_c_max = 8.9; % Shaft speed saturation
%Speed reference model
zeta_speed = 1;
omega_speed = 0.5;
%Integral windup - anti-windup gains
K_windup_heading = 1;
K_windup_speed = 1;
