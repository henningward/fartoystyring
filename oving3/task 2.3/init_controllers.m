rad2deg = 180/pi;
deg2rad = pi/180;

%maximum inputs
rudder_max = 25*deg2rad;
velocity_max = 85*2*pi/60;


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

%speed reference model parameters
omega_n = 0.008;
d1 = -0.0021;
d2 = 1.0377;
m = 5550;
lambda_speed = 0.3;
Kp_speed = 2*lambda_speed;
Ki_speed = lambda_speed^2;
u_r_0 = 100;
u_r_final = 100;
u_r_step_time = 2000;






