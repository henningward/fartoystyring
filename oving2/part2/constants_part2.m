% constants
deg2rad = pi/180;   
rad2deg = 180/pi;
g = 9.81;


V_g = 637/3.6; %m/s

a_phi_1 = 2.87;
a_phi_2 = -0.65;

delta_a_max = 25*deg2rad;
e_phi_max = 15*deg2rad;
zeta_phi = 0.707;

omega_n_phi = sqrt(abs(a_phi_2)*delta_a_max /e_phi_max);

k_p_phi = delta_a_max / e_phi_max * sign(a_phi_2);
k_d_phi = (2*zeta_phi*omega_n_phi - a_phi_1) / a_phi_2;

k_i_phi = -.1;


W_chi = 8;
zeta_chi = 1;
omega_n_chi = 1 / W_chi*omega_n_phi;

k_p_chi = 2 * zeta_chi * omega_n_chi * V_g / g;
k_i_chi = omega_n_chi^2 * V_g / g;











%{
W_chi = 7;
zeta_chi = 1;
omega_n_chi = 1/W_chi*omega_n_phi;

k_p_chi = 2*zeta_chi*omega_n_chi*V_g/g;
k_i_chi = omega_n_chi^2*V_g/g;
%}
A = [ -0.322 0.052 0.028 -1.12 0.002;
     0 0 1              -0.001     0;
     -10.6 0 -2.87 0.46        -0.65;
     6.87 0 -0.04 -0.32        -0.02;
     0 0 0 0 -10];
B = [ 0 0 0 0 10]';

C = [0 0 0 1 0;
     0 0 1 0 0;
     1 0 0 0 0;
     0 1 0 0 0];
 
D = [0 0 0 0]';

E = eye(4);

h = 0.01;

Q = 10^-6* eye(4);  

var_phi = 2^2;
var_roll_rate = 0.5^2;
var_yaw_rate = 0.2^2;

R = diag([var_phi var_roll_rate var_yaw_rate]);

[A_d, B_d] = c2d(A(1:4, 1:4), A(1:4, 5), h);

[A_d, E_d] = c2d(A(1:4, 1:4), eye(4), h);

C_d = [0 0 0 1;
       0 0 1 0;
       0 1 0 0];



