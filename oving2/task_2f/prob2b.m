clear all;
clc;
close all;
autopilot_constants;



s = tf('s');
%% Task 2b
H_evans_form = 1/(s*(s^2+(a_phi_1 + a_phi_2*k_d_phi)*s + a_phi_2*k_p_phi));
figure(7);
rlocus(H_evans_form); %This gives the limit for a_phi_2*k_i_phi


%% task 2d
H_phi_open = minreal(a_phi_2*(k_p_phi*s + k_i_phi) / (s^2*(s + a_phi_1 + a_phi_2*k_d_phi)));
H_phi_closed = H_phi_open/(1+H_phi_open);
H_chi_open = minreal(g/(V_g*s)*(H_phi_closed)*(k_i_chi/s+k_p_chi));

figure(1)
bode(H_phi_open); 
hold on; grid on;
bode(H_chi_open); legend('H_{\phi}','H_{\chi}');
hold off;

figure(2)
grid on;
margin(H_phi_open); 
legend('H_{\phi}');

figure(3)
grid on;
margin(H_chi_open); 
legend('H_{\chi}');

%setting the frequency of the course loop equal to the roll loop

W_chi = 1;
omega_n_chi = 1 / W_chi * omega_n_phi;

k_p_chi = 2 * zeta_chi * omega_n_chi * V_g / g;
k_i_chi = omega_n_chi^2 * V_g / g;

H_phi_open = minreal(a_phi_2*(k_p_phi*s + k_i_phi) / (s^2*(s + a_phi_1 + a_phi_2*k_d_phi)));
H_phi_closed = H_phi_open/(1+H_phi_open);
H_chi_open = minreal(g/(V_g*s)*(H_phi_closed)*(k_i_chi/s+k_p_chi));

figure(4)
bode(H_phi_open); 
hold on; grid on;
bode(H_chi_open); legend('H_{\phi}','H_{\chi}');
hold off;

figure(5)
grid on;
margin(H_phi_open); 
legend('H_{\phi}');

figure(6)
grid on;
margin(H_chi_open); 
legend('H_{\chi}');

