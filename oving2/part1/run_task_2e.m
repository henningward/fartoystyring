clear all;
clc;
close all;
autopilot_constants;

d = 0;

s = tf('s');
H_phi_open = minreal(a_phi_2*(k_p_phi*s + k_i_phi) / (s^2*(s + a_phi_1 + a_phi_2*k_d_phi)));
H_phi_closed = H_phi_open/(1+H_phi_open);
H_chi_open = minreal(g/(V_g*s)*(H_phi_closed)*(k_i_chi/s+k_p_chi));
H_chi_closed = H_chi_open/(1+H_chi_open);
%{
S = 500;
t = 0:1:S-1;
reference = zeros(1, S);
step_val = 0;
k = 0;
for i = 1:S,
    if i == 100
        step_val = 2;
    elseif i == 200
        step_val = 5;
    elseif i == 300
        step_val = 10;
    elseif i == 400
        step_val = 20;
         
    end
    reference(i) = step_val;
end

figure(1)
lsim(H_chi_closed,reference',t)

%}


sim_time = 500;

figure(1)
load_system('autopilot.slx');
sim('autopilot.slx');
subplot(211);
plot(chi.time,chi.signals.values.*rad2deg); hold on; 
subplot(212);
plot(chi.time,chi.signals.values.*rad2deg); hold on; 
