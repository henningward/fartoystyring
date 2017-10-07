clear all;
clc;
close all;
autopilot_constants;

d = 15*deg2rad;
sim_time = 500;

figure(1)
load_system('autopilot_full.slx');
sim('autopilot_full.slx');
subplot(211);
plot(chi.time,chi.signals.values.*rad2deg); hold on; 
plot(ref.time,ref.signals.values.*rad2deg); hold on; 
legend('\chi', 'reference');
subplot(212);
plot(delta_a_c.time,delta_a_c.signals.values.*rad2deg); hold on; 
legend('\delta_a');
