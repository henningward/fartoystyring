clear all;
clc;
close all;
constants_part2;


e = 1;
f = 0;

% exercise selector
feedback_select = e;

noise_amplifier = 1;

model_param_struct = struct('A_d', A_d, 'B_d', B_d, 'C_d', C_d, 'E_d', E_d, 'Q', Q, 'R', R, 'h', h);

load_system('autopilot');
sim('autopilot.slx');

figure(1)
plot(chi.time, chi.signals.values);
grid on;
xlabel('time');
ylabel('deg');
legend('\chi', '\chi_c');
if feedback_select == e
    title('course with noise-contaminated feedback');
else
    title('course with kalman-filtered feedback');
end

figure(2)
plot(aileron.time, aileron.signals.values);
grid on;
xlabel('time');
ylabel('deg');
legend('aileron');
if feedback_select == e
    title('aileron with noise-contaminated feedback');
else
    title('aileron with kalman-filtered feedback');
end

figure(3)
grid on;
plot(model_output_noise_contaminated.time, model_output_noise_contaminated.signals.values(:,3)); hold on;
plot(kalman_output.time, kalman_output.signals.values(:,2), '--');
plot(model_output.time, model_output.signals.values(:,3));
ylim([-30 25])
grid on;
xlabel('time');
ylabel('deg');
legend('noise contaminated signal', 'kalman output', 'true value');
if feedback_select == e
    title('roll angle with noise-contaminated feedback');
else
    title('roll angle with kalman-filtered feedback');
end
hold off;
axes('position',[.55 .15 .35 .3])
box on
hold on;
title('zoomed portion of figure');
indexOfInterest = (model_output_noise_contaminated.time < 100) & (model_output_noise_contaminated.time > 80); % range of t near perturbation
plot(model_output_noise_contaminated.time(indexOfInterest),model_output_noise_contaminated.signals.values(indexOfInterest,3)) % plot on new axes
plot(kalman_output.time(indexOfInterest),kalman_output.signals.values(indexOfInterest,2),  'linewidth', 2) % plot on new axes
plot(model_output.time(indexOfInterest),model_output.signals.values(indexOfInterest,3), 'linewidth', 2) % plot on new axes
ylim([-0.5 0.5])

figure(4)
plot(model_output_noise_contaminated.time, model_output_noise_contaminated.signals.values(:,2)); hold on;
plot(kalman_output.time, kalman_output.signals.values(:,3), '--');
plot(model_output.time, model_output.signals.values(:,2));
grid on;
xlabel('time');
ylabel('deg/s');
legend('noise contaminated signal', 'kalman output', 'true value');
if feedback_select == e
    title('roll rate with noise-contaminated feedback');
else
    title('roll rate with kalman-filtered feedback');
end
hold off;
axes('position',[.55 .15 .35 .3])
box on
hold on;
title('zoomed portion of figure');
indexOfInterest = (model_output_noise_contaminated.time < 100) & (model_output_noise_contaminated.time > 80); % range of t near perturbation
plot(model_output_noise_contaminated.time(indexOfInterest),model_output_noise_contaminated.signals.values(indexOfInterest,2)) % plot on new axes
plot(kalman_output.time(indexOfInterest),kalman_output.signals.values(indexOfInterest,3),  'linewidth', 2) % plot on new axes
plot(model_output.time(indexOfInterest),model_output.signals.values(indexOfInterest,2), 'linewidth', 2) % plot on new axes
ylim([-0.6 0.6])

figure(5)
grid on;
plot(model_output_noise_contaminated.time, model_output_noise_contaminated.signals.values(:,1)); hold on;
plot(kalman_output.time, kalman_output.signals.values(:,4), '--');
plot(model_output.time, model_output.signals.values(:,1));
ylim([-2 1.5])
xlabel('time');
ylabel('deg/s');
grid on;
legend('noise contaminated signal', 'kalman output', 'true value');
if feedback_select == e
    title('yaw rate with noise-contaminated feedback');
else
    title('yaw rate with kalman-filtered feedback');
end
hold off;

axes('position',[.55 .15 .35 .3])
box on
hold on;
title('zoomed portion of figure');
indexOfInterest = (model_output_noise_contaminated.time < 100) & (model_output_noise_contaminated.time > 80); % range of t near perturbation
plot(model_output_noise_contaminated.time(indexOfInterest),model_output_noise_contaminated.signals.values(indexOfInterest,1)) % plot on new axes
plot(kalman_output.time(indexOfInterest),kalman_output.signals.values(indexOfInterest,4),  'linewidth', 2) % plot on new axes
plot(model_output.time(indexOfInterest),model_output.signals.values(indexOfInterest,1), 'linewidth', 2) % plot on new axes
ylim([-0.02 0.03])









%{
if feedback_select == e
    chi_e = chi;
    aileron_e = aileron;
    model_output_e = model_output;
    kalman_output_e = kalman_output;
    model_output_noise_contaminated_e = model_output_noise_contaminated;
else
    chi_f = chi;
    aileron_f = aileron;
    model_output_f = model_output;
    kalman_output_f = kalman_output;
    model_output_noise_contaminated_f = model_output_noise_contaminated;
end
%}
