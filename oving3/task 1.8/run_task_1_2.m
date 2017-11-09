rad2deg = 180/pi;
deg2rad = pi/180;

%1.2 finding optimal Nomoto-parameters 

amp = -0.3;
omega_d = 0.008;
%desired delta_c
dc = 10*deg2rad;

sim MSFartoystyring12


% nonlinear least-squares parametrization: x(1)=1/T and x(2)=K
%x = firstordernomoto(tdata, rdata);
tdata = t;
rdata = r;


x0 = [0.01 0.01]';
F = @(x, t) exp(-tdata*x(1))*r0 +x(2)*(1-exp(-tdata*x(1)))*dc;
x = lsqcurvefit(F,x0, tdata, rdata);


%SAME MODEL WITH DIFFERENT SIMULATION TIME. The K and T values
%achieved with tstop=5000 are stated below. If you want a shorter plot,
%while using the same agreed upon model, uncomment these two lines:
%x(1) = 1/62.6809;%1/T
%x(2) = -0.0337;%K
%.


plot(tdata,rdata*rad2deg,'--', 'LineWidth', 2);
hold on;
grid on;
a = exp(-tdata*x(1))*r0 + x(2)*(1-exp(-tdata*x(1)))*dc*rad2deg;
plot(tdata, a,'LineWidth', 2);
%title('NLS fit of Mariner model for $\delta$ = 10 (deg)','Interpreter','latex', 'FontSize',16)
xlabel('time (s)')
ylabel('Yaw rate [deg/s]')
legend('Nonlinear ship system','Estimated 1st-order Nomoto model')
T = 1/x(1);
K = x(2);