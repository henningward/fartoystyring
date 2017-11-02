
rad2deg = 180/pi;
deg2rad = pi/180;


%desired delta_c
dc = 10*deg2rad;


sim MSFartoystyring12


% nonlinear least-squares parametrization: x(1)=1/T and x(2)=K
%x = firstordernomoto(tdata, rdata);
tdata = t;
rdata = r;


x0 = [0.01 0.1]'
F = @(x, t) exp(-tdata*x(1))*r0 +x(2)*(1-exp(-tdata*x(1)))*dc
x = lsqcurvefit(F,x0, tdata, rdata);
plot(tdata,rdata,'g',tdata,exp(-tdata*x(1))*0 +...
x(2)*(1-exp(-tdata*x(1)))*dc-0.1*10^-3,'r'),grid
title('NLS fit of Mariner model for \delta = 5 (deg)')
xlabel('time (s)')
legend('Nonlinear model','Estimated 1st-order Nomoto model')