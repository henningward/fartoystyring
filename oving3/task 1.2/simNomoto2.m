function [ r ] = simNomoto2(x, delta_c, tstop, tsamp )
tstart = 0;
options = simset('SrcWorkspace','current');
sim('nomotosecondorder',[],options)
end




%{
function [x] = firstordernomoto(tdata, rdata)

x0 = [0.01 0.1]'
F = inline('exp(-tdata*x(1))*0 +x(2)*(1-exp(-tdata*x(1)))*5','x','tdata')
x = lsqcurvefit(F,x0, tdata, rdata);

plot(tdata,rdata,'g',tdata,exp(-tdata*x(1))*0 +...
x(2)*(1-exp(-tdata*x(1)))*5-0.52*10^-3,'r'),grid
title('NLS fit of Mariner model for \delta = 5 (deg)')
xlabel('time (s)')
legend('Nonlinear model','Estimated 1st-order Nomoto model')

end

%}