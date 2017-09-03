% M-script for numerical integration of the attitude dynamics of a rigid 
% body represented by unit quaternions. The MSS m-files must be on your
% Matlab path in order to run the script.
%
% System:                      .
%                              q = T(q)w
%                              .
%                            I w - S(Iw)w = tau
% Control law:
%                            tau = constant
% 
% Definitions:             
%                            I = inertia matrix (3x3)
%                            S(w) = skew-symmetric matrix (3x3)
%                            T(q) = transformation matrix (4x3)
%                            tau = control input (3x1)
%                            w = angular velocity vector (3x1)
%                            q = unit quaternion vector (4x1)
%
% Author:                   2016-05-30 Thor I. Fossen 

clear all;
clc;

%% USER INPUTS
h = 0.1;                     % sample time (s)
N  = 2000;                    % number of samples

% model parameters
m = 100;
r = 2.0;
I = m*r^2*eye(3);       % inertia matrix
I_inv = inv(I);
kd = 300;
kp = 10;
Kd = kd * eye(3);

% constants
deg2rad = pi/180;   
rad2deg = 180/pi;

phi     = 10*deg2rad;            % initial Euler angles
theta   = -5*deg2rad;
psi     = 15*deg2rad;

q       = euler2q(phi,theta,psi)   % transform initial Euler angles to q


w = [0 0 0]';                 % initial angular rates
table = zeros(N+1,20);        % memory allocation

%% FOR-END LOOP
for i = 1:N+1,
   t = (i-1)*h;                  % time
   
   phi_d =10*sin(0.1*t)*deg2rad;
   theta_d = 0*deg2rad;
   psi_d = 15*cos(0.05*t)*deg2rad;
   q_d = euler2q(phi_d,theta_d,psi_d);  
   
   q_tilde = quatmultiply(quatconj(q_d'), q')';
   
   tau = -Kd * w - kp* q_tilde(2:4);   % control law 
   
   [phi,theta,psi] = q2euler(q); % transform q to Euler angles
   
   [phi_tilde,theta_tilde,psi_tilde] = q2euler(q_tilde); % transform q_tilde to Euler angles
   [J,J1,J2] = quatern(q);       % kinematic transformation matrices
   
   q_dot = J2*w;                        % quaternion kinematics
   w_dot = I_inv*(Smtrx(I*w)*w + tau);  % rigid-body kinetics

   table(i,:) = [t q' phi theta psi w' tau' phi_tilde theta_tilde psi_tilde phi_d theta_d psi_d];  % store data in table
   
   q    = q + h*q_dot;	             % Euler integration
   w    = w + h*w_dot;
   
   q    = q/norm(q);               % unit quaternion normalization
   

end 

%% PLOT FIGURES
t           = table(:,1);  
q           = table(:,2:5); 
phi         = rad2deg*table(:,6);
theta       = rad2deg*table(:,7);
psi         = rad2deg*table(:,8);
w           = rad2deg*table(:,9:11);  
tau         = table(:,12:14);
Theta_tilde = rad2deg*table(:,15:17);
desired_ang = rad2deg*table(:,18:20);
actual_ang =  rad2deg*table(:,6:8);

figure(1)
plot(t,w),xlabel('time (s)'),ylabel('deg/s'),title('w'),grid
wLeg = legend('$$\dot{\phi}$$', '$$\dot{\psi}$$', '$$\dot{\theta}$$');
set(wLeg, 'Interpreter', 'Latex');
figure(2)
plot(t,Theta_tilde),xlabel('time (s)'),ylabel('deg'),grid
Ttit = title('$\tilde{\Theta}$', 'Interpreter', 'Latex');
TLeg = legend('$$\tilde{\phi}$$ tracking error', '$$\tilde{\psi}$$ tracking error', '$$\tilde{\theta}$$ tracking error');
set(TLeg, 'Interpreter', 'Latex');
figure(3)
plot(t,tau),xlabel('time (s)'),ylabel('Nm'),title('\tau'),grid

figure(4)
plot(t,desired_ang),xlabel('time (s)'),ylabel('deg'),grid
hold on;
plot(t,actual_ang, '--'),xlabel('time (s)'),ylabel('deg'),grid, 
Ttit = title('$\tilde{\Theta}$', 'Interpreter', 'Latex');
TLeg = legend('$$\tilde{\phi}$$', '$$\tilde{\psi}$$', '$$\tilde{\theta}$$', '$$\tilde{\phi}$$ tracking reference', '$$\tilde{\psi}$$ tracking reference', '$$\tilde{\theta}$$ tracking reference');
set(TLeg, 'Interpreter', 'Latex');
hold off;





