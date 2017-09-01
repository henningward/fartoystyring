%task 1b

%finding eigenvalues:

eta = 1;
m = 100;
r = 2;
kp = 1;
kd = 20;


A_BK = [zeros(3) 1/2*eta*eye(3);
    -1/(m*r^2)*kp*eye(3) -1/(m*r^2)*kd*eye(3)];

%In latex - write this as A matrix, B matrix and K matrix

eig(A_BK)
    

(kd/(m*r^2)+sqrt( ((kd/(m*r^2))^2 - 2*eta*kp/(m*r^2) ))) / 2