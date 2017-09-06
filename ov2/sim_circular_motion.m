clear all;
clc;

%constants
deg2rad = pi/180;   
rad2deg = 180/pi;


%model parameters
phi = 0*deg2rad;
theta = 2*deg2rad;
psi = 30*deg2rad;
U = 1.5;
U_c = 0.6;
alpha_c = 10*deg2rad;
beta_c = 45*deg2rad;



%simulation parameters
N = 1000;
h = 0.1;

w = 360*U/(2*pi*100);
table = zeros(N+1,10);


pos = [0 0 0]';
u_c = U_c*cos(alpha_c)*cos(beta_c);
v_c = sin(beta_c)*U_c;
w_c = U*sin(alpha_c)*cos(beta_c);
U_c_vect = [u_c v_c w_c]';

for i = 1:N+1,
    t = (i-1)*h;
    U_vect = [U*cos(w*t) U*sin(w*t) 0]';
    U_r_vect = U_vect + U_c_vect;
    
    pos = pos+h*U_r_vect;
    
    table(i,:) = [t U_vect' pos' U_r_vect'];

end


%{
for i = 1:N+1,
    t = (i-1)*h;
    %U_vect = [U*cos(w*t) U*sin(w*t) 0]' + [U_c*cos(a_c) U_c*sin(b_c) U_c*sin(a_c)*cos(b_c]';



end

%}

U         = table(:,2:4);
t         = table(:,1);
pos       = table(:,5:7);
U_r       = table(:,8:10);

figure(1)
plot(pos(:,2), pos(:,1)),xlabel('East'),ylabel('North'),title('position'),grid
