function out = kalmanfilter(y, u, model_param_struct)

persistent init_flag x_k P_k;

h   =   model_param_struct.h;

A_d =   model_param_struct.A_d;
B_d =   model_param_struct.B_d;
E_d =   model_param_struct.E_d;
H   =   model_param_struct.C_d;
R   =   model_param_struct.R;
Q   =   model_param_struct.Q;



if isempty(init_flag)
    %initialize enter prior estimate,and error cov;
    init_flag=1;
    x_k = [0;
           0;
           0;
           0];
       
    P_k = eye(4);
    
end

%calculate Kalman gain K_k;
K_k = P_k*H'/(H*P_k*H'+R); 

% update estimate with measured z_k
x_k_ = x_k+K_k*(y-H*x_k);

%compute error covarians
P_k = (eye(5) - K_k*H)*P_k*(eye(5)-K_k*H)'+K_k*R*K_k';

%project ahead
x_k = A_d*x_k_+B_d*u; 
P_k = A_d*P_k*A_d'+E_d*Q*E_d';

out = x_k_;

