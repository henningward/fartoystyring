function [ guidance ] = headingGuidance( position )
%Function that calculates X_p and e for a guidance system for ship heading
%   Based on the theory from [1] chapter 10.3.2

global k WP_xpos WP_ypos R_k1;

N = length(WP_ypos); %number of way points

if k == 0 %not yet reached first way point
    xk = 0;
    yk = 0;
else
    xk = WP_xpos(k);
    yk = WP_ypos(k);
end

xk1 = WP_xpos(k+1);
yk1 = WP_ypos(k+1);

x_p = atan2(yk1 - yk,xk1 - xk); %(10.64)
e = - (position(1) - xk)*sin(x_p) + (position(2)- yk)*cos(x_p); %(10.59)
delta = sqrt(R_k1^2 - e^2);
if delta < 0.0000001
    delta = 0.000001; %making sure delta(t) > 0
end

guidance = [x_p; e; delta];
    
circle_of_acceptance = (xk1 - position(1))^2 + (yk1 - position(2))^2;
if circle_of_acceptance <= R_k1^2 %go to next waypoint (10.86)
    if k < N-1
        k = k+1;
    end
end
end


