function [ output_vector ] = guidance_control( p )
    %% Variables
    global k;   %Waypoint
    global WP;  %Waypoint vector
    global L_pp;%Ship length
    global R    %Circle of acceptace radius


    %% Look ahead based steering
    waypoint_vector = WP(:, k+1)-WP(:, k);
    alpha_k = atan2(waypoint_vector(2), waypoint_vector(1));
    
    R_p = @(theta) [cos(theta), -sin(theta); sin(theta), cos(theta)]; %Rotation matrix
    
    epsilon = R_p(alpha_k)'*(p - WP(:, k));
    [s, e] = deal(epsilon(1), epsilon(2)); 

    %delta = sqrt(max((3*L_pp)^2 - e^2, 0)); %Enclosure based approach
    delta = 2*L_pp;
    chi_r = atan(-e/delta);
    
    chi_d = alpha_k + chi_r;
    
    %normalization
    chi_d = mod(chi_d, 2*pi);
    
    %% Circle of acceptance(Waypoint switching)
    % if(WP(1, k+1) - p(1))^2 + (WP(2, k+1) - p(2))^2 <= R(k+1)^2
    %     k = min(k + 1, size(WP, 2)-1);
    % end
    
    if norm(WP(:, k +1)-WP(:, k)) - s <= R(k+1)
       k = min(k + 1, size(WP, 2)-1); 
    end
    
    output_vector = [chi_d e s];
    
end

