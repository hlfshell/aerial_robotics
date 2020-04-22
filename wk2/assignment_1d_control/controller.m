function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% error would be z_desired - z_current, so
error = s_des(1) - s(1);
% error_dot would be the difference between v_z on desired - current
error_dot = s_des(2) - s(2);

% z_ddes is z double dot desired.
% It is 0, since a derivative of the constant
% is 0.
z_ddes = 0;

% Define our constants for kv and kp
% This will require the adjustment for tuning
k_v = 20;
k_p = 250;

u = params.mass * (z_ddes + (k_p * error) + (k_v * error_dot) + params.gravity );

% Clamp our responses to within u_max and u_min as provided
% AKA we can't apply negative thrust or more than our
% motors can provide
if(u < params.u_min)
    u = params.u_min;
else if(u > params.u_max)
    u = params.u_max;
end


end

