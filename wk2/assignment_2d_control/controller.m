function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% Robot Parameters:
% params.gravity - 9.8100
% params.mass - weight of quadrotor - 0.1800
% params.Ixx - Moment of inertia - 2.5e-04
% params.arm_length - length of quadrotor arm - 0.060
% params.minF - minimum thrust force - 0
% params.maxF - maximum thrust force - 3.5316

% State des_state
% pos - 2x1 [] double - [y_des, z_des]
% vel - 2x1 [] double - [y_d_des, z_d_des]
% acc - 2x1 [] double - [y_d_des, z_d_des]

% State state
% pos - 2x1 [] double - [y, z]
% rot - double - phi
% vel - 2x1 [] double - [y_d, z_d]
% omega - double - phi_d

y = state.pos(1);
y_des = des_state.pos(1);
y_d = state.vel(1);
y_d_des = des_state.vel(1);
y_dd_des = des_state.acc(1);

z = state.pos(2);
z_des = des_state.pos(2);
z_d = state.vel(2);
z_d_des = des_state.vel(2);
z_dd_des = des_state.acc(2);

phi = state.rot;
phi_d = state.omega;

% CONSTANT GAINS
% U1 GAINS
K_dy = 5; % derivative Y gain
K_py = 20; % proportional Y gain
% U2 GAINS
K_dphi = 50; % derivative Phi gain
K_pphi = 1000; % proportional Phi gain
% PHI GAINS
K_dz = 20; % derivative Z gain
K_pz = 80; % proportonal Z gain
% 
% K_dy = 5; % derivative Y gain Kv_y
% K_py = 40; % proportional Y gain Kp_y
% K_dphi = 50; % derivative Phi gain Kv_z
% K_pphi = 1000; % proportional Phi gain Kp_z
% K_dz = 20; % derivative Z gain Kv_z
% K_pz = 130; % proportonal Z gain Kp_z

%phi_c = -(1/g) (y_dd_des + K_dy (y_d_des - y_d) + K_py(y_des - y))
phi_c = (-1.0/params.gravity) * (y_dd_des + ( K_dy * (y_d_des - y_d))  + ( K_py * (y_des - y)) );
phi_d_c = 0;
phi_dd_c = 0;

% u2 = Ixx(phi_dd_c + K_pphi(phi_c - phi) + K_dphi(phi_d_c-phi_d))
u2 = params.Ixx * (phi_dd_c + ( K_pphi * (phi_c - phi)) + ( K_dphi * (phi_d_c - phi_d)) );

% u1 = m(g + z_dd_des + K_dz(z_d_des - z_d) + K_pz(z_des - z))
u1 = params.mass * (params.gravity + z_dd_des + (K_dz * (z_d_des - z_d)) + (K_pz * (z_des - z)));

end