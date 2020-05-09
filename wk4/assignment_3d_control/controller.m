function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% params.mass
% params.l
% params.invl
% params.gravity
% params.arm_length
% parmas.minF
% params.maxF

% =================== Your code goes here ===================

% Constants
K_vx = 10;
K_px = 200;

K_vy = 10;
K_py = 200;

K_vz = 25;
K_pz = 200;

K_pphi = 2800;
K_vphi = 50;
K_ptheta = 2800;
K_vtheta = 50;
K_ppsi = 2800;
K_vpsi = 50;

% Acceleratios
r1_dd_des = des_state.acc(1) + K_vx * (des_state.vel(1) - state.vel(1)) + K_px * (des_state.pos(1) - state.pos(1));
r2_dd_des = des_state.acc(2) + K_vy * (des_state.vel(2) - state.vel(2)) + K_py * (des_state.pos(2) - state.pos(2));
r3_dd_des = des_state.acc(3) + K_vz * (des_state.vel(3) - state.vel(3)) + K_pz * (des_state.pos(3) - state.pos(3));

% Rotations
phi_des = (1/params.gravity) * ( (r1_dd_des * sin(des_state.yaw)) - (r2_dd_des * cos(des_state.yaw)) );
theta_des = (1/params.gravity) * ( (r1_dd_des * cos(des_state.yaw)) + (r2_dd_des * sin(des_state.yaw)) );;
psi_des = des_state.yaw;

% Thrust vectors
u1 = params.mass * ( params.gravity + r3_dd_des );

u2 = [
        K_pphi * (phi_des - state.rot(1)) + K_vphi * (-state.omega(1));
        K_ptheta * (theta_des - state.rot(2)) + K_vtheta * (-state.omega(2));
        K_ppsi * (psi_des - state.rot(3)) + K_vpsi * (des_state.yawdot - state.omega(3));
     ];

% Thrust
F = u1;

% Moment
M = params.I * u2;

% =================== Your code ends here ===================


end
