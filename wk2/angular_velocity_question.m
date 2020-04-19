%Following the same procedure, what is the angular velocity
% of a rotation about the x axis?

%Note that R(x,θ)= [1 0 0; 0 cos(θ) -sin(θ); 0 sin(θ) cos(θ)]

syms theta theta_der;
R = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)]
R_t = transpose(R)
R_der = diff(R)

R_der * R_t

% Note that cos(theta)^2 + sin(theta)^2 equals 1 when viewing this answer
% Thus we have an answer of [0 0 0; 0 0 -1; 0 1 0]