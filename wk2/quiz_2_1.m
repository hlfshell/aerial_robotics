% Which of the following matricies are rotation matricies?

disp("===================")
disp("QUESTION 1")
disp("===================")

% Check to see if the determinant is == 1
r1 = [ sqrt(2)/2 0 sqrt(2)/2; 0 1 0; -sqrt(2)/2 0 sqrt(2)/2]
det(r1)
r1 * transpose(r1)

r2 = [0.3835 0.5730 0.9287; 0.5710 0.5919 -0.4119; -1.3954 0.0217 1.1105]
det(r2)
r2 * transpose(r2)

syms theta
r3 = [cos(theta) -sin(theta); sin(theta) cos(theta)]
det(r3)
r3 * transpose(r3)
% Note that cos(theta)^2 + sin(theta)^2 is 1 so determinant passes

r4 = [0.2120 0.7743 0.5963; 0.2120 -0.6321 0.7454; 0.9540 -0.0316 -0.2981]
det(r4)
r4 * transpose(r4)

disp("===================")
disp("QUESTION 2")
disp("===================")

r = [0.6927 -0.7146 0.0978; 0.7165 0.6973 0.0198; -0.0824 0.0564 0.995]

% Euler angles:
% theta = +/-1 acos(R33)
% psi = atan2(R32/sin(theta), -R31/sin(theta))
% phi = atan2(R23/sin(theta), r13/sin(theta))

theta = acos(r(3,3))
psi = atan2(r(3,2)/sin(theta), -r(3,1)/sin(theta))
phi = atan2(r(2,3)/sin(theta), r(1,3)/sin(theta))


disp("===================")
disp("QUESTION 3")
disp("===================")

r = [0.675 -0.1724 0.7174; 0.2474 0.9689 0; -0.6951 0.1775 0.6967]

w_hat_b = [0 -1 0.9689; 1 0 -0.2474; -0.9689 .2474 0]

% whats w^s ?

r_t = transpose(r)

r_der = w_hat_b / r_t

w_hat_s = r_der * r_t

disp("===================")
disp("QUESTION 4")
disp("===================")

r = [0.2919 0.643 -0.7081; -0.643 -0.4161 -0.643; -0.7081 0.643 0.2919]
% angle is restricted to [0, pi]

%calculate the trace
tau = r(1,1) + r(2, 2) + r(3, 3)

% since this isnt 3 or -1, we're good to continue

phi = acos((tau - 1) /2)

u = (1/(2*sin(phi))) * (r - transpose(r))

u_hat = transpose([u(3, 2) u(1, 3) u(2, 1)])

% sqrt(2)/2 == 0.7071

disp("===================")
disp("QUESTION 5")
disp("===================")

r = [-1/3 2/3 -2/3; 2/3 -1/3 -2/3; -2/3 -2/3 -1/3]

tau = r(1,1) + r(2, 2) + r(3, 3)

% tau is -1, so we don't have enough information

disp("===================")
disp("QUESTION 6")
disp("===================")

disp("Nothing to do here")