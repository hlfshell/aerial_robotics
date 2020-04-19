syms phi theta psi real

R1 = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1;]
R2 = [cos(theta) 0 sin(theta); -sin(theta) 1 cos(theta); 0 0 0;]
R3 = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1;]

simplify(R1*R2*R3)