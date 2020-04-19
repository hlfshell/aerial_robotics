u = [sqrt(3)/3 sqrt(3)/3 sqrt(3)/3]
u = transpose(u);
ut= transpose(u);

u_hat = [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0;];

phi = 2*pi/3;

I = [1 0 0; 0 1 0; 0 0 1;];



vrot = I * cos(phi) + (u * ut) *(1- cos(phi)) + u_hat * sin(phi)