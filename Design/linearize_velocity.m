function B = linearize_velocity(theta, phi, g, m)
    theta = theta*pi/180;
    phi = phi*pi/180;
    B = [-g*cos(theta)           0                      0;...
         -g*sin(theta)*sin(phi)  g*cos(theta)*cos(phi)  0;...
         -g*sin(theta)*cos(phi) -g*sin(phi)*cos(theta) -1/m];
end