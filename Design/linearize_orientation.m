function B = linearize_orientation(theta, phi)
% phidot = wx + sin(phi)*tan(theta)*wy + cos(phi)*tan(theta)*wz
% thetadot = cos(phi)*wy - sin(phi)*wz
% psidot = sin(phi)*sec(theta)*wy + cos(phi)*sec(theta)*wz
theta = theta*pi/180;
phi = phi*pi/180;
B = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);...
     0 cos(phi)           -sin(phi);...
     0 sin(phi)*sec(theta) cos(phi)*sec(theta)];

end