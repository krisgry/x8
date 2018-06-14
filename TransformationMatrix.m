function T = TransformationMatrix(euler_ang)
    phi = euler_ang(1);
    theta = euler_ang(2);
    psi = euler_ang(3);
    T = [...
          1  sin(phi)*sin(theta)/cos(theta)  cos(phi)*sin(theta)/cos(theta);
          0  cos(phi)          -sin(phi);
          0  sin(phi)/cos(theta)      cos(phi)/cos(theta) ];
end