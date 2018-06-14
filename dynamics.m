function xdot = dynamics(t,y,P,tau)
    % t time
    % y state vector
    % P object containing platform-specific parameters: mass, inertia
    % matrix I_cg, location of IMU wrt CG r_cg
    pos = y(1:3);
    Theta = y(4:6);
    vel = y(7:9);
    Omega = y(10:12);
    C_rb = [zeros(3),                                             -P.mass*Smtrx(vel)-P.mass*Smtrx(Omega)*Smtrx(P.r_cg);
            -P.mass*Smtrx(vel)+P.mass*Smtrx(P.r_cg)*Smtrx(Omega), -Smtrx(P.I_cg*Omega)]; %Fossen 3.56

    ny_dot = P.M_rb\(tau-C_rb*[vel;Omega]);
    vel_dot = ny_dot(1:3);
    Omega_dot = ny_dot(4:6);
    
    pos_dot = Rzyx(Theta(1),Theta(2),Theta(3))*vel;
    Theta_dot   = TransformationMatrix(Theta)*Omega;
    
    xdot = [pos_dot',Theta_dot',vel_dot',Omega_dot']';
end