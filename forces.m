function [output] = forces(t,y,P,u,wind)
%     y: state vector
%     P: struct containing the parameters for the plane
%     u: elevator, aileron, throttle
%     wind: wind components in N E D, rates in pqr
%     pos     = eta(1:3);
    %pos = y(1:3);
    Theta = y(4:6);
    vel = y(7:9);
    rate = y(10:12);
    
    phi     = Theta(1);
    theta   = Theta(2);
    psi     = Theta(3);
    p       = rate(1) + wind(4);
    q       = rate(2) + wind(5);
    r       = rate(3) + wind(6);
    elevator= u(1);
    aileron = u(2);
    rudder  = u(3);
    throttle= u(4);
       
    %% relative velocity
    wind_b = wind(1:3);
    vel_r = vel-wind_b;                               
    u_r = vel_r(1);
    v_r = vel_r(2);
    w_r = vel_r(3);
    
    %% compute airspeed Va, angle-of-attack alpha, side-slip beta. See (2.8)
    % in B&M
    Va    = sqrt(u_r^2+v_r^2+w_r^2);
    if(Va == 0)
        Va = 0.00001;
    end

    alpha = atan2(w_r,u_r);
    beta  = asin(v_r/Va);
 
    %% compute gravitational force
    fg_N = [0;0;P.mass*P.gravity];                  %gravity in NED frame
    fg_b = Rzyx(phi,theta,psi)'*fg_N;                %body frame
    
    %% Longitudinal mode
    % compute aero lift force
    C_L_alpha = P.C_L_0 + P.C_L_alpha*alpha;
    f_lift_s = 0.5*P.rho*Va^2*P.S_wing*(C_L_alpha + P.C_L_q*P.c/(2*Va)*q + P.C_L_delta_e*elevator);     %eqn 4.6 in B&M
    
    % drag
    C_D_alpha = P.C_D_0 + P.C_D_alpha1*alpha + P.C_D_alpha2*alpha^2;
    C_D_beta = P.C_D_beta1*beta + P.C_D_beta2*beta^2;
    f_drag_s = 0.5*P.rho*Va^2*P.S_wing*(C_D_alpha + C_D_beta + P.C_D_q*P.c/(2*Va)*q + P.C_D_delta_e*elevator^2);
    
    % pitch moment
    m_a = P.C_m_0 + (P.C_m_alpha)*alpha;
    m = 0.5*P.rho*Va^2*P.S_wing*P.c*(m_a + (P.C_m_q)*P.c/(2*Va)*q + P.C_m_delta_e*elevator);
    
    %% Lateral mode
    f_y = 0.5*P.rho*Va^2*P.S_wing*    (P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*P.b/(2*Va)*p + P.C_Y_r*P.b/(2*Va)*r + P.C_Y_delta_a*aileron + P.C_Y_delta_r*rudder);
    l   = 0.5*P.rho*Va^2*P.b*P.S_wing*(P.C_l_0 + P.C_l_beta*beta + P.C_l_p*P.b/(2*Va)*p + P.C_l_r*P.b/(2*Va)*r + P.C_l_delta_a*aileron + P.C_l_delta_r*rudder);
    n   = 0.5*P.rho*Va^2*P.b*P.S_wing*(P.C_n_0 + P.C_n_beta*beta + P.C_n_p*P.b/(2*Va)*p + P.C_n_r*P.b/(2*Va)*r + P.C_n_delta_a*aileron + P.C_n_delta_r*rudder);
    
    %% Sum aero    %Convert from s to b frame
    F_aero = Rzyx(0,alpha,beta)'*[-f_drag_s; f_y; -f_lift_s];
    T_aero = [l; m; n];
          
    %% Prop force
    Vd = Va + throttle*(P.k_motor-Va); %Discharge velocity
    F_prop = [0.5*P.rho*P.S_prop*P.C_prop*Vd*(Vd-Va); 0; 0]; %B&M/ Calculation of thrust in a ducted fan assembly for hovercraft
    T_prop = [-P.k_T_P*(P.k_Omega*throttle)^2;0;0];
    
    %% Sum forces
    Force = F_prop + fg_b + F_aero;
    Torque = T_aero + T_prop;
    
    output = [Force; Torque];
end
