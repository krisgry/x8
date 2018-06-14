%% This is a simple simulation to show how the aerodynamic model of the Skywalker X8 can be used.
% The aerodynamic model is based on the paper
% K. Gryte, R. Hann, M. Alam, J. Rohác, T. A. Johansen, T. I. Fossen, Aerodynamic modeling of the Skywalker X8 Fixed-Wing Unmanned Aerial Vehicle, International Conference on Unmanned Aircraft Systems, Dallas, 2018

% Please note that only the aerodynamic model comes from the paper. 
% The thrust model is adapted from the Aerosonde model found in 
% Beard & McClain. Small Unmanned Aircraft
% The inertia is based on very rough initial tests, that we plan to redo in
% the future

function simX8
  clear all
  clear controller

  P = load('x8_param.mat');
  %Inertia matrix: assuming symmetry wrt xz-plane -> Jxy=Jyz=0
  P.I_cg =  [P.Jx,  0,  -P.Jxz;
             0      P.Jy,    0;
             -P.Jxz,0,    P.Jz];
  % mass matrix
  P.M_rb = [eye(3)*P.mass,     -P.mass*Smtrx(P.r_cg);
                  P.mass*Smtrx(P.r_cg),      P.I_cg       ];
  P.rho = 1.2250;
  P.gravity = 9.81;

  settings = odeset('OutputFcn',[]);

  tend = 400;
  tspan = [0 tend];
  y0 = [0;0;-200; %pos
        0;0;0; %euler ang
        18;0;0; %vel
        0;0;0]; %rates      

  wind = [0;0;0;0;0;0]; %wind velocity components in body. Could also be a function

  do_trim = false;

  if do_trim
      [y_trim,u_trim] = findTrim(y0,P);
  else 
     u_trim = [0.0370,0.0000,0,0.1219]';
     y_trim = [0.0000,-0.0000,-200.0000,0.0000,0.0308,0.0000,17.9914,0.0000,0.5551,0.0000,0.0000,0.0000]';
  end

  P.kp_h     = -0.025;
  P.ki_h     = 0;

  P.kp_theta = 0.1;
  P.kd_theta = -0.01;
  P.ki_theta = 0;

  P.kp_V     = -0.05;
  P.ki_V     = -0.01;

  P.kp_phi   = -0.5;
  P.ki_phi   = 0;
  P.kd_phi   = 0;

  P.kp_chi = -0.05;
  P.ki_chi = 0.0;

  [t,y] = ode45(@(t,y) dynamics(t,y,P,forces(t,y,P,controller(t,y,P,u_trim,ref(t,y)),wind)), tspan, y_trim, settings);
  figure(321);
  subplot(411)
  plot(t,y(:,3));
  legend('D');
  subplot(412)
  plot(t,rad2deg(y(:,4:6)));
  legend('\phi','\theta','\psi');
  subplot(413)
  plot(t,y(:,7:9));
  legend('u','v','w');
  subplot(414)
  plot(t,rad2deg(y(:,10:12)));
  legend('p','q','r');
end

function ref = ref(t,y)
   V = 18;
   if t < 20
    chi = 0;
   elseif t < 35
    chi = (t - 20)*1*pi/180;
   else
       chi = 15*pi/180;
   end
   climb_rate = 0.2;
   if t<75
     h = 200;
   elseif t < 325
     h = 200 + (t - 75)*climb_rate;
   else
     h = 250;
   end
  ref = [V chi h];
end

function u = controller(t,y,P,u_trim, ref)
   persistent i_phi i_h i_V i_theta i_chi
   if isempty(i_phi)
      disp('init')
      i_phi = 0;
      i_h = 0;
      i_V = 0;
      i_theta = 0;
      i_chi = 0;
      t_prev = 0;
      u_prev = u_trim;
      dt_max = -inf;
      dt_min = inf;
   end
   pos= y(1:3);
   Theta= y(4:6);
   vel= y(7:9);
   Omega= y(10:12);

   V_ref = ref(1);
   chi_ref = ref(2);
   h_ref = ref(3);
   
   v_n = Rzyx(Theta(1),Theta(2),Theta(3))*vel;
   chi = atan2(v_n(2),v_n(1));

   i_chi = i_chi + (chi - chi_ref);
   phi_ref = P.kp_chi*(chi - chi_ref) + P.ki_chi*i_chi;
   
   
   i_h = i_h + (-pos(3) - h_ref);
   theta_ref = P.kp_h*(-pos(3) - h_ref) + P.ki_h*i_h;

   i_theta = i_theta + (Theta(2) - theta_ref);
   delta_e = P.kp_theta*(Theta(2) - theta_ref) + P.ki_theta*i_theta - P.kd_theta*Omega(2);
   i_phi = i_phi + (Theta(1) - phi_ref);
   delta_a = P.kp_phi*(Theta(1) - phi_ref) + P.ki_phi*i_phi - P.kd_phi*Omega(1);

   delta_r = 0;

   i_V = i_V + (norm(vel) - V_ref);
   delta_t = P.kp_V*(norm(vel) - V_ref) + P.ki_V*i_V;

   u = [delta_e delta_a delta_r delta_t]' + u_trim;
   
   u = min(ones(4,1),max(-1*ones(4,1),u)); % saturate to -1..1
   u(4) = max(0,u(4)); %saturate thr to 0..1
end

function [xtrim, utrim] = findTrim(x0,P)
   assignin('base','x0',x0); %make initial condition available for simulink
   assignin('base','P',P); %make plane config available for simulink
    [sizes,x0,names]=trim_helper([],[],[],0);

    %% Trim Model
    % Set initial conditions for states, derivatives, inputs and outputs
    % indicate which states/derivatives/inputs/outputs should be fixed 
    % to the initial conditions
    %                  e a r t
    u0              = [0 0 0 0.5]'; %initial guess
    fixed_inputs    = [];

    gamma = 0; %path angle
    theta0=gamma;psi0=0;phi0=0
    Va = 18;
    R = Inf; %turning radius
    %                   N E    D           roll     pitch     yaw    u   v  w   p   q   r  
    %                   1 2    3            4       5         6      7   8  9  10  11  11 
    x0              =  [0 0  x0(3)         phi0     theta0    psi0   Va  0  0   0   0   0  ]';
    dx0             =  [0 0 -Va*sin(gamma) 0        0           0    0   0  0   0   0   0  ]';
    if R~=Inf,dx0(6)= Va/R; end %psidot = Va/R
    fixed_states    = [];%14:17 18];
    fixed_deriv     = [3:12];

    %                  Va alpha beta
    y0              = [Va];% gamma 0];%   x0(1:13)' phi0,theta0,psi0]';
    fixed_outputs   = [1];% 3];%, 4:19];

    % Calculate trim conditions
    options(14) = 1e6;
    options(2) = 1e-10;
    options(3) = 1e-10;
    options(4) = 1e-10;
    [xtrim,utrim,Y_trim,DX,options]=trim('trim_helper',x0,u0,y0, ...
                                   fixed_states,fixed_inputs,fixed_outputs, ...
                                   dx0,fixed_deriv,options);  %#ok<NOPTS>

    % Find linearized model around trim conditions
    [A,B,C,D]=linmod('trim_helper',xtrim,utrim);
end

