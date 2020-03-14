function dydt = Specs2DNC(t, Conditions, Force, Disturb, givens)
    % define givens
    m = givens(6);      % mass of the drone [kg]
    r = givens(7);       % body to motor distance [m]
    k = givens(8);     % [Nm/N]
    R = givens(9);  % [m]
    g = givens(10);% gravity [m/s^2]
    alpha = givens(1);   % [N/(m/s)^2]
    eta = givens(2);    % [N/(rad/s)^2]
    I_x = givens(3);   % moment of inertia in the x direction [kg m^2]
    I_y = givens(4);   % moment of inertia in the y direction[kg m^2]
    I_z = givens(5);  % moment of inertia in the z direction[kg m^2]
    
    % define conditions vector
    u = Conditions(1); % inertial velocity in the u direction in body coordinates [m/s]
    v = Conditions(2); % inertial velocity in the v direction in body coordinates [m/s]
    w = Conditions(3); % inertial velocity in the w direction in body coordinates [m/s]
    p = Conditions(4); % inertial angular velocity in the p direction in body coordinates [rad/s]
    q = Conditions(5); % inertial angular velocity in the q direction in body coordinates [rad/s]
    r = Conditions(6); % inertial angular velocity in the r direction in body coordinates [rad/s]
   phi = Conditions(7); % bank [rad]
    theta = Conditions(8); % elevation [rad]
    psi = Conditions(9); % azimuth [rad]
    x_E = Conditions(10); % position vector in the x direction in inertial coordinates [m]
    y_E = Conditions(11); % position vector in the y direction in inertial coordinates [m]
    z_E = Conditions(12); % position vector in the z direction in inertial coordinates [m]
    f1 = Force(1); % trim force exerted by motor 1 [N]
    f2 = Force(2); % trim force exerted by motor 1 [N]
    f3 = Force(3); % trim force exerted by motor 1 [N]
    f4 = Force(4); % trim force exerted by motor 1 [N]
    
    % Aerodynamic/Control Moments and Forces
    Laero = - alpha^2 * p^2 * sign(p); % p component of the aerodynamic moments
    Maero = - alpha^2 * q^2 * sign(q); % q component of the aerodynamic moments
    Naero = - alpha^2 * r^2 * sign(r); % w component of the aerodynamic moments
    Lcontrol = ((f1 + f2) - (f3 + f4)) * R; % p component of the control moments
    Mcontrol = ((f3 + f2) - (f1 + f4)) * R; % q component of the control moments
    Ncontrol = (f1 + f3 - (f4 + f2)) * k; % w component of the aerodynamic moments
    L = Laero + Lcontrol; % L moment sum (abt x axis)
    M = Maero + Mcontrol; % M moment sum (abt y axis)
    N = Naero + Ncontrol; % N moment sum (abt Z axis)
    
    if t > 2 && t < 2.5
        L = L + Disturb(2); % Disturbed L moment (abt x axis)
        M = M + Disturb(1); % Disturbed M moment (abt y axis)
        N = N + Disturb(3); % Disturbed N moment (abt z axis)
    end
    Xaero = - eta^2 * u^2 * sign(u); % x component of aerodynamic force
    Yaero = - eta^2 * v^2 * sign(v); % y component of aerodynamic force
    Zaero = - eta^2 * w^2 * sign(w); % z component of aerodynamic force
    Xcontrol = 0; % x control
    Ycontrol = 0; % y control
    Zcontrol = -sum(Force); % gravitational force counteraction
    X = Xaero + Xcontrol; % sum of x forces
    Y = Yaero + Ycontrol; % sum of y forces
    Z = Zaero + Zcontrol; % sum of z forces
    pdot = (I_y - I_z)/I_x * q * r - 1/I_x * L; % roll rate derrivative
    qdot = (I_z - I_x)/I_y * p * r - 1/I_y * M; % pitch rate derrivative
    rdot = (I_x - I_y)/I_z * p * q - 1/I_z * N; % yaw rate derrivative
  
    dOmega_bdt = [pdot, qdot, rdot]';
    udot = r*v - q*w - g*sin(theta) + 1/m * X; % 4.7,1 acceleration in the x axis
    vdot = p*w - r*u + g*sin(phi)*sin(theta) + 1/m * Y; % acceleration in the y axis
    wdot = q*u - p*v + g*cos(phi)*cos(theta) + 1/m * Z; % acceleration in the z axis
    dVbdt = [udot, vdot, wdot]';
    xdot =  u * cos(theta)*cos(psi) +v * (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) + w * (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
    ydot =  u * cos(theta)*sin(psi) + v * (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) + w * (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
    zdot = -u * sin(theta) + v * sin(phi)*cos(theta) + w * cos(phi)*cos(theta);
    dVEdt = [xdot, ydot, zdot]';
    phidot  = p + (q*sin(phi)+r*cos(phi))*tan(theta); % bank roc (pg 104)
    thetadot = q*cos(phi) - r*sin(phi); % elevation roc (pg 104)
    psidot   = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta); % azimuth roc (pg 104)
  
    dEuldt = [phidot, thetadot, psidot]';
    dydt = [dVbdt; dOmega_bdt; dEuldt; dVEdt];
    
    
    
end