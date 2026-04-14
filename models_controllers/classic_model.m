%% Main function
function [discreteStateSpace, t, theta] = classic_model(torque)
    %% Parameter definitions
    
    % Struct p with parameters
    p.cartMass = 0.5; % unit: kg
    p.rodMass = 0.082; % unit: kg
    p.tipMass = 0.002; % unit: kg
    p.pendulumMass = 0.084; % unit: kg
    p.rodLength = 0.35; % unit: m
    p.gravity = -9.82; % unit: m/s^2
    p.rollerRadius = 0.05; % unit: m
    p.beltLength = 1.72; % unit: m
    p.LinearDampingCoefficient = 5; % unit: N/(m/s)
    p.RotationalDampingCoefficient = 0.0012; % unit: Nm/(rad/s)
    
    % Center of mass of the pendulum (unit: m)
    p.centerOfMass = (p.rodMass*(p.rodLength/2) + p.tipMass*p.rodLength) / p.pendulumMass;
    
    % Intertia of the pendulum about pivot (unit: kg*m^2)
    p.rodInertia = (1/3)*p.rodMass*p.rodLength^2; % Rod
    p.tipInertia = p.tipMass*p.rodLength^2; % Tip
    p.pendulumInertia = p.rodInertia + p.tipInertia % Total pendulum
    
    %% Classic nonlinear model with ODE
    
    % Initial conditions (theta = 0 means pendulum points upward)
    state_0 = [0; 0; deg2rad(5); 0]; % Initial states
    t_span = [0, 10]; % Timespan for simulation
    
    % ODE
    ode = @(t,state) nonLinearModel(t, state, torque, p);
    [t, state] = ode45(ode, t_span, state_0);
    theta = state(:,3); % Export theta for figure in mainscript
    
    %% Plotting the non linear simulation
    
    %{
    % New figure
    figure;
    
    % Cart position
    subplot(2,1,1)
    plot(t, state(:,1)); 
    ylabel('x [m]')
    xlabel('time [s]')
    title('Position of the cart')
    grid on;
    
    % Pendulum angle
    subplot(2,1,2)
    plot(t, rad2deg(state(:,3))); 
    ylabel('\theta [deg]')
    xlabel('time [s]')
    title('Angle of the pendulum')
    grid on;
    %}

    %% Linearization around upright equilibrium
    stateSpaceModel = linearModel(p);
    
    %% Discretize the state space model
    Ts = 0.01; % PLC sampling time
    discreteStateSpace = c2d(stateSpaceModel, Ts, 'zoh')
end

%% Non linear model for ode
function stateDerivatives = nonLinearModel(t, state, torque, p)
    
    % States
    x = state(1);
    d_x = state(2);
    theta = state(3);
    d_theta = state(4);

    % Input force 
    F_m = torque/p.rollerRadius;

    % Parameters with shorter variable names to simplify
    m_c  = p.cartMass;
    m_p = p.pendulumMass;
    l_com  = p.centerOfMass;
    g  = p.gravity;
    b_c = p.LinearDampingCoefficient;
    b_p = p.RotationalDampingCoefficient;
    I_p = p.pendulumInertia;
    
    % Euler-Lagrance equations
    % Matrix
    A = [m_c+m_p, m_p*l_com*cos(theta);
         m_p*l_com*cos(theta), I_p];
    % Right-hand side
    b = [F_m-b_c*d_x+m_p*l_com*d_theta^2*sin(theta);
         -b_p*d_theta-m_p*g*l_com*sin(theta)]; 

    % Solve for accelerations
    accelerations = A\b;
    dd_x = accelerations(1);
    dd_theta = accelerations(2);

    % Return derivatives of states
    stateDerivatives = [d_x; dd_x; d_theta; dd_theta];
end

%% Linearization around upright equilibrium
function stateSpaceModel = linearModel(p)
    % Symbolic variables: state and torque
    syms x d_x theta d_theta torque real;
    states = [x; d_x; theta; d_theta];

    % Input force 
    F_m = torque/p.rollerRadius;
    
    % Parameters with shorter variable names to simplify
    m_c  = p.cartMass;
    m_p = p.pendulumMass;
    l_com  = p.centerOfMass;
    g  = p.gravity;
    b_c = p.LinearDampingCoefficient;
    b_p = p.RotationalDampingCoefficient;
    I_p = p.pendulumInertia;

    % Euler-Lagrance equations
    % Matrix
    A = [m_c+m_p, m_p*l_com*cos(theta);
         m_p*l_com*cos(theta), I_p];
    % Right-hand side
    b = [F_m-b_c*d_x+m_p*l_com*d_theta^2*sin(theta);
         -b_p*d_theta-m_p*g*l_com*sin(theta)]; 

    % Solve for accelerations
    accelerations = A\b;
    dd_x = accelerations(1);
    dd_theta = accelerations(2);

    % Derivatives of states
    stateDerivatives = [d_x; dd_x; d_theta; dd_theta];

    % Jacobian matrices (partial derivatives) regarding states and torque
    statesJacobian = jacobian(stateDerivatives, states);
    torqueJacobian = jacobian(stateDerivatives, torque);

    % Jacobians around upright equilibrium
    linearStatesJacobian = double(subs(statesJacobian, {x, d_x, theta, d_theta, torque}, {0, 0, 0, 0, 0}));
    linearTorqueJacobian = double(subs(torqueJacobian, {x, d_x, theta, d_theta, torque}, {0, 0, 0, 0, 0}));

    % Making the full state space model
    stateSpaceModel = ss(linearStatesJacobian, linearTorqueJacobian, eye(4), zeros(4,1))
end