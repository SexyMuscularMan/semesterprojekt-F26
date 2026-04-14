%% Main function
function pole_placement_results = pole_placement(discreteStateSpace)
    %% Extracting variables from the state space model
    A = discreteStateSpace.A; % A matrix
    B = discreteStateSpace.B; % B matrix
    C = discreteStateSpace.C; % C matrix
    D = discreteStateSpace.D; % D matrix
    Ts = discreteStateSpace.Ts; % Sample time
    n = size(A, 1); % Number of states
    
    %% Checking controllability of the discrete state space
    controlMatrix = ctrb(A, B); % Controllability matrix (rank must be n for system to be controllable)
    if rank(controlMatrix) ~= n
        error("System is not controllable.");
    end

    %% Defining the poles and calculating the state feedback gain 
    poles = [0.95, 0.9, 0.85, 0.8];
    K = place(A, B, poles) % State feedback gain K 

    %% Simulation with difference equations
    % Setting up the simulation
    timeVector = 0:Ts:10; % Time vector for simulation
    steps = length(timeVector); % Number of time steps to be simulated 
    u = zeros(1, steps); % control input (torque)
    x = zeros(4, steps); % states (x, x_d, theta, theta_d)
    x(:,1) = [0; 0; deg2rad(5); 0]; % Initial states

    % Simulation loop
    for k = 1:steps-1
        % Adding noise
        noise_standart = [0.001; 0.001; 0.001; 0.001]; % units: [m; m; rad; rad]
        noise_gaussian = noise_standart .* randn(4,1); % Gaussian distribution of noise
        noisy_x = x(:,k) + noise_gaussian; % Adding noise to measured states
        % difference equations
        u(k) = -K * noisy_x; % Control equation
        x(:,k+1) = A * x(:,k) + B * u(k); % Step update
    end

    %{
    % Plotting response for x and theta
    figure;
    % Cart position x
    subplot(2,1,1);
    plot(timeVector, x(1,:));
    xlabel('time [s]')
    ylabel('x [m]');
    title('Position of the cart');
    grid on;
    % Pendulum angle theta
    subplot(2,1,2);
    plot(timeVector, rad2deg(x(3,:)));
    xlabel('time [s]')
    ylabel('\theta [deg]');
    title('Angle of the pendulum');
    grid on;
    %}

    %% Returning results for comparison with other controllers
    pole_placement_results.time = timeVector;
    pole_placement_results.x = x(1,:);
    pole_placement_results.theta = x(3,:);
end