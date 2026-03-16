% Define terrain function (example: sinusoidal terrain)
terrain_func = @(x, y) 0.1 * sin(0.5 * x) + 0.05 * cos(0.3 * y);
% Define parameters
params.R0 = 0.3;       % Unloaded radius
params.K_t = 50000;    % Vertical stiffness
params.C_t = 1000;     % Vertical damping coefficient
params.sigma_0 = 1000; % LuGre stiffness
params.sigma_1 = 10;   % LuGre damping
params.sigma_2 = 0.5;  % Viscous friction
params.F_c = 500;      % Coulomb friction
params.F_s = 600;      % Static friction
params.v_s = 1;        % Characteristic velocity
params.terrain_func = terrain_func;
% Initial values
V = [10; 2; -1];       % Wheel velocity
omega = [30; 0; 0];    % Angular velocity
P_wheel = [0; 0; 0.2]; % Wheel center position
V_ground = [0; 0; 0];  % Static terrain
z = [0; 0];
dt = 0.01;
% Compute forces
[F_tire, z_next, N_next] = lugre_tire_model_dynamic(V, omega, P_wheel, V_ground, z, dt, params);
% Display results
disp('Tire Forces (N):');
disp(F_tire');
disp('Updated Normal Force:');
disp(N_next);