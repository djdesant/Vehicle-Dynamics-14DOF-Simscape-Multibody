function [F_tire, z_next, N_next] = lugre_tire_model_dynamic(V, omega, P_wheel, V_ground, z, dt, params)
% Inputs:
%   V        - [3x1] Wheel center velocity [Vx; Vy; Vz] in world frame
%   omega    - [3x1] Angular velocity [wx; wy; wz] in world frame
%   P_wheel  - [3x1] Wheel center position [x; y; z] in world frame
%   V_ground - [3x1] Terrain velocity at contact point
%   z        - [2x1] LuGre internal state [z_x; z_y]
%   dt       - Time step
%   params   - Struct containing LuGre parameters
%
% Outputs:
%   F_tire - [3x1] Tire force vector [Fx; Fy; Fz]
%   z_next - [2x1] Updated internal state
%   N_next - Computed normal force
% Extract parameters
R0 = params.R0;              % Unloaded tire radius
K_t = params.K_t;            % Tire vertical stiffness
C_t = params.C_t;            % Tire vertical damping
sigma_0 = params.sigma_0;    % LuGre stiffness coefficient
sigma_1 = params.sigma_1;    % LuGre damping coefficient
sigma_2 = params.sigma_2;    % Viscous friction coefficient
F_c = params.F_c;            % Coulomb friction force
F_s = params.F_s;            % Static friction force
v_s = params.v_s;            % Characteristic velocity
terrain_func = params.terrain_func;
% Compute terrain normal and contact point
[P_c, N_t] = get_contact_point(P_wheel, R0, terrain_func);
% Compute relative vertical velocity
V_rel_z = V(3) - V_ground(3);
% Compute normal force dynamically using a spring-damper model
N_next = max(K_t * (R0 - norm(P_wheel - P_c)) + C_t * V_rel_z, 0);
% Compute dynamic tire radius
R_eff = R0 - N_next / K_t;
% Adjust contact point based on new effective radius
P_c = P_wheel - R_eff * N_t;
% Compute contact point velocity
V_c = V + cross(omega, (P_c - P_wheel));
% Compute slip velocity
v_slip = V_c - V_ground;
v_slip_xy = v_slip(1:2);  % Extract x, y components
% Compute friction function g(v)
v_mag = norm(v_slip_xy);
g_v = F_c + (F_s - F_c) * exp(-(v_mag / v_s)^2);
% Compute LuGre internal state derivative
dz = v_slip_xy - (v_mag / max(sigma_0 * g_v, 1e-5)) * z;
% Compute tire forces
F_xy = sigma_0 * z + sigma_1 * dz + sigma_2 * v_slip_xy;
F_z = N_t * N_next;  % Normal force projected onto terrain normal
% Combine into force vector
F_tire = [F_xy; F_z];
% Time integration (Euler method)
z_next = z + dz * dt;
end