function [P_c, N_t] = get_contact_point(P_wheel, R_eff, terrain_func)
% Inputs:
%   P_wheel     - [3x1] Wheel center position
%   R_eff       - Effective tire radius
%   terrain_func - Function handle for terrain height f(x,y)
%
% Outputs:
%   P_c  - [3x1] Contact point position
%   N_t  - [3x1] Terrain normal vector at P_c
% Estimate ground height
x = P_wheel(1);
y = P_wheel(2);
z_ground = terrain_func(x, y);
P_c = [x; y; z_ground];
% Compute terrain normal using numerical differentiation
delta = 1e-3;
dzdx = (terrain_func(x + delta, y) - terrain_func(x - delta, y)) / (2 * delta);
dzdy = (terrain_func(x, y + delta) - terrain_func(x, y - delta)) / (2 * delta);
% Compute normal vector
N_t = [-dzdx; -dzdy; 1];
N_t = N_t / norm(N_t);
% Adjust contact point using dynamic radius
P_c = P_wheel - R_eff * N_t;
end