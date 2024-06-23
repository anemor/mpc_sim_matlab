%%% All the constants for the hopper and propulsion system
% hopper CG     = h
% gimbal joint  = g
% propellers cg = p

USE_CAD = 0;

M_h = 10; % [kg] main hopper body
m_p = 3; % 3 [kg] propulsion mass, under gimbal joint
mass_tot = M_h + m_p;

cg_hopper = [];
d_hopper_gimbal = 0.5; 	% [m] optimistic guess:)
d_hopper_length = 1.5;  % [m] total length of hopper
d_hopper_radius = 0.1; 	% [m] somewhat educated guess
d_hopper_nose = d_hopper_length-d_hopper_gimbal; 	% [m] just used for visualization


r_hg = [-d_hopper_gimbal 0 0]'; 

d_gimbal_prop = 0.15; % [m] distance from gimbal to propeller cm
d_prop_radius = 0.1;

r_hp = [-d_hopper_gimbal-d_gimbal_prop 0 0]'; % [m] from gimbal to propeller % without for now -d_gimbal_prop

Ix_hopper = (1/2)*M_h*d_hopper_radius^2; % [kg*m^2] main hopper body
Iyz_hopper = (1/12)*M_h*(3*d_hopper_radius^2 + d_hopper_length^2); % [kg*m^2] main hopper body
J_hopper = diag([Ix_hopper, Iyz_hopper, Iyz_hopper]); % epfl drone: diag([0.0128, 0.0644, 0.0644]); 

% Propulsion system is a cone now:)
%Ix_prop = (3/10)*m_p*d_gimbal_prop^2; 
%Iyz_prop = (3/5)*m_p*d_gimbal_prop^2 + (3/20)*m_p*d_prop_radius^2; 
%J_prop = diag([Ix_prop, Iyz_prop, Iyz_prop]); 
%J = J_hopper + J_prop;
%J_inv = inv(J);

% Inertia matrix of hopper + propellers from hopper/cylinder center
cg_shift = m_p/(mass_tot) * r_hp;
J_tot_h = J_hopper + m_p*(norm(r_hp,2)^2)*eye(3) - r_hp*r_hp';

% Inertia matrix of hopper + propellers from total mass center
J_tot = J_tot_h - mass_tot * (norm(cg_shift,2)^2*eye(3) - cg_shift*cg_shift');
J = J_tot;

J_inv = inv(J_tot);

G = mass_tot*9.81

% === CAD ================
if USE_CAD % the bad values;) :3dexperience:
    I_xx = 1.97539;
    I_yy = 1.97539;
    I_zz = 1.97539;
    I_xy = 0.0827767;
    I_xz = -0.87486;
    I_yz = 0.4126;
    
    J = [I_xx I_xy I_xz
         I_xy I_yy I_yz
         I_xz I_yz I_zz];
end

% ========================
global J_g J_inv_g dry_mass_g r_hopper_gimbal_g r_hopper_prop_g r_hopper_nose_g cg_shift_g
J_g = J;
J_inv_g = J_inv;
dry_mass_g = mass_tot;
r_hopper_prop_g = r_hp;
r_hopper_gimbal_g = r_hg;
r_hopper_nose_g = [d_hopper_nose 0 0]';
cg_shift_g = cg_shift;
d_hopper_radius_g = d_hopper_radius;
