function Params = SetParameters(Params)

USE_LQR_TERMINAL_CONSTRAINT = false; 

% ===== HOPPER VEHICLE ====================================================
Params.Hopper = struct();

M_h = 10; % [kg] main hopper body
m_p = 3; % 3 [kg] propulsion mass, under gimbal joint
mass_tot = M_h + m_p;

Params.Hopper.mass_tot = mass_tot;

cg_hopper = [];
d_hopper_gimbal = 0.5; 	% [m] optimistic guess:)
d_hopper_length = 1.5;  % [m] total length of hopper
d_hopper_radius = 0.1; 	% [m] somewhat educated guess
d_hopper_nose = d_hopper_length-d_hopper_gimbal; 	% [m] just used for visualization


r_hg = [-d_hopper_gimbal 0 0]'; 

d_gimbal_prop = 0.15; % [m] distance from gimbal to propeller cm
d_prop_radius = 0.1;
d_prop = -d_hopper_gimbal-d_gimbal_prop;

r_hp = [d_prop 0 0]'; % [m] from gimbal to propeller % without for now -d_gimbal_prop

Ix_hopper = (1/2)*M_h*d_hopper_radius^2; % [kg*m^2] main hopper body
Iyz_hopper = (1/12)*M_h*(3*d_hopper_radius^2 + d_hopper_length^2); % [kg*m^2] main hopper body
J_hopper = diag([Ix_hopper, Iyz_hopper, Iyz_hopper]); % epfl drone: diag([0.0128, 0.0644, 0.0644]); 


% Inertia matrix of hopper + propellers from hopper/cylinder center
cg_shift = m_p/(mass_tot) * r_hp;
J_tot_h = J_hopper + m_p*(norm(r_hp,2)^2)*eye(3) - r_hp*r_hp';

% Inertia matrix of hopper + propellers from total mass center
J_tot = J_tot_h - mass_tot * (norm(cg_shift,2)^2*eye(3) - cg_shift*cg_shift');
J_cylinder = J_tot;

J_cylinder_inv = inv(J_tot);

% G = mass_tot*9.81;

I_CAD_xx = 1.97539;
I_CAD_yy = 1.97539;
I_CAD_zz = 1.97539;
I_CAD_xy = 0.0827767;
I_CAD_xz = -0.87486;
I_CAD_yz = 0.4126;

J_CAD = [I_CAD_xx I_CAD_xy I_CAD_xz
        I_CAD_xy I_CAD_yy I_CAD_yz
        I_CAD_xz I_CAD_yz I_CAD_zz];

J_CAD_inv = inv(J_CAD);

Params.Hopper.J_cylinder        = J_cylinder;
Params.Hopper.J_cylinder_inv    = J_cylinder_inv;
Params.Hopper.J_CAD             = J_CAD;
Params.Hopper.J_CAD_inv         = J_CAD_inv;

Params.Hopper.r_hopper_prop = r_hp;
Params.Hopper.r_hopper_gimbal = r_hg;
Params.Hopper.r_hopper_nose = [d_hopper_nose 0 0]';
Params.Hopper.cg_shift = cg_shift;
Params.Hopper.radius = d_hopper_radius;
Params.Hopper.d_prop = d_prop;

% ==== INIT CONDITIONS ====================================================
% ========== OPEN-LOOP SIMULATIONS ========================================
h = 0.1; % 10 ms
N = 500; 
Params.Init.Sim = struct();
Params.Init.Sim.h = h;    % s timestep
Params.Init.Sim.N = N;    % samples

% SIM_1 - Free-fall from 100m ---------------------------------------------
Params.Init.Sim1 = struct();
Params.Init.Sim1.pos    = [0 0 -100]';
Params.Init.Sim1.vel    = [0 0 0]';
Params.Init.Sim1.quat   = euler2q(0, deg2rad(90), 0);
Params.Init.Sim1.angvel = [0 0 0]';

Sim1_U = zeros(4, N+1);
Params.Init.Sim1.AllControls = Sim1_U;
Params.Init.Sim1.AllDistForce = zeros(3, N+1);
Params.Init.Sim1.AllDistTorque = zeros(3, N+1);

% SIM_2 - Thrust up, no gimbal --------------------------------------------
Params.Init.Sim2 = struct();
Params.Init.Sim2.pos    = [0 0 d_prop]';
Params.Init.Sim2.vel    = [0 0 0]';
Params.Init.Sim2.quat   = euler2q(0, deg2rad(90), 0);
Params.Init.Sim2.angvel = [0 0 0]';

Sim2_U = zeros(4, N+1);
% Decreasing F_avgT for the first 5 seconds
pitch = 0; yaw = 0; F_avg = 0; M_diff = 0;
for i = 1:N+1
    t = i*h;
    if t <= 5
        F_avg = (200/5)*(5 - t); 
    else
        F_avg = 0;
    end
    Sim2_U(:, i) = [pitch yaw F_avg M_diff]';
end
Params.Init.Sim2.AllControls = Sim2_U;
Params.Init.Sim2.AllDistForce = zeros(3, N+1);
Params.Init.Sim2.AllDistTorque = zeros(3, N+1);

% SIM_3 - Thrust up with ROLL MOMENT  -------------------------------------
Params.Init.Sim3 = struct();
Params.Init.Sim3.pos    = [0 0 d_prop]';
Params.Init.Sim3.vel    = [0 0 0]';
Params.Init.Sim3.quat   = euler2q(0, deg2rad(90), 0);
Params.Init.Sim3.angvel = [0 0 0]';

Sim3_U = zeros(4, N+1);
% Decreasing F_avgT for the first 5 seconds, small roll moment at beginning
pitch = 0; yaw = 0; F_avg = 0; M_diff = 0;
for i = 1:N+1
    t = i*h;
    if t <= 0.5
        M_diff = 5;
        F_avg = (200/5)*(5 - t);
    elseif t <= 5
        M_diff = 0;
        F_avg = (200/5)*(5 - t);
    else
        F_avg = 0;
        M_diff = 0;
    end
    Sim3_U(:, i) = [pitch yaw F_avg M_diff]';
end
Params.Init.Sim3.AllControls = Sim3_U;
Params.Init.Sim3.AllDistForce = zeros(3, N+1);
Params.Init.Sim3.AllDistTorque = zeros(3, N+1);

% SIM_4 - Thrust and PITCH +-1deg sin -------------------------------------
Params.Init.Sim4 = struct();
Params.Init.Sim4.pos    = [0 0 d_prop]';
Params.Init.Sim4.vel    = [0 0 0]';
Params.Init.Sim4.quat   = euler2q(0, deg2rad(90), 0);
Params.Init.Sim4.angvel = [0 0 0]';

Sim4_U = zeros(4, N+1);
pitch = 0; yaw = 0; F_avg = 0; M_diff = 0;
for i = 1:N+1
    t = i*h;
    if t <= 5
        F_avg = (200/5)*(5 - t);
    else
        F_avg = 0;
    end        
    pitch = deg2rad(0.5)*sin(5*(t-h)); % frequency = 5 Hz
    Sim4_U(:, i) = [pitch yaw F_avg M_diff]';
end
Params.Init.Sim4.AllControls = Sim4_U;
Params.Init.Sim4.AllDistForce = zeros(3, N+1);
Params.Init.Sim4.AllDistTorque = zeros(3, N+1);


% SIM_5 - Thrust and YAW +-1deg sin ---------------------------------------
Params.Init.Sim5 = struct();
Params.Init.Sim5.pos    = [0 0 d_prop]';
Params.Init.Sim5.vel    = [0 0 0]';
Params.Init.Sim5.quat   = euler2q(0, deg2rad(90), 0);
Params.Init.Sim5.angvel = [0 0 0]';

Sim5_U = zeros(4, N+1);
pitch = 0; yaw = 0; F_avg = 0; M_diff = 0;
for i = 1:N+1
    t = i*h;
    if t <= 5
        F_avg = (200/5)*(5 - t);
    else
        F_avg = 0;
    end        
    yaw = deg2rad(0.5)*sin(5*(t-h)); % frequency = 5 Hz
    Sim5_U(:, i) = [pitch yaw F_avg M_diff]';
end
Params.Init.Sim5.AllControls = Sim5_U;
Params.Init.Sim5.AllDistForce = zeros(3, N+1);
Params.Init.Sim5.AllDistTorque = zeros(3, N+1);


% SIM_6 - SIM_2 with disturbance --------------------------------------------
Params.Init.Sim6 = struct();
Params.Init.Sim6.pos    = [0 0 d_prop]';
Params.Init.Sim6.vel    = [0 0 0]';
Params.Init.Sim6.quat   = euler2q(0, deg2rad(90), 0);
Params.Init.Sim6.angvel = [0 0 0]';

Sim6_U = zeros(4, N+1);
% Decreasing F_avgT for the first 5 seconds
pitch = 0; yaw = 0; F_avg = 0; M_diff = 0;
for i = 1:N+1
    t = i*h;
    if t <= 5
        F_avg = (200/5)*(5 - t); 
    else
        F_avg = 0;
    end
    Sim6_U(:, i) = [pitch yaw F_avg M_diff]';
end
Params.Init.Sim6.AllControls = Sim6_U;
Sim6_DistForce = zeros(3, N+1);
for i = 1:N+1
        Sim6_DistForce(:, i) = [0.5 0 0]'; % Wind towards the North x axis
end
Params.Init.Sim6.AllDistForce = Sim6_DistForce;
Params.Init.Sim6.AllDistTorque = zeros(3, N+1);


% SIM_7 - More realistic thrust, not used  ------------------------------------------
Params.Init.Sim7 = struct();
Params.Init.Sim7.pos    = [0 0 d_prop]';
Params.Init.Sim7.vel    = [0 0 0]';
Params.Init.Sim7.quat   = euler2q(0, deg2rad(90), 0);
Params.Init.Sim7.angvel = [0 0 0]';

Sim7_U = zeros(4, N+1);

% Thrust curve parameters
t_ramp_up = 0; % Time to ramp up to full thrust (seconds)
t_full_thrust = 2; % Duration of full thrust (seconds)
t_ramp_down = 0.5; % Time to ramp down from full thrust (seconds)
F_max = 200; % Maximum thrust

% Initialize arrays
pitch = 0; yaw = 0; F_avg = 0; M_diff = 0;
for i = 1:N+1
    t = i*h;
    if t <= t_ramp_up
        F_avg = F_max * (t / t_ramp_up);
    elseif t <= t_ramp_up + t_full_thrust
        F_avg = F_max;
    elseif t <= t_ramp_up + t_full_thrust + t_ramp_down
        F_avg = F_max * (1 - (t - (t_ramp_up + t_full_thrust)) / t_ramp_down);
    else
        F_avg = 0;
    end
    Sim7_U(:, i) = [pitch yaw F_avg M_diff]';
end

% Populate Sim4_U for further use
Params.Init.Sim7.AllControls = Sim7_U;
Params.Init.Sim7.AllDistForce = zeros(3, N+1);
Params.Init.Sim7.AllDistTorque = zeros(3, N+1);


% ========== MPC SIMULATIONS ==============================================
% Position [m] in NED
x0 = 0; y0 = 0; z0 = d_prop;

% Linear Velocity [m/s] in NED
vx0 = 0; vy0 = 0; vz0 = 0;

% Attitude Euler Angles (NED to Body) [rad]
% Hopper Body frame is pitched 90 degs in NED. NED x,y,z => z,y,-x BODY
roll0  = deg2rad(0.01); pitch0 = deg2rad(91); yaw0   = deg2rad(-0.01);

% Angular Velocity [rad/s] in Body
p0 = -0.001; q0 = 0.00001; r0 = 0.00001;

% ============== CONTROL VARIABLES ==============
% Angles created by linear actuator [rad]
linac_pitch_0 = 0; 
linac_yaw_0 = deg2rad(1); 

% Commanded Propeller speed (currently speed=force, so [N])
prop_speed_avg_0 = 200;
prop_speed_diff_0 = 0;

% Set global variables
% global pos0_n_g vel0_n_g q0_nb_g w0_b_g u0_g
Params.Init.pos = [x0 y0 z0]';
Params.Init.vel = [vx0 vy0 vz0]';
Params.Init.quat = euler2q(roll0, pitch0, yaw0);
Params.Init.angvel = [p0 q0 r0]';

Params.Init.control = [linac_pitch_0 
                        linac_yaw_0 
                        prop_speed_avg_0 
                        prop_speed_diff_0]';
Params.Init.MPC = struct();
Params.Init.MPC.h = 0.1;
Params.Init.MPC.N = 200;

% MPC_1 - Setpoint Multshoot Up and down again  ---------------------------
Params.Init.MPC1 = struct();
Params.Init.MPC1.pos    = [0 0 d_prop]';
Params.Init.MPC1.vel    = [0 0 0]';
Params.Init.MPC1.quat   = euler2q(roll0, pitch0, roll0);
Params.Init.MPC1.angvel = [0 0 0]';

% MPC_2 - Setpoint Multshoot Up sideways and down again  ------------------
Params.Init.MPC2 = struct();
Params.Init.MPC2 = Params.Init.MPC1;

% MPC_3 - Setpoint Multshoot Up to 1,1 and down to 2,2  -------------------
Params.Init.MPC3 = struct();
Params.Init.MPC3 = Params.Init.MPC1;

% MPC_3 - Setpoint Multshoot Up to 1,1 and down to 2,2  -------------------
Params.Init.MPC3 = struct();
Params.Init.MPC3 = Params.Init.MPC1;

% MPC_5 - Setpoint Multshoot Only up  -------------------------------------
Params.Init.MPC5 = struct();
Params.Init.MPC5 = Params.Init.MPC1;

% ===== MPC PARAMETERS ====================================================
Params.MPC = struct();

% Weight on states: Q is 13 x 13
w_xy = 125;
w_dxy = 5; % 5 good
w_z = 150;
w_dz = 10; % 10
w_att_roll = 0.01;
w_att_pitchyaw = 1; %100;
w_angvel_roll   = 5;
w_angvel_pitchyaw = 50; % 100


% Weights on controls: R is 4x4
cost_linac      = 0.1;
cost_thrustavg  = 0.01; % thrust
cost_thrustdiff = 0.1; % torque

Q = diag([w_xy,         w_xy,           w_z, ...
        w_dxy,          w_dxy,          w_dz, ...
        w_att_roll,     w_att_roll,     w_att_pitchyaw,         w_att_pitchyaw, ...
        w_angvel_roll,  w_angvel_pitchyaw, w_angvel_pitchyaw]);


R = diag([cost_linac, cost_linac, cost_thrustavg, cost_thrustdiff]);

% Slack variables
% Ws = diag([1e3, 1e3, 1e3, 1e3]);

% Check that they are positive definite
assert(all(eig(Q) > 0), 'Matrix Q is not positive definite.');
assert(all(eig(R) > 0), 'Matrix R is not positive definite.');

Params.MPC.Q = Q;
Params.MPC.R = R;

% Terminal Cost
Params.MPC.Q_tf = Q;   
Params.MPC.Q_tf_mod = diag([w_xy,         w_xy,           w_z, ...
                    w_dxy,          w_dxy,          w_dz, ...
                    w_att_pitchyaw,         w_att_pitchyaw, ...
                    w_angvel_roll,  w_angvel_pitchyaw, w_angvel_pitchyaw]);


if USE_LQR_TERMINAL_CONSTRAINT
    h = 0.1;
    x_eq = [0 0 0 0 0 0 1 0 0 0 0 0 0]'; %
    u_eq = [0 0 1 0]';                %F = avg hover speed
    
    compute_lqr_terminal_cost(Params, x_eq, u_eq, h);
end

% Constraints
% Control u
max_linac_angle = deg2rad(10); %deg
min_prop_speed = 0;
max_prop_speed = 200;
max_prop_diff = 100; 

Params.MPC.lbu = [-max_linac_angle; -max_linac_angle; min_prop_speed; -max_prop_diff/2];
Params.MPC.ubu = [max_linac_angle; max_linac_angle; max_prop_speed; max_prop_diff/2];

% State x
eps = 1e-3;

%global r_hopper_prop_g % to get somewhat more correct z_max position
max_z = -d_prop; % - distance from CG to propeller 0;
max_dx = 1; %1.2;
max_dz = 2; %1.5;
max_d_angvel = deg2rad(100); %0.6;


Params.MPC.lbx  = [-inf; -inf; -inf
                    -max_dx; -max_dx; -max_dz
                    -inf(4,1) 
                    -max_d_angvel; -max_d_angvel; -max_d_angvel];
Params.MPC.ubx = [inf;inf; max_z+eps;
                    max_dx; max_dx; max_dz
                    inf(4,1)
                    max_d_angvel; max_d_angvel; max_d_angvel];

% Control rate (not used)
% controls lower and upper bounds N times
max_linac_rate = deg2rad(90); %deg/s
max_prop_rate = 100;   % N/s

Params.MPC.lbdu = [-max_linac_rate; -max_linac_rate; -max_prop_rate; -max_prop_rate];
Params.MPC.ubdu = [max_linac_rate; max_linac_rate; max_prop_rate; max_prop_rate];



% MPC_1 - Setpoint Multshoot Up and down again  ---------------------------
Params.MPC.MPC1 = struct();
Params.MPC.MPC1.Np = 10;
Params.MPC.MPC1.Q = Q;
Params.MPC.MPC1.R = R;
Params.MPC.MPC1.lbx = [-inf; -inf; -inf
                        -max_dx; -max_dx; -max_dz
                        -inf(4,1) 
                        -max_d_angvel; -max_d_angvel; -max_d_angvel];
Params.MPC.MPC1.ubx = [inf;inf; max_z+eps;
                        max_dx; max_dx; max_dz
                        inf(4,1)
                        max_d_angvel; max_d_angvel; max_d_angvel];
Params.MPC.MPC1.lbu = [-max_linac_angle; -max_linac_angle; min_prop_speed; -max_prop_diff/2];
Params.MPC.MPC1.ubu = [max_linac_angle; max_linac_angle; max_prop_speed; max_prop_diff/2];
             
% MPC_2 - Setpoint Multshoot Up sideways and down again  ---------------------------
Params.MPC.MPC2 = struct();
Params.MPC.MPC2 = Params.MPC.MPC1; % same Q,R and bounds as MCP1

% MPC_3 - Setpoint Multshoot Up to 1,1 and down to 2,2  ---------------------------
Params.MPC.MPC3 = struct();
Params.MPC.MPC3 = Params.MPC.MPC1; % same Q,R and bounds as MCP1

% MPC_5 - Setpoint Multshoot only up ---------------------------
Params.MPC.MPC5 = struct();
Params.MPC.MPC5 = Params.MPC.MPC1; % same Q,R and bounds as MCP1

% ===== GUIDANCE MODULE (CURRENTLY SETPOINT ONLY) =========================
Params.Guidance = struct();

apogee_pos = [0 0 -10]'; % [m]
landing_pos = [0 0 d_prop]'; % [m]

Params.Guidance.apogee_setpoint = [  apogee_pos;
                                        0; 0; 0;
                                        1; 0; 0; 0;
                                        0; 0; 0]; 

Params.Guidance.landing_setpoint = [  landing_pos;
                                        0; 0; 0;
                                        1; 0; 0; 0;
                                        0; 0; 0]; 


% MPC_1 - Setpoint Multshoot Up and down again  ---------------------------
Params.Guidance.MPC1 = struct();
Params.Guidance.MPC1.apogee_setpoint = [0; 0; -10;
                                        0; 0; 0;
                                        1; 0; 0; 0;
                                        0; 0; 0]; 
Params.Guidance.MPC1.landing_setpoint = [0; 0; d_prop;
                                        0; 0; 0;
                                        1; 0; 0; 0;
                                        0; 0; 0];                                              
                  
% MPC_2 - Setpoint Multshoot Up to 1,1 and down again  --------------------
Params.Guidance.MPC2 = struct();
Params.Guidance.MPC2.apogee_setpoint = [1; 0; -10;
                                        0; 0; 0;
                                        1; 0; 0; 0;
                                        0; 0; 0]; 
Params.Guidance.MPC2.landing_setpoint = [1; 0; d_prop;
                                        0; 0; 0;
                                        1; 0; 0; 0;
                                        0; 0; 0];                                              

% MPC_3 - Setpoint Multshoot Up to 1,1 and down to 2,2  -------------------
Params.Guidance.MPC3 = struct();
Params.Guidance.MPC3.apogee_setpoint = [1; 0; -10;
                                        0; 0; 0;
                                        1; 0; 0; 0;
                                        0; 0; 0]; 
Params.Guidance.MPC3.landing_setpoint = [2; 0; d_prop;
                                        0; 0; 0;
                                        1; 0; 0; 0;
                                        0; 0; 0];                                              
% MPC_5 - Setpoint Multshoot only up --------------------------------------
Params.Guidance.MPC1 = struct();
Params.Guidance.MPC1.apogee_setpoint = [0; 0; -10;
                                        0; 0; 0;
                                        1; 0; 0; 0;
                                        0; 0; 0]; 
Params.Guidance.MPC1.landing_setpoint = Params.Guidance.MPC1.apogee_setpoint;                                            
         
end