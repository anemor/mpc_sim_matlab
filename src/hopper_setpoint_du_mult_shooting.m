% point stabilization + Multiple shooting + Runge Kutta
clear all
close all
clc

global J_g J_inv_g dry_mass_g r_hopper_prop_g d_hopper_radius_g % hopper_properties
global pos0_n_g vel0_n_g q0_nb_g w0_b_g                         % initial_conditions
global target_apogee_pos_vec_g target_landing_pos_vec_g             % guidance_settings

% CasADi v3.6.5
addpath('C:\Users\aneki\OneDrive - NTNU\Documents\casadi-3.6.5-windows64-matlab2018b'); % laptop
% addpath('C:\Users\akmorkem\OneDrive - NTNU\Documents\casadi-3.6.4-windows64-matlab2018b');
import casadi.*

% Initialize state while MPC is set up
fsm_state = FSM.IDLE;

h = 0.1; %[s]
Np = 10; % prediction horizon


% load constants
hopper_properties


% Create symbolic variables for states and control inputs
pos = SX.sym('pos', 3); % position
vel = SX.sym('vel', 3); % velocity
q = SX.sym('q', 4); % quaternion
angvel = SX.sym('angvel', 3); % angular velocity
states_x = [pos; vel; q; angvel]; 
NX = length(states_x);


linac_pitch = SX.sym('linac_pitch'); 
linac_yaw = SX.sym('linac_yaw'); 
prop_speed_avg = SX.sym('prop_speed_avg');
prop_speed_diff = SX.sym('prop_speed_diff');
controls_u = [linac_pitch; linac_yaw; prop_speed_avg; prop_speed_diff]; 
NU = length(controls_u);

% Define input rate variables
delta_u = SX.sym('delta_u', NU);

rhs = state_dynamics(states_x, controls_u, [0 0 0]', [0 0 0]'); 

f = Function('f',{states_x,controls_u}, {rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U', NU, Np);     % Decision variables (controls)
P = SX.sym('P', NX + NX);   % parameters (which include the initial state and the reference state)



% A vector that represents the states over the optimization problem.
X = SX.sym('X', NX, (Np+1));

% Control rates
U_rate = SX.sym('U_rate', NU, Np); % Control rate variables

S_u = SX.sym('S_u', NU, Np); % slack variables for control rate over the horizon


obj = 0; % Objective function
g = [];  % constraints vector
g_continuity = []; % state constraints
g_u = []; % control constraints
g_du = []; % control rate constraints

[Q,R,Ws] = get_obj();


k1 = f(st, con);   
k2 = f(st + h/2*k1, con); 
k3 = f(st + h/2*k2, con);
k4 = f(st + h*k3, con); 
st_next_RK4=st + h/6*(k1 +2*k2 +2*k3 +k4); 
st_next_RK4(7:10) = st_next_RK4(7:10)/norm(st_next_RK4(7:10)); % quaternion normalization!

RK4 = Function('RK4', {st, con}, {st_next_RK4});


st0 = X(:,1);           % initial state
g_continuity = [g_continuity; st0 - P(1:NX)];    % initial condition constraints

% st  = X(:,1);           % initial state
% g = [g; st-P(1:NX)];    % initial condition constraints


for k = 1:Np

    % Previous control
    if k == 1
        u_km1 = P(2*NX+1:2*NX+NU,1);
    else
        % controlling the system
        du_temp = zeros(NX,1);
        for i = 1 : k-1
            du_temp = du_temp + U_rate(:,i);
        end
        u_km1 = P(2*NX+1:2*NX+NU,1) + du_temp;
    end
    
    z_k = X(:,k);   % current state

    du_k = U_rate(:,k); % current control rate
    sc_k = S_c(:,k); % current slack variable

    r_k = P(NX+1:2*NX,1); % reference state

    % Objective function
    obj  = obj + (z_k-r_k)'*Q*(z_k-r_k) + du_k'*R*du_k + sc_k'*W*sc_k;


    u_k = u_km1 + du_k; % current control
    
    % constraints on control + control rate
    g_u = [g_u, u_k];
    g_du = [g_du; du_k - sc_k; du_k + sc_k];
    z_kp_next = RK4(z_k, u_k); % dx dynamics

    % continuity condition
    x_kp1_next = X(:,k+1); % cont condition
    g_continuity = [g_continuity; x_kp1_next - z_kp_next]; % ensure dynamics are respected
end

% Make decision variable a column vec
OPT_variables = [reshape(X, NX*(Np+1), 1);  % x
                % reshape(U, NU*Np, 1); 
                reshape(U_rate, NU*Np, 1);  % u
                reshape(S_u, NU*Np, 1)];    % du


% for i = 1:N-1
%     g = [g; U_rate(:,i) - ( U(:,i+1) - U(:,i) )  / h];
% end
g = [g_continuity g_u g_du]';
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts                                    = struct;
opts.ipopt.max_iter                     = 2000;
opts.ipopt.print_level                  = 3;%0,3
opts.print_time                         = 0;
opts.ipopt.acceptable_tol               = 1e-8;
opts.ipopt.acceptable_obj_change_tol    = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

% ===== Time to set up the constraints =============================
% MY CODE STARTS HERE

NXall = NX*(Np+1);  % all state variables across prediction horizon
NUall = NU*Np;      % all control variables across prediction horizon
NU_rate_all = NU*(Np-1); % all control rate variables across prediction horizon
NU_rate_slack_all = 2*NU_rate_all;  % for each control variable, also handle slack variables for flexibility

args = struct;


% Equality/continuity constraints (= bounds are 0)
args.lbg(1:NXall) = 0;  % -1e-20  % Equality constraints
args.ubg(1:NXall) = 0;  % 1e-20   % Equality constraints

% Continuity constraints to g 
for k = 1:Np

end



% Inequality constraints on state and control
% These ensure continuity between states across prediction horizon Np 

% lbx = [x_low_bounds repeated untill NXall, then u_low_bounds repeated until NXall+NUall]
% same with ubxg
[x_low_bounds, x_up_bounds] = ineq_constraints_state();
start = 0; vars = NX; stop = NXall;
for i = 1:vars
    args.lbx(start+i:vars:stop,1) = x_low_bounds(i);
    args.ubx(start+i:vars:stop,1) = x_up_bounds(i); 
end


% Control constraints to ensure that they are physically feasible and within safe limits

[u_low_bounds, u_up_bounds] = ineq_constraints_control();
start = NXall; vars = NU; stop = NXall+NUall;
for i = 1:vars
    args.lbx(start+i:vars:stop,1) = u_low_bounds(i);
    args.ubx(start+i:vars:stop,1) = u_up_bounds(i); 
end

% Control rate constraints
[u_rate_low_bounds, u_rate_up_bounds] = ineq_constraints_control_rate();
for k = 1:Np-1 % no control change after Np
    uk = 
end


% Control rate with slack variables (so limit both control rate and slack for each control action)
% start = NXall+NUall; vars = NU; stop = NXall+NUall+NUall;
% for i = 1:NU
%     args.lbx(start+i:NU:NXall+NUall+NU*(Np-1),1) = [-Inf -Inf -Inf -Inf]';
%     args.ubx(start+i:NU:NXall+NUall+NU*(Np-1),1) = [Inf Inf Inf Inf]'; 
% end

% args.lbg(NXall+NUall+1 : n_state*(Np+1)+n_control*Nc+2*n_control*Nc) ...
%     = kron(ones(Nc,1), [-Inf; -Inf; dv_min; domega_min]);
% args.ubg(n_state*(Np+1)+n_control*Nc+1 : n_state*(Np+1)+n_control*Nc+2*n_control*Nc) ...
%     = kron(ones(Nc,1), [dv_max; domega_max; Inf; Inf]);


% add bounds on slack variables on control rate
% [u_rate_low_bounds, u_rate_up_bounds] = ineq_constraints_control_rate();
% start = NXall+NUall; vars = NU; stop = NXall+NUall+NUall;
% for i = 1:vars
%     args.lbx(start+i:vars:stop,1) = u_rate_low_bounds(i);
%     args.ubx(start+i:vars:stop,1) = u_rate_up_bounds(i); 
% end

% MY CODE ENDS HERE
% EXAMPLE NMPC multi shooting Nc slack STARTS HERE

args = struct;

% Constraints to ensure continuity between states across prediction horizon Np 
args.lbg(1:n_state*(Np+1),1) = 0;  % -1e-20  % Continuity constraints
args.ubg(1:n_state*(Np+1),1) = 0;  % 1e-20   % Continuity constraints

% Control constraints to ensure that they are physically feasible and within safe limits
% args.lbg(n_state*(Np+1)+1 : n_state*(Np+1) + n_control*Nc,1) ...
%                                     = kron(ones(Nc,1),[v_min; omega_min]);
% args.ubg(n_state*(Np+1)+1 : n_state*(Np+1) + n_control*Nc,1) ...
%                                     = kron(ones(Nc,1),[v_max; omega_max]);

% Control Increment constraints with slack variables
args.lbg(n_state*(Np+1)+n_control*Nc+1 : n_state*(Np+1)+n_control*Nc+2*n_control*Nc) ...
    = kron(ones(Nc,1), [-Inf; -Inf; dv_min; domega_min]);
args.ubg(n_state*(Np+1)+n_control*Nc+1 : n_state*(Np+1)+n_control*Nc+2*n_control*Nc) ...
    = kron(ones(Nc,1), [dv_max; domega_max; Inf; Inf]);

% State constraints
% args.lbx(1:n_state*(Np+1),1) = kron(ones(Np+1,1),[x_min; y_min; theta_min]);
% args.ubx(1:n_state*(Np+1),1) = kron(ones(Np+1,1),[x_max; y_max; theta_max]);

% Control Increment Constraints
args.lbx(n_state*(Np+1)+1: n_state*(Np+1) + n_control*Nc,1) = ...
                                    kron(ones(Nc,1),[dv_min; domega_min] + Sc_min);
args.ubx(n_state*(Np+1)+1: n_state*(Np+1) + n_control*Nc,1) = ...
                                    kron(ones(Nc,1),[dv_max; domega_max] + Sc_max);
% Slack variable constraints
args.lbx(n_state*(Np+1)+n_control*Nc+1 : n_state*(Np+1)+n_control*Nc + n_control*Nc) ...
    = kron(ones(Nc,1),Sc_min);
args.ubx(n_state*(Np+1)+n_control*Nc+1 : n_state*(Np+1)+n_control*Nc + n_control*Nc) ...
    = kron(ones(Nc,1),Sc_max);




% EXAMPLE NMPC multi shooting Nc slack ENDS HERE

%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP


% THE SIMULATION LOOP STARTS HERE
%-------------------------------------------
% load initial state and control
init_conditions

guidance_settings;

t0 = 0;
%x0 = [pos0_n_g; vel0_n_g; q0_nb_g; w0_b_g]; 
%x0 = [  0; 0; -d_rocket_gimbal_g(1);
 %      0; 0; 0;
  %     0.7132; 0.0001; 0.7009; 0.0001;
   %    0; 0; 0];    % initial condition
        
x_ref = [target_apogee_pos_vec_g;
        0; 0; 0;
        1; 0; 0; 0;
        0; 0; 0]; % Reference posture

pos0_n=pos0_n_g; vel0_n=vel0_n_g; q0_nb=q0_nb_g; w0_b=w0_b_g;
x0 = [pos0_n; vel0_n; q0_nb; w0_b];

update_sim_figure(pos0_n, q0_nb, w0_b);

x_out_mpc(:,1) = x0;               % contains the history of states
t(1) = t0;

U0 = zeros(Np,NU);           % NU control inputs for each robot
% U_rate0 = zeros(N-1, NU);   % initialization of the states decision variables
X0 = repmat(x0,1,Np+1)';     

sim_tim = 20; % Maximum simulation time
Nsim = sim_tim / h;
moving_avg_window = 10; % for n moving average for position error


% Start MPC
state_err = 10; % l
pos_err = norm(x0(3)-x_ref(3), 2);
pos_err_vec = pos_err*ones(1, moving_avg_window);
pos_err_moving_avg = pos_err;

i_mpc = 0;
i_end = Nsim;

% Matrices for storing data
x_traj_all = []; %zeros(NX,N+1,Nsim);
u_opt_all=[];
u_mpcsol_store = zeros(NU,Np,Nsim);   % Initialize storage for u_cl
x_ref_all = zeros(NX,Nsim);         % Initialize storage for x_ref

% MAIN SIMULATION LOOP:))
% runs for Nsim steps, or until fsm_state is FSM.LANDED or FSM.ABORT
fsm_state = FSM.ARMED;

main_loop = tic;
while(i_mpc < Nsim)
    
    state_err = norm(state_error(x0,x_ref),2);
    
    pos_err = norm(x0(1:3)-x_ref(1:3), 2);
    [pos_err_vec, pos_err_moving_avg] = update_moving_avg(pos_err_vec, pos_err);
    
    if state_err > 100 || pos_err > 20 
        fsm_state = FSM.ABORT;
    end

    switch fsm_state
        case FSM.ARMED
            x_ref = [   target_apogee_pos_vec_g
                        0; 0; 0;
                        1; 0; 0; 0;
                        0; 0; 0]; % Reference posture
            fsm_state = FSM.ASCENT;

        case FSM.ASCENT
            if pos_err_moving_avg < 0.1
                disp('Reached apogee, setting new reference and entering FSM.LANDING')
                x_ref = [   target_landing_pos_vec_g
                            0; 0; 0;
                            1; 0; 0; 0;
                            0; 0; 0]; % Reference posture
                fsm_state = FSM.HOVER;
            end

        case FSM.HOVER
            fsm_state = FSM.LANDING;
            
        case FSM.LANDING
            if pos_err_moving_avg < 1
                fsm_state = FSM.TOUCHDOWN;
            end

        case FSM.TOUCHDOWN
            % Mission accomplished - Exit simulation loop.
            i_end = i_mpc;
            break;
            
        case FSM.ABORT
            % Exit simulation. 
            % In real experiment we would do some safe maneuver here
            i_end = i_mpc;
            break;
    end

    
    % ======== NMPC to get control ==================================
    args.p   = [x0; x_ref]; % set the values of the parameters vector
    x_ref_all(:,i_mpc+1) = x_ref;   % store current reference

    % initial value of the optimization variables
    args.x0  = [reshape(X0', NXall, 1); 
                reshape(U0', NUall, 1)];
                % reshape(U_rate0, NU*(N-1), 1)]; % initial value of the decision variables

    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

    % Extract state and control trajectories from solution
    x_traj_raw = full(sol.x(1:NXall));
    x_traj = reshape(full(sol.x(1:NXall))', NX, Np+1)';

    u_traj_raw = full(sol.x(NXall+1:end));
    u_traj = reshape(u_traj_raw', NU, Np)';
    x_traj_all(:,1:NX,i_mpc+1) = x_traj; 
    %u_mpcsol_store(:,1:NU,i_mpc+1) = u_traj;
    u_opt_all = [u_opt_all ; u_traj(1,:)];

    t(i_mpc+1) = t0;
    
    % ===============================================================
    % Apply the control and shift the solution using f (dynamics and RK4)
    [t0, x0, U0] = shift(h, t0, x0, u_traj, f);

    
    x_out_mpc(:, i_mpc+2) = x0;
    X0 = reshape(x_traj', NX, Np+1)' % get solution TRAJECTORY
    
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:); X0(end,:)]
    
    update_sim_figure(x0(1:3), x0(7:10), x0(11:13));
    
    i_mpc
    i_mpc = i_mpc + 1;

end;

main_loop_time = toc(main_loop);
ss_error = state_err
average_mpc_time = main_loop_time/(i_mpc+1)

i_end
%% Store data
%u_FM_simout = zeros(length(i_end), 9);
mpcstore_timestamps         = t(1:i_end)';
mpcstore_states             = x_out_mpc(:,1:i_end)';
mpcstore_predicted_states   = x_traj_all(:,:,1:i_end);     % (N+1)xNXx(Nsim/h) (11x13x150)
mpcstore_controls           = u_opt_all(1:i_end,:);
mpcstore_predicted_controls = u_mpcsol_store(:,:,1:i_end); %mpcstore_forces_moments     = u_FM_simout(1:i_end);
mpcstore_state_reference    = x_ref_all(:,1:i_end)'; %mpcstore_forces_moments     = u_FM_simout(1:i_end);

fig_folder = 'fig/';
name = 'setpoint_xynonzero';

save([fig_folder name '/mpc_timestamps'],         'mpcstore_timestamps')
save([fig_folder name '/mpc_states'],             'mpcstore_states')
save([fig_folder name '/mpc_controls'],           'mpcstore_controls')
save([fig_folder name '/mpc_predicted_states'],   'mpcstore_predicted_states')
save([fig_folder name '/mpc_predicted_controls'], 'mpcstore_predicted_controls')
save([fig_folder name '/mpc_state_reference'],    'mpcstore_state_reference')


%% Plot

figname = name;
plot_sim(mpcstore_timestamps, ...
        mpcstore_states, ...
        mpcstore_controls, ...
        mpcstore_state_reference,   ...
        figname);


clear all
close all
clc

global J_g J_inv_g dry_mass_g r_hopper_prop_g d_hopper_radius_g % hopper_properties
global pos0_n_g vel0_n_g q0_nb_g w0_b_g                         % initial_conditions
global target_apogee_pos_vec_g target_landing_pos_vec_g             % guidance_settings

% CasADi v3.6.5
addpath('C:\Users\aneki\OneDrive - NTNU\Documents\casadi-3.6.5-windows64-matlab2018b'); % laptop
% addpath('C:\Users\akmorkem\OneDrive - NTNU\Documents\casadi-3.6.4-windows64-matlab2018b');
import casadi.*

% Initialize state while MPC is set up
fsm_state = FSM.IDLE;

h = 0.1; %[s]
Np = 10; % prediction horizon


% load constants
hopper_properties


% Create symbolic variables for states and control inputs
pos = SX.sym('pos', 3); % position
vel = SX.sym('vel', 3); % velocity
q = SX.sym('q', 4); % quaternion
angvel = SX.sym('angvel', 3); % angular velocity
states_x = [pos; vel; q; angvel]; 
NX = length(states_x);


linac_pitch = SX.sym('linac_pitch'); 
linac_yaw = SX.sym('linac_yaw'); 
prop_speed_avg = SX.sym('prop_speed_avg');
prop_speed_diff = SX.sym('prop_speed_diff');
controls_u = [linac_pitch; linac_yaw; prop_speed_avg; prop_speed_diff]; 
NU = length(controls_u);

% Define input rate variables
delta_u = SX.sym('delta_u', NU);

rhs = state_dynamics(states_x, controls_u, [0 0 0]', [0 0 0]'); 

f = Function('f',{states_x,controls_u}, {rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U', NU, Np);     % Decision variables (controls)
P = SX.sym('P', NX + NX);   % parameters (which include the initial state and the reference state)

% Control rates
U_rate = SX.sym('U_rate', NU, Np-1); % Control rate variables

% A vector that represents the states over the optimization problem.
X = SX.sym('X', NX, (Np+1));

obj = 0; % Objective function
g = [];  % constraints vector

st  = X(:,1);           % initial state
g = [g; st-P(1:NX)];    % initial condition constraints
for k = 1:Np
    st = X(:,k);  
    con = U(:,k);

    obj = obj + stage_cost(st, P(NX+1:end), con);
    st_next = X(:,k+1);

    k1 = f(st, con);   
    k2 = f(st + h/2*k1, con); 
    k3 = f(st + h/2*k2, con);
    k4 = f(st + h*k3, con); 
    st_next_RK4=st + h/6*(k1 +2*k2 +2*k3 +k4); 
    st_next_RK4(7:10) = st_next_RK4(7:10)/norm(st_next_RK4(7:10)); % quaternion normalization!

    g = [g; st_next-st_next_RK4]; % compute constraints 
end

% make the decision variable a column vector
OPT_variables = [reshape(X, NX*(Np+1), 1); 
                reshape(U, NU*Np, 1)]; 
                %reshape(U_rate, NU*(N-1), 1)];

% for i = 1:N-1
%     g = [g; U_rate(:,i) - ( U(:,i+1) - U(:,i) )  / h];
% end

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 4000; %2000
opts.ipopt.print_level =3;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

% Time to set up the constraints
NXall = NX*(Np+1);
NUall = NU*Np;

args = struct;


% Equality constraints (= bounds are 0)
args.lbg(1:NXall) = 0;  % -1e-20  % Equality constraints
args.ubg(1:NXall) = 0;  % 1e-20   % Equality constraints

% Inequality constraints on state and control
% lbx = [x_low_bounds repeated untill NXall, then u_low_bounds repeated until NXall+NUall]
% same with ubxg
[x_low_bounds, x_up_bounds] = ineq_constraints_state();
for i = 1:NX
    args.lbx(i:NX:NXall,1) = x_low_bounds(i);
    args.ubx(i:NX:NXall,1) = x_up_bounds(i); 
end

[u_low_bounds, u_up_bounds] = ineq_constraints_control();
for i = 1:NU
    args.lbx(NXall+i:NU:NXall+NUall,1) = u_low_bounds(i);
    args.ubx(NXall+i:NU:NXall+NUall,1) = u_up_bounds(i); 
end

% add bounds on slack variables on control rate
[u_rate_low_bounds, u_rate_up_bounds] = ineq_constraints_control_rate();

Su_min = u_rate_low_bounds;
Su_max = u_rate_up_bounds;
% add the ineq constraints on control rate here
for i = 1:NU % NU*(N-1), is NU_rate_all
    args.lbx(NXall+NUall+i:NU:NXall+NUall+NU*(Np-1),1) = u_rate_low_bounds(i);
    args.ubx(NXall+NUall+i:NU:NXall+NUall+NU*(Np-1),1) = u_rate_up_bounds(i); 
end
%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP


% THE SIMULATION LOOP STARTS HERE
%-------------------------------------------
% load initial state and control
init_conditions

guidance_settings;

t0 = 0;
%x0 = [pos0_n_g; vel0_n_g; q0_nb_g; w0_b_g]; 
%x0 = [  0; 0; -d_rocket_gimbal_g(1);
 %      0; 0; 0;
  %     0.7132; 0.0001; 0.7009; 0.0001;
   %    0; 0; 0];    % initial condition
        
x_ref = [target_apogee_pos_vec_g;
        0; 0; 0;
        1; 0; 0; 0;
        0; 0; 0]; % Reference posture

pos0_n=pos0_n_g; vel0_n=vel0_n_g; q0_nb=q0_nb_g; w0_b=w0_b_g;
x0 = [pos0_n; vel0_n; q0_nb; w0_b];

update_sim_figure(pos0_n, q0_nb, w0_b);

x_out_mpc(:,1) = x0;               % contains the history of states
t(1) = t0;

U0 = zeros(Np,NU);           % NU control inputs for each robot
% U_rate0 = zeros(N-1, NU);   % initialization of the states decision variables
X0 = repmat(x0,1,Np+1)';     

sim_tim = 20; % Maximum simulation time
Nsim = sim_tim / h;
moving_avg_window = 10; % for n moving average for position error


% Start MPC
state_err = 10; % l
pos_err = norm(x0(3)-x_ref(3), 2);
pos_err_vec = pos_err*ones(1, moving_avg_window);
pos_err_moving_avg = pos_err;

i_mpc = 0;
i_end = Nsim;

% Matrices for storing data
x_traj_all = []; %zeros(NX,N+1,Nsim);
u_opt_all=[];
u_mpcsol_store = zeros(NU,Np,Nsim);   % Initialize storage for u_cl
x_ref_all = zeros(NX,Nsim);         % Initialize storage for x_ref

% MAIN SIMULATION LOOP:))
% runs for Nsim steps, or until fsm_state is FSM.LANDED or FSM.ABORT
fsm_state = FSM.ARMED;

main_loop = tic;
while(i_mpc < Nsim)
    
    state_err = norm(state_error(x0,x_ref),2);
    
    pos_err = norm(x0(1:3)-x_ref(1:3), 2);
    [pos_err_vec, pos_err_moving_avg] = update_moving_avg(pos_err_vec, pos_err);
    
    if state_err > 100 || pos_err > 20 
        fsm_state = FSM.ABORT;
    end

    switch fsm_state
        case FSM.ARMED
            x_ref = [   target_apogee_pos_vec_g
                        0; 0; 0;
                        1; 0; 0; 0;
                        0; 0; 0]; % Reference posture
            fsm_state = FSM.ASCENT;

        case FSM.ASCENT
            if pos_err_moving_avg < 0.1
                disp('Reached apogee, setting new reference and entering FSM.LANDING')
                x_ref = [   target_landing_pos_vec_g
                            0; 0; 0;
                            1; 0; 0; 0;
                            0; 0; 0]; % Reference posture
                fsm_state = FSM.HOVER;
            end

        case FSM.HOVER
            fsm_state = FSM.LANDING;
            
        case FSM.LANDING
            if pos_err_moving_avg < 1
                fsm_state = FSM.TOUCHDOWN;
            end

        case FSM.TOUCHDOWN
            % Mission accomplished - Exit simulation loop.
            i_end = i_mpc;
            break;
            
        case FSM.ABORT
            % Exit simulation. 
            % In real experiment we would do some safe maneuver here
            i_end = i_mpc;
            break;
    end

    
    % ======== NMPC to get control ==================================
    args.p   = [x0; x_ref]; % set the values of the parameters vector
    x_ref_all(:,i_mpc+1) = x_ref;   % store current reference

    % initial value of the optimization variables
    args.x0  = [reshape(X0', NXall, 1); 
                reshape(U0', NUall, 1)];
                % reshape(U_rate0, NU*(N-1), 1)]; % initial value of the decision variables

    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

    % Extract state and control trajectories from solution
    x_traj_raw = full(sol.x(1:NXall));
    x_traj = reshape(full(sol.x(1:NXall))', NX, Np+1)';

    u_traj_raw = full(sol.x(NXall+1:end));
    u_traj = reshape(u_traj_raw', NU, Np)';
    x_traj_all(:,1:NX,i_mpc+1) = x_traj; 
    %u_mpcsol_store(:,1:NU,i_mpc+1) = u_traj;
    u_opt_all = [u_opt_all ; u_traj(1,:)];

    t(i_mpc+1) = t0;
    
    % ===============================================================
    % Apply the control and shift the solution using f (dynamics and RK4)
    [t0, x0, U0] = shift(h, t0, x0, u_traj, f);

    
    x_out_mpc(:, i_mpc+2) = x0;
    X0 = reshape(x_traj', NX, Np+1)' % get solution TRAJECTORY
    
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:); X0(end,:)]
    
    update_sim_figure(x0(1:3), x0(7:10), x0(11:13));
    
    i_mpc
    i_mpc = i_mpc + 1;

end;

main_loop_time = toc(main_loop);
ss_error = state_err
average_mpc_time = main_loop_time/(i_mpc+1)

i_end
%% Store data
%u_FM_simout = zeros(length(i_end), 9);
mpcstore_timestamps         = t(1:i_end)';
mpcstore_states             = x_out_mpc(:,1:i_end)';
mpcstore_predicted_states   = x_traj_all(:,:,1:i_end);     % (N+1)xNXx(Nsim/h) (11x13x150)
mpcstore_controls           = u_opt_all(1:i_end,:);
mpcstore_predicted_controls = u_mpcsol_store(:,:,1:i_end); %mpcstore_forces_moments     = u_FM_simout(1:i_end);
mpcstore_state_reference    = x_ref_all(:,1:i_end)'; %mpcstore_forces_moments     = u_FM_simout(1:i_end);

fig_folder = 'fig/';
name = 'setpoint_xynonzero';

save([fig_folder name '/mpc_timestamps'],         'mpcstore_timestamps')
save([fig_folder name '/mpc_states'],             'mpcstore_states')
save([fig_folder name '/mpc_controls'],           'mpcstore_controls')
save([fig_folder name '/mpc_predicted_states'],   'mpcstore_predicted_states')
save([fig_folder name '/mpc_predicted_controls'], 'mpcstore_predicted_controls')
save([fig_folder name '/mpc_state_reference'],    'mpcstore_state_reference')


%% Plot

figname = name;
plot_sim(mpcstore_timestamps, ...
        mpcstore_states, ...
        mpcstore_controls, ...
        mpcstore_state_reference,   ...
        figname);

%plot_sim(t', x_out_mpc(:,1:end-1)', u_opt_all, u_FM_simout, figname);

%disp('xs')
%plot_sim(t,x_mpcsol,sol_u)

% xs = reference, same all the time so (NX,1)
% xx = actual trajectory
% xx1 = solution trajectory

%Draw_MPC_point_stabilization_v1 (t,xx,xx1,u_cl,xs,N,0.3)