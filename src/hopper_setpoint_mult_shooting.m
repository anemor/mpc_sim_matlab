% point stabilization + Multiple shooting + Runge Kutta

clear all; close all; clc

% ===== PARAMETERS =========================================================================
DATA_FOLDER = 'fig/';
DATA_NAME ='mpc_1/';
PLOT = true;
LIVE_3D_VIZ = true;

CASADI_PATH = 'C:\Users\aneki\OneDrive - NTNU\Documents\casadi-3.6.5-windows64-matlab2018b'; 
% ==========================================================================================


addpath(CASADI_PATH);
import casadi.*

% ==========================================================================================
% SET UP MPC
% ==========================================================================================

% Load global settings
Params = SetParameters();

% Initialize state while MPC is set up
fsm_state = FSM.IDLE;

h = 0.1; %[s]
Np = 10; % prediction horizon


% load constants

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

rhs = state_dynamics(Params.Hopper, states_x, controls_u, [0 0 0]', [0 0 0]'); 

f = Function('f',{states_x,controls_u}, {rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U', NU, Np);     % Decision variables (controls)
P = SX.sym('P', NX + NX);   % parameters (which include the initial state and the reference state)

% Control rates
% U_rate = SX.sym('U_rate', NU, Np-1); % Control rate variables

% A vector that represents the states over the optimization problem.
X = SX.sym('X', NX, (Np+1));

obj = 0; % Objective function
g = [];  % constraints vector

% [Q, R] = get_cost_matrices(); 
Q = Params.MPC.Q;
R = Params.MPC.R;

st  = X(:,1);           % initial state
g = [g; st-P(1:NX)];    % initial condition constraints
for k = 1:Np
    st = X(:,k);  
    con = U(:,k);

    ex = state_error(st, P(NX+1:end));
    stage_cost = ex'*Q*ex + con'*R*con;

    obj = obj + stage_cost;
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
opts.ipopt.print_level = 0;%0,3
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
x_low_bounds = Params.MPC.lbx;
x_up_bounds  = Params.MPC.ubx;
for i = 1:NX
    args.lbx(i:NX:NXall,1) = x_low_bounds(i);
    args.ubx(i:NX:NXall,1) = x_up_bounds(i); 
end

u_low_bounds = Params.MPC.lbu;
u_up_bounds  = Params.MPC.ubu;
for i = 1:NU
    args.lbx(NXall+i:NU:NXall+NUall,1) = u_low_bounds(i);
    args.ubx(NXall+i:NU:NXall+NUall,1) = u_up_bounds(i); 
end

% add bounds on slack variables on control rate
%u_rate_low_bounds = Params.MPC.lbdu;
%u_rate_up_bounds  = Params.MPC.ubdu;

% Su_min = u_rate_low_bounds;
% Su_max = u_rate_up_bounds;
% % add the ineq constraints on control rate here
% for i = 1:NU % NU*(N-1), is NU_rate_all
%     args.lbx(NXall+NUall+i:NU:NXall+NUall+NU*(Np-1),1) = u_rate_low_bounds(i);
%     args.ubx(NXall+NUall+i:NU:NXall+NUall+NU*(Np-1),1) = u_rate_up_bounds(i); 
% end

% ==========================================================================================
% SIMULATION 
% ==========================================================================================
% load initial state and control
t0 = 0;        
x_ref = [0 0 0 0 0 0 1 0 0 0 0 0]'; % start with no reference

pos0_n  = Params.Init.pos; 
vel0_n  = Params.Init.vel; 
q0_nb   = Params.Init.quat; 
w0_b    = Params.Init.angvel;
x0 = [pos0_n; vel0_n; q0_nb; w0_b];

if LIVE_3D_VIZ
    update_sim_figure(Params.Hopper, pos0_n, q0_nb, w0_b);
end

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

pause(1);

fsm_state = FSM.ARMED;
x_ref = Params.Guidance.apogee_setpoint;

main_loop = tic;
while(i_mpc < Nsim || i_end ~= Nsim)
    
    state_err = norm(state_error(x0,x_ref),2);
    
    pos_err = norm(x0(1:3)-x_ref(1:3), 2);
    [pos_err_vec, pos_err_moving_avg] = update_moving_avg(pos_err_vec, pos_err);
    q_norm_err = abs(norm( x0(7:10) )-1)
    if state_err > 100 || pos_err > 20 || q_norm_err > 1e-7
        fsm_state = FSM.ABORT;
    end

    switch fsm_state
        case FSM.ARMED
            x_ref = Params.Guidance.apogee_setpoint;
            fsm_state = FSM.ASCENT;

        case FSM.ASCENT
            if pos_err_moving_avg < 0.1
                disp('Reached apogee, setting new reference and entering FSM.LANDING')
                x_ref = Params.Guidance.landing_setpoint;
                fsm_state = FSM.HOVER;
            end

        case FSM.HOVER
            fsm_state = FSM.LANDING;
            
        case FSM.LANDING
            if pos_err_moving_avg < 0.1
                fsm_state = FSM.TOUCHDOWN;
            end

        case FSM.TOUCHDOWN
            % Mission accomplished - Exit simulation loop.
            i_end = i_mpc;
            break;
            
        case FSM.ABORT
            % Exit simulation. 
            % In real experiment we would do go into a safe state here
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
    
    % ==========================================================================================
    % Apply the control and shift the solution using f (dynamics and RK4)
    [t0, x0, U0] = shift(h, t0, x0, u_traj, f);

    
    x_out_mpc(:, i_mpc+2) = x0;
    X0 = reshape(x_traj', NX, Np+1)'; % get solution TRAJECTORY
    
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:); X0(end,:)];
    
    if LIVE_3D_VIZ
        update_sim_figure(Params.Hopper, x0(1:3), x0(7:10), x0(11:13));
    end

    i_mpc
    i_mpc = i_mpc + 1;

end;

main_loop_time = toc(main_loop);
ss_error = state_err
average_mpc_time = main_loop_time/(i_mpc+1)

i_end

% Store data
if strlength(DATA_NAME) ~= 0
    mpc_timestamps         = t(1:i_end)';
    mpc_states             = x_out_mpc(:,1:i_end)';
    mpc_predicted_states   = x_traj_all(:,:,1:i_end);     % (N+1)xNXx(Nsim/h) (11x13x150)
    mpc_controls           = u_opt_all(1:i_end,:);
    mpc_predicted_controls = u_mpcsol_store(:,:,1:i_end); %mpcstore_forces_moments     = u_FM_simout(1:i_end);
    mpc_state_reference    = x_ref_all(:,1:i_end)'; %mpcstore_forces_moments     = u_FM_simout(1:i_end);
    
    fig_folder = DATA_FOLDER;
    name = DATA_NAME;
    figpath = [fig_folder name];
    
    mkdir([fig_folder name]) % creates dir if it does not already exist
    
    save([figpath 'mpc_timestamps'],         'mpc_timestamps')
    save([figpath 'mpc_states'],             'mpc_states')
    save([figpath 'mpc_controls'],           'mpc_controls')
    save([figpath 'mpc_predicted_states'],   'mpc_predicted_states')
    save([figpath 'mpc_predicted_controls'], 'mpc_predicted_controls')
    save([figpath 'mpc_state_reference'],    'mpc_state_reference')

    if PLOT
    plot_mpc(mpc_timestamps, ...
        mpc_states, ...
        mpc_controls, ...
        mpc_state_reference,   ...
        figpath);
    end
else
    disp('No DATA_NAME entered to store and plot.')
end
