classdef casadi_block_rocket < matlab.System & matlab.system.mixin.Propagates
    % untitled2 Direct multiple shooting  
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.
    % Based on this example: https://github.com/casadi/casadi/blob/3.1.0/docs/examples/matlab/direct_multiple_shooting.m
    properties
        % Public, tunable properties.

    end

    properties (DiscreteState)
    end

    % Pre-computed constants
    properties (Access = private)
        casadi_solver
        w0
        lbw
        ubw
        lbg
        ubg
    end

    methods (Access = protected)
        function num = getNumInputsImpl(~)
            num = 2;
        end
        function num = getNumOutputsImpl(~)
            num = 1;
        end
        function dt1 = getOutputDataTypeImpl(~)
        	dt1 = 'double';
        end
        function dt1 = getInputDataTypeImpl(~)
        	dt1 = 'double';
        end
        function sz1 = getOutputSizeImpl(~)
        	sz1 = [4,1];
        end
        function sz1 = getInputSizeImpl(~)
        	sz1 = {[1,1], [13,1]};
        end
        function cp1 = isInputComplexImpl(~)
        	cp1 = false;
        end
        function cp1 = isOutputComplexImpl(~)
        	cp1 = false;
        end
        function fz1 = isInputFixedSizeImpl(~)
        	fz1 = true;
        end
        function fz1 = isOutputFixedSizeImpl(~)
        	fz1 = true;
        end
        function setupImpl(obj,~,~)
            % Implement tasks that need to be performed only once, 
            % such as pre-computed constants.
            
            MULTIPLE_SHOOTING = 1;

            T = 10; % Time horizon for the optimization
            N = 5; % number of control intervals

            NX = 13;
            NU = 4;
            % Symbolic declaration of model parameters x and u
            x = casadi.SX.sym('x', NX); % p, v, q, w
            u = casadi.SX.sym('u', NU); % pitch, yaw, speed_avg, speed_diff
            
            % Model equations
            x_dot = state_dynamics_f(x,u, zeros(3,1), zeros(3,1));


            % Objective function
            L = objective_function(x,u);

            % Formulate discrete time dynamics
            % Fixed step Runge-Kutta 4 integrator

            % CasADi Funtion for continuous time dynamics and objective
            f = casadi.Function('f', {x,u}, {x_dot, L}, {'x', 'u'}, {'x_dot', 'L'}); % nonlinear mapping function

            M = 4; % RK4 steps per interval
            DT = T/N/M;

            X0 = casadi.MX.sym('X0', NX);   % the states over the optimization problem
            X = X0; % Controls / decision variables
            U = casadi.MX.sym('U', NU);

            Q = 0; % Objective function accumulator
            for j=1:M
               [k1, k1_q] = f(X, U);
               [k2, k2_q] = f(X + DT/2 * k1, U);
               [k3, k3_q] = f(X + DT/2 * k2, U);
               [k4, k4_q] = f(X + DT * k3, U);
               X=X+DT/6*(k1 +2*k2 +2*k3 +k4); % next state
               Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
            end
            F = casadi.Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});

            % Start with an empty NLP
            % Decision variables, initialized with X0 now and U0 later
            w={};
            w0 = [];
            lbw = [];
            ubw = [];

            x0 = [0 0 10 0 0 0 1 0 0 0 0 0 0]';  % Initialize w with guess for the state
            u0 = [0.1 0.1 0.1 0]';  % Initial guess for the control, is added later in the loop
            X0 = casadi.MX.sym('X0', NX); % only used in multiple shooting

            if MULTIPLE_SHOOTING    
                w = {w{:}, X0};

                w0 = x0;

                % Initialize lower and upper bounds for state
                lbw = -inf(NX, 1); 
                ubw = inf(NX, 1);  
            end

            % Initialize the objective and constraints
            J = 0;
            g={};
            lbg = [];
            ubg = [];
            
            % Set constraints for state and control
            % [lbw_k, ubw_k] = set_constraints(lbw, ubw, k);  % Set constraints for state and control
            [lbx_k, ubx_k, lbu_k, ubu_k] = get_constraints();
            % Formulate the NLP
            Xk = X0;
            for k = 0 : N-1
                % New NLP control variables for this interval
                Uk = casadi.MX.sym(['U_' num2str(k)], NU);
                w = {w{:}, Uk};
                lbw = [lbw; lbu_k];  % Append lower bounds for U
                ubw = [ubw; ubu_k];  % Append upper bounds for U
                w0 = [w0; u0];  % Append initial guesses for U

                % ---------------------- Next interval ----------------------
                % Integrate till the end of the interval using dynamics
                Fk = F('x0', Xk, 'p', Uk);
                Xk_end = Fk.xf;
                J = J + Fk.qf;
                
                if MULTIPLE_SHOOTING
                    % New NLP state variable for next loop
                    Xk = casadi.MX.sym(['X_' num2str(k+1)], NX);
                    w = {w{:}, Xk};
                    lbw = [lbw; lbx_k];
                    ubw = [ubw; ubx_k];
                    w0 = [w0; x0];
                end
                % Add equality constraints (= bounds are zero, predicted and actual state should be equal)
                g = {g{:}, Xk_end-Xk}; % predicted state at end of interval
                lbg = [lbg; zeros(NX,1)];
                ubg = [ubg; zeros(NX,1)];

            end    

            % Create an NLP solver
            prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
            options = struct('ipopt',struct('print_level',1),'print_time',false);
            solver = casadi.nlpsol('solver', 'ipopt', prob, options);

            obj.casadi_solver = solver;
            obj.w0 = w0;
            obj.lbw = lbw;
            obj.ubw = ubw;
            obj.lbg = lbg;
            obj.ubg = ubg;
        end

        function u = stepImpl(obj,x,t)
            arguments
                obj matlab.System
                x (13,1) double
                t double
            end

            disp(t)
            tic
            w0 = obj.w0;
            lbw = obj.lbw;
            ubw = obj.ubw;
            solver = obj.casadi_solver;
            % Set current state as the initial condition
            lbw(1:13) = x;
            ubw(1:13) = x;

            % Solve optimization problem, extracting u from solution
            sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                        'lbg', obj.lbg, 'ubg', obj.ubg);
            % only first element of input sequence is used
            u = full(sol.x(14:17)); 
            toc
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end

function [cost] = objective_function(x,u)

    cost_xy = 20;
    cost_dxy = 10;

    cost_z = 2;
    cost_dz = 1;

    cost_att = 100;
    cost_angvel_roll = 0.0004;
    cost_angvel_pitchyaw = 1;


    cost_linac = 0.01;
    cost_thrustavg = 0.005; % thrust
    cost_thrustdiff = 0.001; % torque
  
    % weights
    Q = diag([cost_xy, cost_xy, cost_z, ...
            cost_dxy, cost_dxy, cost_dz, ...
            cost_att, cost_att, cost_att, cost_att, ...
            cost_angvel_roll, cost_angvel_pitchyaw, cost_angvel_pitchyaw]);
    R = diag([cost_linac, cost_linac, cost_thrustavg, cost_thrustdiff]);
    
    cost = x'*Q*x + u'*R*u;
end

function [lbx, ubx, lbu, ubu] = get_constraints()

    eps = 1e-3;
    % max_linac_rate = deg2rad(100); %deg/s
    max_linac_angle = deg2rad(10); %deg
    % these are from tvcdrone
    min_prop_speed = 50;
    max_prop_speed = 80;
    max_prop_diff = 40; 

    % Control lower and upper bound
    lbu = [-max_linac_angle; -max_linac_angle; -min_prop_speed; -max_prop_diff/2];
    ubu = [max_linac_angle; max_linac_angle; max_prop_speed; max_prop_diff/2];

    % State lower and upper bound
    min_z = 0;
    max_dx = 1.2;
    max_dz = 1.5;
    max_d_angvel = 0.6;

    lbx = [-inf;-inf; min_z-eps
            -max_dx; -max_dx; -max_dz
            -inf(4,1) 
            -max_d_angvel; -max_d_angvel; -inf]; %-inf(3,1)];
    ubx = [inf(3,1)
            max_dx; max_dx; max_dz
            inf(4,1)
            max_d_angvel; max_d_angvel; inf];
end
