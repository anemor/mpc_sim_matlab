function RunOpenloopSim(h, N, SimInitParams, Hopper, storepath, do_plotting)
    
    % Initialize simulation stuff
    t_f = N*h;   % s time horizon total simulation time
    t_simout = 0:h:t_f;
    
    % load constants
    % hopper_properties
    
    % load initial state and control
    % init_conditions
    pos0_n  = SimInitParams.pos; 
    vel0_n  = SimInitParams.vel; 
    q0_nb   = SimInitParams.quat; 
    w0_b    = SimInitParams.angvel;
    x = [pos0_n; vel0_n; q0_nb; w0_b];
    
    % Populate predefined control matrix
    U = SimInitParams.AllControls;
    u = U(:,1);
    
    % Disturbances
    dist_force = SimInitParams.AllDistForce;
    dist_torque = SimInitParams.AllDistTorque;

    % zeros(1, length(t_simout));    %TODO currently scalar 0
    % dist_torque = zeros(1, length(t_simout));   %TODO currently scalar 0
    
    % 3D Visualization
    %update_sim_figure(Hopper, pos0_n, q0_nb, w0_b);
    
    % Simulation with predefined control inputs
    % =========================================================================
    
    print_period = 0.1; %s
    x_simout = zeros(length(t_simout), length(x));
    u_simout = zeros(length(t_simout), length(u));
    u_FM_simout = zeros(length(t_simout), 9);
    i_end = 0;
    stop_height = 0 + Hopper.d_prop; % -0.65 m
    for i = 1:N+1 %1:length(t_simout)	% 0:h:t_f
        
        t = t_simout(i);
    
        % Save state and control
        x_simout(i,:) = x';
        u_simout(i,:) = u';
    
        % print progress
        if mod(t_simout(i), print_period) == 0
            disp(['Progress: ', num2str(t/t_f*100), '%'])
        end
    
        % Update control
        %[F_thrustavg, M_thrustavg, M_thrustdiff] = actuator_dynamics(u);
        
        % =========== Integrate and update state ===========
        % Euler's method
        % x_dot = state_dynamics(x, u, dist_force(i), dist_force(i)); 
        % x = x + h*x_dot;
        % x(7:11) = x(7:11) / norm(x(7:11)); % unit quaternion normalization
        % disp(x);
    
	    % Runge-Kutta 4 (RK4)
        % [k1, F_thrustavg, M_thrustavg, M_thrustdiff] = state_dynamics(x, u, dist_force(i), dist_force(i));
        % [k2, F, M1, M2] = state_dynamics(x + h/2*k1, u, dist_force(i), dist_force(i)); 
        % [k3, F, M1, M2] = state_dynamics(x + h/2*k2, u, dist_force(i), dist_force(i));
        % [k4,F, M1, M2] = state_dynamics(x + h*k3, u,dist_force(i), dist_force(i) );
        % x_next_RK4 = x + h/6*(k1 +2*k2 +2*k3 +k4);     
    
        k1 = state_dynamics( Hopper, x, u, dist_force(:,i), dist_torque(:,i) );
        k2 = state_dynamics( Hopper, x + h/2*k1, u, dist_force(:,i), dist_torque(:,i) ); 
        k3 = state_dynamics( Hopper, x + h/2*k2, u, dist_force(:,i), dist_torque(:,i) );
        k4 = state_dynamics( Hopper, x + h*k3, u, dist_force(:,i), dist_torque(:,i) );
        x = x + h/6*(k1 +2*k2 +2*k3 +k4);     
        x(7:10) = x(7:10) / norm(x(7:10)); % unit quaternion normalization
       
        % Access predefined input vector
        u = U(:,i);
    
        % Visualize current state
        pos_n = x(1:3);
        q_nb = x(7:10);
        w_b = x(11:13);
        %update_sim_figure(Hopper, pos_n, q_nb, w_b);
    
        % Check for negative altitude, stop sim if t > 2s
        if t > 2 && (pos_n(3) +stop_height) >= 0
            i_end = i;
            break;
        elseif t <= 2 && (pos_n(3) +stop_height) >= 0
            x(3) = +stop_height;
            x(6) = 0;
        end
        
    end
    
    % cut off data after the simulation ended
    if i_end ~= 0
        t_simout = t_simout(1:i_end);
        x_simout = x_simout(1:i_end,:);
        u_simout = u_simout(1:i_end,:);
    end
    
    
    % Store data
    sim_timestamps         = t_simout';
    sim_states             = x_simout;
    sim_controls           = u_simout;
        
    mkdir(storepath) % creates dir if it does not already exist
    
    save([storepath 'sim_timestamps'],         'sim_timestamps')
    save([storepath 'sim_states'],             'sim_states')
    save([storepath 'sim_controls'],           'sim_controls')

    if do_plotting
    plot_sim_data(sim_timestamps, ...
        sim_states, ...
        sim_controls, ...
        storepath);
    end
end