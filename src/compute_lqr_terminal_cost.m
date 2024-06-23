function Qf_lqr = compute_lqr_terminal_cost(Params, x_eq, u_eq, h)

    [A, B] = state_dynamics_linearize( @state_dynamics, Params.Hopper, x_eq, u_eq, h);
    controllability = ctrb(A,B);
    c_rank = rank(controllability)    
    
    %[K,~,~] = dlqr(A,B,Q_tf_small,R);
    %Qf = K;

    % Modify A and B matrices to match the reduced dimension
    A_mod = zeros(11, 11);
    A_mod(1:9, 1:9) = A(1:9, 1:9);      % pos vel
    A_mod(10:11, 1:9) = A(10:11, 1:9);  %   
    A_mod(1:9, 10:11) = A(1:9, 10:11);
    A_mod(10:11, 10:11) = A(10:11, 10:11);
    
    B_mod = zeros(11, 4);
    B_mod(1:9, :) = B(1:9, :);
    B_mod(10:11, :) = B(10:11, :);
    
    % Solve the discrete-time Riccati equation
    [~, S, ~] = dlqr(A_mod, B_mod, Params.MPC.Q_tf_mod, Params.MPC.R);
    
    % Transform S back to the original state space 13x13
    Qf = zeros(13, 13);
    Qf(1:9, 1:9) = S(1:9, 1:9);             % pos, vel
    Qf(10:12, 1:9) = S(10:12, 1:9);          % quat parts
    Qf(1:9, 10:12) = S(1:9, 10:12);          % quat parts
    Qf(10:12, 10:12) = S(10:12, 10:12);      % angvel parts

    %Qf(1:9, 1:9) = S(1:9, 1:9);     % pos, vel
    %Qf(11:12, 1:9) = S(10:11, 1:9); % quat?? 2x1 -> 4x1
    %Qf(1:9, 11:12) = S(1:9, 10:11); % quat?? angvel x??
    %Qf(11:12, 11:12) = S(10:11, 10:11); % angvel yz?
    
    Qf_lqr = Qf;
    % Now Qf is the terminal cost matrix to be used in the MPC

end