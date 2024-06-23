function [A, B] = state_dynamics_linearize(state_dynamics, Hopper, x_eq, u_eq, h)
	import casadi.*
	%global target_apogee_pos_vec_g

    % Define symbolic variables for states and controls
    x = SX.sym('x', 13); % State vector
    u = SX.sym('u', 4);  % Control input vector

    % Compute the Jacobians
    rhs = state_dynamics(Hopper, x, u, zeros(3,1), zeros(3,1)); % State dynamics function
	f = Function('f',{x,u}, {rhs}); % nonlinear mapping function f(x,u)

	st = x; con = u;
	k1 = f(st, con);   
    k2 = f(st + h/2*k1, con); 
    k3 = f(st + h/2*k2, con);
    k4 = f(st + h*k3, con); 
    st_next_RK4=st + h/6*(k1 +2*k2 +2*k3 +k4); 
    st_next_RK4(7:10) = st_next_RK4(7:10)/norm(st_next_RK4(7:10)); % quaternion normalization!


	A_sym = jacobian(st_next_RK4, x); % Jacobian w.r.t. state
    B_sym = jacobian(st_next_RK4, u); % Jacobian w.r.t. control

    % Create a CasADi function for the Jacobians
    A_func = Function('A_func', {x, u}, {A_sym});
    B_func = Function('B_func', {x, u}, {B_sym});

    % Evaluate the Jacobians at the equilibrium point
    A = full(A_func(x_eq, u_eq));
    B = full(B_func(x_eq, u_eq));
end