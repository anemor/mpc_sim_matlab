function [x_dot] = rocket_model_gamma(Hopper, x_b, thrust_prop_vec_b, torque_thrust_force_b, torque_prop_vec_b, dist_force_b, dist_torque_b, gamma)
    %ROCKET_DYNAMICS Calculates generic 6dof dynamics for rocket in inertial frame
    %   

    mass_tot = Hopper.mass_tot;
    J       = Hopper.J_cylinder;
    J_inv   = Hopper.J_cylinder_inv;

    % ================== Extract state vector ==================
    pos_b = [x_b(1:3)];
    vel_linear_b = [x_b(4:6)];
    att_q_nb = x_b(7:10);
    vel_angular_b = [x_b(11:13)];

    g = [0 0 9.81]';
    % ================== Differential equations ==================
    
    % Position derivative = speed
    pos_dot_b = vel_linear_b;
    
    % Linear velicity derivative a = F/m    
    force_tot_b = thrust_prop_vec_b + dist_force_b;
    force_tot_n = v_quatrotate(att_q_nb, force_tot_b) + mass_tot*g;
    vel_linear_dot_b = (1/mass_tot) * force_tot_n;

    % Quaternion kinematics 1/2 * q_bi * w_b_quat
    gamma_q = gamma;
    attitude_quat_dot_bn = Tquat(att_q_nb)*vel_angular_b + (gamma_q/2)*(1 - att_q_nb'*att_q_nb) * att_q_nb;
    
    % Angular velocity derivative, Euler's rotation equations
    torque_tot_b = torque_thrust_force_b + torque_prop_vec_b + dist_torque_b;
    %vel_angular_b = J_inv_g * ( Smtrx(J_g*vel_angular_b)*vel_angular_b + torque_tot_b);

    vel_angular_dot_b = J_inv * (- Smtrx(vel_angular_b)*(J*vel_angular_b) + torque_tot_b); % alternative formulation

    x_dot = [pos_dot_b; vel_linear_dot_b; attitude_quat_dot_bn; vel_angular_dot_b];
end