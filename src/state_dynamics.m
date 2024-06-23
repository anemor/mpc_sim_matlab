function [x_dot] = state_dynamics (Hopper, x, u, disturbance_force, disturbance_torque) 

    [F_thrustavg, M_thrustavg, M_thrustdiff] = actuator_dynamics(Hopper, u);
    x_dot = rocket_model(Hopper, x, F_thrustavg, M_thrustavg, M_thrustdiff, disturbance_force, disturbance_torque);
end