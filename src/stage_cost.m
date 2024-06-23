function [cost, Q, R] = stage_cost(x, x_ref, u)

    cost_xy = 100;
    cost_dxy = 1;

    cost_z = 200;
    cost_dz = 10;

    cost_att_roll = 0.01;
    cost_att_pitchyaw = 1; %100;
    cost_angvel_roll = 5;
    cost_angvel_pitchyaw = 50; % 100
  
    % weight on states: Q is NX x NX
    Q = diag([cost_xy, cost_xy, cost_z, ...
            cost_dxy, cost_dxy, cost_dz, ...
            cost_att_roll, cost_att_roll, cost_att_pitchyaw, cost_att_pitchyaw, ...
            cost_angvel_roll, cost_angvel_pitchyaw, cost_angvel_pitchyaw]);
    
    Q_qfix = diag([cost_xy, cost_xy, cost_z, ...
            cost_dxy, cost_dxy, cost_dz, ...
            cost_att_pitchyaw, cost_att_pitchyaw, ...
            cost_angvel_roll, cost_angvel_pitchyaw, cost_angvel_pitchyaw]);

    e_x = [x(1:3) - x_ref(1:3);
        x(4:6) - x_ref(4:6);
        (x(7)*x(8)-x(9)*x(10)) - (x_ref(7)*x_ref(8)-x_ref(9)*x_ref(10));
        (x(7)*x(9)+x(8)*x(10)) - (x_ref(7)*x_ref(9)+x_ref(8)*x_ref(10));
        x(11:13) - x(11:13)];
    
    e_xdiff = [ x(1:3) - x_ref(1:3);
                x(4:6) - x_ref(4:6);
                quat_error(x(7:13), x_ref(7:13));
                x(11:13) - x(11:13)];
    e_xdiff = state_error(x, x_ref);
    % weight controls: R is NU x NU
    cost_linac = 0.1;
    cost_thrustavg = 0.01; % thrust
    cost_thrustdiff = 0.1; % torque


    R = diag([cost_linac, cost_linac, cost_thrustavg, cost_thrustdiff]);

    %cost = x'*Q*x + u'*R*u;

    % calculate objective function
    %cost = (x-x_ref)'*Q*(x-x_ref) + u'*R*u; 
    %cost = e_x'*Q_qfix*e_x + u'*R*u; 
    cost = e_xdiff'*Q*e_xdiff + u'*R*u; 

end