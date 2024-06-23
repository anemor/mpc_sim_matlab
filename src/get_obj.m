function [Q,R,Ws] = get_obj()
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
    
	R = diag([cost_linac, cost_linac, cost_thrustavg, cost_thrustdiff]);

	% Slack variables
	Ws = diag([1e3, 1e3, 1e3, 1e3]);
end