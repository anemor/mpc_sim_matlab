% Initial state (position, velocity, quaternion, angular velocity)
pos = [1; 2; 0];
vel = [0.1; 0.2; -0.2];
q = [1; 1; 1; 1];
q = q / norm(q); % Normalize the initial quaternion
omega = [0.1; 0.2; 0.3];
x = [pos; vel; q; omega];

% Time step
dt = 0.01;

% Correction factor
F = [0.01 0.02 -0.02]';
Hopper = SetParameters().Hopper;
% Simulate for 100 steps using RK4
for i = 1:100
    % RK4 integration
    k1 = rocket_model_gamma(Hopper,x, F,F,F,F,F);
    k2 = rocket_model_gamma(Hopper,x + dt/2 * k1, F,F,F,F,F);
    k3 = rocket_model_gamma(Hopper,x + dt/2 * k2, F,F,F,F,F);
    k4 = rocket_model_gamma(Hopper,x + dt * k3, F,F,F,F,F);
    x = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
    
    % Extract quaternion part of the state
    q = x(7:10);
    
    % Print the norm of the quaternion
    fprintf('Step %d: Norm = %.15f\n', i, norm(q));
end

