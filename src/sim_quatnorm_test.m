% Initial state (position, velocity, quaternion, angular velocity)
pos = [1; 2; 0];
vel = [0.1; 0.2; -0.2];
q = [1; 1; 1; 1];
q = q / norm(q); % Normalize the initial quaternion
omega = [0.1; 0.2; 0.3];
x_initial = [pos; vel; q; omega];

% Correction factor
F = [0.01 0.02 -0.02]';
Hopper = SetParameters().Hopper;

% Define the different parameters for gamma, dt, and N
gamma_values = [0.01,0.1, 0.01, 0.02, 0.03, 0.04];
dt_values = [0.1];
N_values = [10 20 30];

% Initialize cell arrays to store data for plotting
quat_norms_array = cell(length(gamma_values), length(dt_values), length(N_values));
time_arrays = cell(length(gamma_values), length(dt_values), length(N_values));

% Loop over all combinations of gamma, dt, and N
for g = 1:length(gamma_values)
    gamma = gamma_values(g);
    for d = 1:length(dt_values)
        dt = dt_values(d);
        for n = 1:length(N_values)
            N = N_values(n);
            
            % Reset the state to the initial state
            x = x_initial;
            
            % Initialize the quaternion norms array for this simulation
            quat_norms = zeros(N, 1);
            time_array = (0:N-1) * dt;
            
            % Simulate for the defined number of time steps using RK4
            for i = 1:N
                % RK4 integration
                k1 = rocket_model_gamma(Hopper, x, F, F, F, F, F, gamma);
                k2 = rocket_model_gamma(Hopper, x + dt/2 * k1, F, F, F, F, F, gamma);
                k3 = rocket_model_gamma(Hopper, x + dt/2 * k2, F, F, F, F, F, gamma);
                k4 = rocket_model_gamma(Hopper, x + dt * k3, F, F, F, F, F, gamma);
                x = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
                
                % Extract quaternion part of the state
                q = x(7:10);
                
                % Store the norm of the quaternion
                quat_norms(i) = norm(q);
            end
            
            % Store the results in cell arrays
            quat_norms_array{g, d, n} = quat_norms;
            time_arrays{g, d, n} = time_array;
        end
    end
end

% Plot the quaternion norm for each combination
figure(1)
set(gcf, 'Position', [10 100 1000 500]);
tiledlayout(3, 1, 'TileSpacing', 'tight', 'Padding', 'none');

i_tile = 0

for g = 1:length(gamma_values)
    for d = 1:length(dt_values)
        for n = 1:length(N_values)
            i_tile = i_tile + 1;

            nexttile(i_tile)
            hold on
            plot(time_arrays{g, d, n}, quat_norms_array{g, d, n}, 'DisplayName', sprintf('Gamma: %.3f, dt: %.2f, N: %d', gamma_values(g), dt_values(d), N_values(n)))
            xlabel('Time [s]')
            ylabel('Quaternion Norm')
            %set(gca, 'YScale', 'log')  % Set y-axis to logarithmic scale
            grid on
            hold off
            legend
            title(sprintf('Simulations for Tile %d', i))
            
        end
    end
end

set(findall(gcf, 'type', 'line'), 'linewidth', 1)
set(findall(gcf, 'type', 'text'), 'FontSize', 14)
set(findall(gcf, 'type', 'legend'), 'FontSize', 12)

saveas(gcf, 'quat_norm_plot.png')