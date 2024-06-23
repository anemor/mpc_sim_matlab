function plot_mpc_data_multiple(figpath)
    % Define constants and initialize arrays
    n_tot = 81; % 3*3*3*3
    all_data = struct();

    % Using cell arrays because the data is not the same length
    all_data.t = cell(n_tot, 1);
    all_data.x = cell(n_tot, 1);
    all_data.y = cell(n_tot, 1);
    all_data.z = cell(n_tot, 1);
    all_data.u = cell(n_tot, 1);
    all_data.v = cell(n_tot, 1);
    all_data.w = cell(n_tot, 1);
    all_data.qw = cell(n_tot, 1);
    all_data.qx = cell(n_tot, 1);
    all_data.qy = cell(n_tot, 1);
    all_data.qz = cell(n_tot, 1);
    all_data.p = cell(n_tot, 1);
    all_data.q = cell(n_tot, 1);
    all_data.r = cell(n_tot, 1);
    all_data.xr = cell(n_tot, 1);
    all_data.yr = cell(n_tot, 1);
    all_data.zr = cell(n_tot, 1);
    all_data.ur = cell(n_tot, 1);
    all_data.vr = cell(n_tot, 1);
    all_data.wr = cell(n_tot, 1);
    all_data.qwr = cell(n_tot, 1);
    all_data.qxr = cell(n_tot, 1);
    all_data.qyr = cell(n_tot, 1);
    all_data.qzr = cell(n_tot, 1);
    all_data.pr = cell(n_tot, 1);
    all_data.qr = cell(n_tot, 1);
    all_data.rr = cell(n_tot, 1);
    all_data.linac_pitch = cell(n_tot, 1);
    all_data.linac_yaw = cell(n_tot, 1);
    all_data.prop_speed_avg = cell(n_tot, 1);
    all_data.prop_speed_diff = cell(n_tot, 1);
    
    % Load all stored data from folders
    for i = 1:n_tot
        fignr_path = [figpath '/nr_' num2str(i) '/'];
        load([fignr_path 'mpc_timestamps'], 'mpc_timestamps');
        load([fignr_path 'mpc_states'], 'mpc_states');
        load([fignr_path 'mpc_controls'], 'mpc_controls');
        %load([fignr_path 'mpc_predicted_states'], 'mpc_predicted_states');
        %load([fignr_path 'mpc_predicted_controls'], 'mpc_predicted_controls');
        load([fignr_path 'mpc_state_reference'], 'mpc_state_reference');
        
        % Store data in cell arrays
        all_data.t{i} = mpc_timestamps;
        all_data.x{i} = mpc_states(:, 1);
        all_data.y{i} = mpc_states(:, 2);
        all_data.z{i} = mpc_states(:, 3);
        all_data.u{i} = mpc_states(:, 4);
        all_data.v{i} = mpc_states(:, 5);
        all_data.w{i} = mpc_states(:, 6);
        all_data.qw{i} = mpc_states(:, 7);
        all_data.qx{i} = mpc_states(:, 8);
        all_data.qy{i} = mpc_states(:, 9);
        all_data.qz{i} = mpc_states(:, 10);
        all_data.p{i} = mpc_states(:, 11);
        all_data.q{i} = mpc_states(:, 12);
        all_data.r{i} = mpc_states(:, 13);
        all_data.xr{i} = mpc_state_reference(:, 1);
        all_data.yr{i} = mpc_state_reference(:, 2);
        all_data.zr{i} = mpc_state_reference(:, 3);
        all_data.ur{i} = mpc_state_reference(:, 4);
        all_data.vr{i} = mpc_state_reference(:, 5);
        all_data.wr{i} = mpc_state_reference(:, 6);
        all_data.qwr{i} = mpc_state_reference(:, 7);
        all_data.qxr{i} = mpc_state_reference(:, 8);
        all_data.qyr{i} = mpc_state_reference(:, 9);
        all_data.qzr{i} = mpc_state_reference(:, 10);
        all_data.pr{i} = mpc_state_reference(:, 11);
        all_data.qr{i} = mpc_state_reference(:, 12);
        all_data.rr{i} = mpc_state_reference(:, 13);
        all_data.linac_pitch{i} = mpc_controls(:, 1);
        all_data.linac_yaw{i} = mpc_controls(:, 2);
        all_data.prop_speed_avg{i} = mpc_controls(:, 3);
        all_data.prop_speed_diff{i} = mpc_controls(:, 4);
    end

    % Plot it all babyy
    %avg_rmse_all = rmse_calculate(all_data, n_tot);
    %plot_rmse_all(avg_rmse_all, figpath);
    plot_pos_err_gradient_rmse(all_data, figpath, n_tot);
    %plot_pos_err(all_data, figpath, n_tot);
    plot_pos_vel_enu(all_data, figpath, n_tot);
    plot_quat_angvel(all_data, figpath);
    plot_prop(all_data, figpath);
end

function plot_pos_vel_enu(data, figpath, n_data)
    % Rotate and plot positions and velocities to ENU frame which is a wayy 
    % cuter way to visualize it
    R_un = Rquat(euler2q(pi, 0, pi/2));  % from NED to R=eye(1)
    
    pos_u = R_un * [data.x, data.y, data.z]';
    pos_ref_u = R_un * [data.xr, data.yr, data.zr]';
    x_u = pos_u(1, :)';     % Extract new x coordinates
    y_u = pos_u(2, :)';     % Extract new y coordinates
    z_u = pos_u(3, :)';     % Extract new z coordinates
    xr_u = pos_ref_u(1, :)'; % Extract new x coordinates
    yr_u = pos_ref_u(2, :)'; % Extract new y coordinates
    zr_u = pos_ref_u(3, :)'; % Extract new z coordinates

    vel_u = R_un * [data.u, data.v, data.w]';
    vel_ref_u = R_un * [data.ur, data.vr, data.wr]';
    u_u = vel_u(1, :)';
    v_u = vel_u(2, :)';
    w_u = vel_u(3, :)'; 
    ur_u = vel_ref_u(1, :)'; 
    vr_u = vel_ref_u(2, :)'; 
    wr_u = vel_ref_u(3, :)';

    % Determine limits
    x_min = min([-2, min(x_u)]);
    x_max = max([2, max(x_u)]);
    y_min = min([-2, min(y_u)]);
    y_max = max([2, max(y_u)]);
    z_min = 0;
    z_max = max(z_u);

    figure(1)
    set(gcf, 'Position', [10 100 1000 800]);
    tiledlayout(3,3,'TileSpacing','tight','Padding','none');
    
    nexttile(1, [3 1]), plot3(x_u,y_u,z_u, '-'), hold on
    plot3(xr_u,yr_u,zr_u,'*'), hold off
    xlabel('x East'), ylabel('y North'), zlabel('z Up')
    xlim([x_min x_max]), ylim([y_min y_max]), zlim([z_min z_max])
    view([-23,12]);
    legend('pos','ref', 'Location', 'northeast')
    title('3D Position [m]')
    grid
    
    % 2D Plots for x, y, z positions and velocities
    nexttile(2, [1 2]), plot(data.t,xr_u, ':'), hold on
    plot(data.t,ur_u, ':'), plot(data.t,u_u), plot(data.t,x_u)
    legend('p_r [m]', 'v_r [m/s]', 'v [m/s]', 'p [m]', 'Location', 'northeast')
    title('East [m]'), grid
    
    nexttile(5, [1 2]), plot(data.t,yr_u, ':'), hold on
    plot(data.t,vr_u, ':'), plot(data.t,v_u), plot(data.t,y_u)
    title('North [m]'), grid
    
    nexttile(8, [1 2]), plot(data.t,zr_u, ':'), hold on
    plot(data.t,wr_u, ':'), plot(data.t,w_u), plot(data.t,z_u)
    xlabel('time [s]'), title('Up [m]'), grid
    
    saveas(gcf, [figpath 'pos_vel_enu'])
    saveas(gcf, [figpath 'pos_vel_enu.png'])
end

function plot_pos_err(data, figpath, n_data)
    % Calculate position errors
    R_un = Rquat(euler2q(pi, 0, pi/2));  % from NED to R=eye(1)
    avg_errors = zeros(n_data, 1);  % To store the average error norms

    figure(4)
    set(gcf, 'Position', [10 100 1200 800]);
    tiledlayout(3,1,'TileSpacing','tight','Padding','none');

    % X position error
    nexttile, hold on
    for i = 1:n_data
        pos_u = R_un * [data.x{i}, data.y{i}, data.z{i}]';
        pos_ref_u = R_un * [data.xr{i}, data.yr{i}, data.zr{i}]';
        x_u = pos_u(1, :)'; % Extract new x coordinates
        xr_u = pos_ref_u(1, :)'; % Extract new x coordinates
        x_err = xr_u - x_u;
        plot(data.t{i}, x_err)

        % Calculate error norms for each run
        y_u = pos_u(2, :)'; % Extract new y coordinates
        yr_u = pos_ref_u(2, :)'; % Extract new y coordinates
        y_err = yr_u - y_u;
        z_u = pos_u(3, :)'; % Extract new z coordinates
        zr_u = pos_ref_u(3, :)'; % Extract new z coordinates
        z_err = zr_u - z_u;

        % Compute the average error norm for this run
        error_norms = sqrt(x_err.^2 + y_err.^2 + z_err.^2);
        avg_errors(i) = mean(error_norms);
    end

    legend(arrayfun(@(i) [num2str(i)], 1:n_data, 'UniformOutput', false), 'NumColumns', 9) % Create legend entries as 'nr1', 'nr2', ..., 'nrn'
    xlabel('Time [s]'), ylabel('Error [m]')
    title('X Position Error'), grid
    
    % Y position error
    nexttile, hold on
    for i = 1:n_data
        pos_u = R_un * [data.x{i}, data.y{i}, data.z{i}]';
        pos_ref_u = R_un * [data.xr{i}, data.yr{i}, data.zr{i}]';
        y_u = pos_u(2, :)'; % Extract new y coordinates
        yr_u = pos_ref_u(2, :)'; % Extract new y coordinates
        y_err = yr_u - y_u;
        plot(data.t{i}, y_err)
    end
    legend(arrayfun(@(i) ['nr', num2str(i)], 1:n_data, 'UniformOutput', false)) % Create legend entries as 'nr1', 'nr2', ..., 'nrn'
    xlabel('Time [s]'), ylabel('Error [m]')
    title('Y Position Error'), grid
    
    % Z position error
    nexttile, hold on
    for i = 1:n_data
        pos_u = R_un * [data.x{i}, data.y{i}, data.z{i}]';
        pos_ref_u = R_un * [data.xr{i}, data.yr{i}, data.zr{i}]';
        z_u = pos_u(3, :)'; % Extract new z coordinates
        zr_u = pos_ref_u(3, :)'; % Extract new z coordinates
        z_err = zr_u - z_u;
        plot(data.t{i}, z_err)
    end
    %legend(arrayfun(@(i) ['nr', num2str(i)], 1:n_data, 'UniformOutput', false)) % Create legend entries as 'nr1', 'nr2', ..., 'nrn'
    xlabel('Time [s]'), ylabel('Error [m]')
    title('Z Position Error'), grid
    
    saveas(gcf, [figpath 'pos_err'])
    saveas(gcf, [figpath 'pos_err.png'])
end

function plot_pos_err_gradient(data, figpath, n_data)
    % Calculate position errors and average error norms
    R_un = Rquat(euler2q(pi, 0, pi/2));  % from NED to R=eye(1)
    avg_errors = zeros(n_data, 1);  % To store the average error norms
    errors = struct('x_err', cell(n_data, 1), 'y_err', cell(n_data, 1), 'z_err', cell(n_data, 1));

    % Calculate errors and average error norms
    for i = 1:n_data
        pos_u = R_un * [data.x{i}, data.y{i}, data.z{i}]';
        pos_ref_u = R_un * [data.xr{i}, data.yr{i}, data.zr{i}]';
        x_u = pos_u(1, :)'; % Extract new x coordinates
        xr_u = pos_ref_u(1, :)'; % Extract new x coordinates
        errors(i).x_err = xr_u - x_u;

        y_u = pos_u(2, :)'; % Extract new y coordinates
        yr_u = pos_ref_u(2, :)'; % Extract new y coordinates
        errors(i).y_err = yr_u - y_u;

        z_u = pos_u(3, :)'; % Extract new z coordinates
        zr_u = pos_ref_u(3, :)'; % Extract new z coordinates
        errors(i).z_err = zr_u - z_u;

        % Compute the average error norm for this run
        error_norms = sqrt(errors(i).x_err.^2 + errors(i).y_err.^2 + 0.1*errors(i).z_err.^2);
        avg_errors(i) = mean(error_norms);
    end

    % Sort the errors and average errors
    [sorted_errors, sort_idx] = sort(avg_errors);

    % Generate color gradient from blue to red
    cmap = jet(n_data);

    figure(4)
    set(gcf, 'Position', [10 100 1200 800]);
    tiledlayout(3,1,'TileSpacing','tight','Padding','none');

    % X position error
    nexttile, hold on
    for i = 1:n_data
        idx = sort_idx(i);
        plot(data.t{idx}, errors(idx).x_err, 'Color', cmap(i, :))
    end
    legend(arrayfun(@(i) ['nr', num2str(sort_idx(i))], 1:n_data, 'UniformOutput', false), 'NumColumns', 9) % Create legend entries as 'nr1', 'nr2', ..., 'nrn'
    xlabel('Time [s]'), ylabel('Error [m]')
    xlim([0 30])
    title('X Position Error'), grid

    % Y position error
    nexttile, hold on
    for i = 1:n_data
        idx = sort_idx(i);
        plot(data.t{idx}, errors(idx).y_err, 'Color', cmap(i, :))
    end
    %legend(arrayfun(@(i) ['nr', num2str(sort_idx(i))], 1:n_data, 'UniformOutput', false), 'NumColumns', 9) % Create legend entries as 'nr1', 'nr2', ..., 'nrn'
    xlim([0 30])
    xlabel('Time [s]'), ylabel('Error [m]')
    title('Y Position Error'), grid

    % Z position error
    nexttile, hold on
    for i = 1:n_data
        idx = sort_idx(i);
        plot(data.t{idx}, errors(idx).z_err, 'Color', cmap(i, :))
    end
    %legend(arrayfun(@(i) ['nr', num2str(sort_idx(i))], 1:n_data, 'UniformOutput', false), 'NumColumns', 9) % Create legend entries as 'nr1', 'nr2', ..., 'nrn'
    xlabel('Time [s]'), ylabel('Error [m]')
    xlim([0 30])
    title('Z Position Error'), grid

    % Find the run with the smallest average error norm
    [min_avg_error, min_run_idx] = min(avg_errors);
    fprintf('Run with smallest average error: nr%d with average error: %.4f\n', min_run_idx, min_avg_error);

    saveas(gcf, [figpath 'pos_err'])
    saveas(gcf, [figpath 'pos_err.png'])
end


function plot_quat_angvel(data, figpath, n_data)
    % Quaternion and angular velocity plot
    quat_lim = [-1.1 1.1];
    
    figure(3)
    set(gcf, 'Position', [10 100 1000 400]);
    tiledlayout(4,4,'TileSpacing','tight','Padding','none');
    
    nexttile(1, [1,2]), plot(data.t, data.qwr, ':'), hold on
    plot(data.t, data.qw), hold off
    title('Quaternion Body to NED'), ylabel('q_w'), ylim(quat_lim), grid
    
    nexttile(5, [1,2]), plot(data.t, data.qxr, ':'), hold on
    plot(data.t, data.qx), hold off
    ylabel('q_x'), ylim(quat_lim), grid
    
    nexttile(9, [1,2]), plot(data.t, data.qyr, ':'), hold on
    plot(data.t, data.qy), hold off
    ylabel('q_y'), ylim(quat_lim), grid
    
    nexttile(13, [1,2]), plot(data.t, data.qzr, ':'), hold on
    plot(data.t, data.qz), hold off
    ylabel('q_z'), xlabel('time [s]'), ylim(quat_lim), grid
    
    nexttile(3, [1,2]), plot(data.t, rad2deg(data.pr), ':'), hold on
    plot(data.t, rad2deg(data.p)), hold off
    ylabel('\omega_x^b [deg/s]'), grid
    title('Angular velocities in Body')
    
    nexttile(7, [1,2]), plot(data.t, rad2deg(data.qr), ':'), hold on
    plot(data.t, rad2deg(data.q)), hold off
    ylabel('\omega_y^b [deg/s]'), grid
    
    nexttile(11, [1,2]), plot(data.t, rad2deg(data.rr), ':'), hold on
    plot(data.t, rad2deg(data.r)), hold off
    ylabel('\omega_z^b [deg/s]'), xlabel('time [s]'), grid

    saveas(gcf, [figpath 'quat_angvel'])
    saveas(gcf, [figpath 'quat_angvel.png'])
end

function plot_pos_err_gradient_rmse(data, figpath, n_data)
    % Calculate position errors and average RMSE
    R_un = Rquat(euler2q(pi, 0, pi/2));  % from NED to R=eye(1)
    avg_rmse = zeros(n_data, 1);  % To store the average RMSE
    errors = struct('x_err', cell(n_data, 1), 'y_err', cell(n_data, 1), 'z_err', cell(n_data, 1));

    % Calculate errors and RMSE
    for i = 1:n_data
        pos_u = R_un * [data.x{i}, data.y{i}, data.z{i}]';
        pos_ref_u = R_un * [data.xr{i}, data.yr{i}, data.zr{i}]';
        x_u = pos_u(1, :)'; % Extract new x coordinates
        xr_u = pos_ref_u(1, :)'; % Extract new x coordinates
        errors(i).x_err = xr_u - x_u;

        y_u = pos_u(2, :)'; % Extract new y coordinates
        yr_u = pos_ref_u(2, :)'; % Extract new y coordinates
        errors(i).y_err = yr_u - y_u;

        z_u = pos_u(3, :)'; % Extract new z coordinates
        zr_u = pos_ref_u(3, :)'; % Extract new z coordinates
        errors(i).z_err = zr_u - z_u;

        % Compute the RMSE for each dimension
        rmse_x = sqrt(mean(errors(i).x_err.^2));
        rmse_y = sqrt(mean(errors(i).y_err.^2));
        rmse_z = sqrt(mean(errors(i).z_err.^2));
        
        % Compute the average RMSE for this run
        avg_rmse(i) = mean([rmse_x, rmse_y, rmse_z]);


    end

    % Sort the errors and average RMSE
    [sorted_rmse, sort_idx] = sort(avg_rmse);
    
    plot_errors(data.t, errors, sorted_rmse, sort_idx, avg_rmse, n_data, figpath)

end

function plot_errors(t, errors, sorted_rmse, sort_idx, avg_rmse, n_data, figpath)

    figure(4)
    set(gcf, 'Position', [10 100 1200 800]);
    tiledlayout(4,1,'TileSpacing','tight','Padding','none');
    
    % Generate color gradient from blue to red
    cmap = winter(n_data);

    xaxis_max = 30; % s
    % X position error
    nexttile, hold on
    for i = 1:n_data
        idx = sort_idx(i);
        plot(t{idx}, errors(idx).x_err, ':','Color', cmap(i, :))
    end
    xlim([0, xaxis_max])
    legend(arrayfun(@(i) ['nr', num2str(sort_idx(i))], 1:n_data, 'UniformOutput', false), 'NumColumns', 9) % Create legend entries as 'nr1', 'nr2', ..., 'nrn'
    xlabel('Time [s]'), ylabel('Error [m]')
    title('X Position Error'), grid

    % Y position error
    nexttile, hold on
    for i = 1:n_data
        idx = sort_idx(i);
        plot(t{idx}, errors(idx).y_err, ':','Color', cmap(i, :))
    end
    xlim([0, xaxis_max])
    legend(arrayfun(@(i) ['nr', num2str(sort_idx(i))], 1:n_data, 'UniformOutput', false), 'NumColumns', 9) % Create legend entries as 'nr1', 'nr2', ..., 'nrn'
    xlabel('Time [s]'), ylabel('Error [m]')
    title('Y Position Error'), grid

    % Z position error
    nexttile, hold on
    for i = 1:n_data
        idx = sort_idx(i);
        plot(t{idx}, errors(idx).z_err, ':','Color', cmap(i, :))
    end
    xlim([0, xaxis_max])
    legend(arrayfun(@(i) ['nr', num2str(sort_idx(i))], 1:n_data, 'UniformOutput', false), 'NumColumns', 9) % Create legend entries as 'nr1', 'nr2', ..., 'nrn'
    xlabel('Time [s]'), ylabel('Error [m]')
    title('Z Position Error'), grid

    % RMSE Bar Chart
    nexttile, hold on
    bar_handle = bar(avg_rmse, 'FaceColor', 'flat');
    colormap(cmap)
    for k = 1:length(avg_rmse)
        bar_handle.CData(k,:) = cmap(k, :);
    end
    xlabel('Run Index')
    ylabel('Average RMSE')
    title('Average RMSE for Each Run')
    xticks(1:length(avg_rmse))
    xticklabels(1:length(avg_rmse))
    grid on

    % Find the run with the smallest average RMSE
    [min_avg_rmse, min_run_idx] = min(avg_rmse);
    fprintf('Run with smallest average RMSE: nr%d with average RMSE: %.4f\n', min_run_idx, min_avg_rmse);

    saveas(gcf, [figpath 'pos_err'])
    saveas(gcf, [figpath 'pos_err.png'])
end


function plot_prop(data, figpath, n_data)
    % Propeller plots
    figure(4)
    set(gcf, 'Position', [10 100 1000 300]);
    tiledlayout(2,1,'TileSpacing','tight','Padding','none');
    
    nexttile, plot(data.t, data.prop_speed_avg, ':'), hold on
    plot(data.t, data.prop_speed_diff), hold off
    title('Propeller speed'), grid
    legend('P_{avg}', 'P_{diff}')
    
    nexttile, plot(data.t, rad2deg(data.linac_pitch), ':'), hold on
    plot(data.t, rad2deg(data.linac_yaw)), hold off
    title('Linear actuators angle (deg)'), xlabel('time [s]'), grid
    legend('\delta_{pitch}', '\delta_{yaw}')
    
    saveas(gcf, [figpath 'prop'])
    saveas(gcf, [figpath 'prop.png'])
end

function avg_rmse_all = rmse_calculate(data, n_data)
    % Calculate RMSE for all data including quaternions and angular velocities
    avg_rmse_all = zeros(n_data, 1);  % To store the average RMSE

    for i = 1:n_data
        % Calculate position errors
        R_un = Rquat(euler2q(pi, 0, pi/2));  % from NED to R=eye(1)
        pos_u = R_un * [data.x{i}, data.y{i}, data.z{i}]';
        pos_ref_u = R_un * [data.xr{i}, data.yr{i}, data.zr{i}]';
        x_u = pos_u(1, :)'; % Extract new x coordinates
        xr_u = pos_ref_u(1, :)'; % Extract new x coordinates
        x_err = xr_u - x_u;

        y_u = pos_u(2, :)'; % Extract new y coordinates
        yr_u = pos_ref_u(2, :)'; % Extract new y coordinates
        y_err = yr_u - y_u;

        z_u = pos_u(3, :)'; % Extract new z coordinates
        zr_u = pos_ref_u(3, :)'; % Extract new z coordinates
        z_err = zr_u - z_u;

        % Compute the RMSE for each dimension
        rmse_x = sqrt(mean(x_err.^2));
        rmse_y = sqrt(mean(y_err.^2));
        rmse_z = sqrt(mean(z_err.^2));
        avg_rmse_pos = mean([rmse_x, rmse_y, rmse_z]);

        % Calculate quaternion errors
        q_err = sqrt((data.qw{i} - data.qwr{i}).^2 + ...
                     (data.qx{i} - data.qxr{i}).^2 + ...
                     (data.qy{i} - data.qyr{i}).^2 + ...
                     (data.qz{i} - data.qzr{i}).^2);
        rmse_q = sqrt(mean(q_err.^2));

        % Calculate angular velocity errors
        p_err = rad2deg(data.pr{i} - data.p{i});
        q_err = rad2deg(data.qr{i} - data.q{i});
        r_err = rad2deg(data.rr{i} - data.r{i});
        rmse_p = sqrt(mean(p_err.^2));
        rmse_q = sqrt(mean(q_err.^2));
        rmse_r = sqrt(mean(r_err.^2));
        avg_rmse_angvel = mean([rmse_p, rmse_q, rmse_r]);

        % Compute the overall average RMSE
        avg_rmse_all(i) = mean([avg_rmse_pos, rmse_q, avg_rmse_angvel]);
    end
end

function plot_rmse_all(avg_rmse_all, figpath)
    % Generate color gradient from blue to red
    cmap = jet(length(avg_rmse_all));

    % Plot RMSE Bar Chart
    figure
    set(gcf, 'Position', [10 100 1200 600]);
    bar_handle = bar(avg_rmse_all, 'FaceColor', 'flat');
    colormap(cmap)
    for k = 1:length(avg_rmse_all)
        bar_handle.CData(k,:) = cmap(k, :);
    end
    xlabel('Run Index')
    ylabel('Average RMSE')
    title('Average RMSE for Each Run')
    xticks(1:length(avg_rmse_all))
    xticklabels(1:length(avg_rmse_all))
    grid on

    % Find the run with the smallest average RMSE
    [min_avg_rmse, min_run_idx] = min(avg_rmse_all);
    fprintf('Run with smallest average RMSE: nr%d with average RMSE: %.4f\n', min_run_idx, min_avg_rmse);

    saveas(gcf, [figpath 'rmse_all'])
    saveas(gcf, [figpath 'rmse_all.png'])
end
