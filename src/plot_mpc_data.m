
% Extract States
function plot_mpc_data (t_simout, x_simout, u_simout, xref_simout, figpath)

    % Select colour for plots <3
    C_PINE          = '#387C44';
    C_SLIME         = '#BCE954';
    C_CLOVER        = "#3EA055";
    C_COOLGREEN     = "#44916F";
    C_BLUE          = "#80B9C8";
    C_ORANGE        = "#FF8B60";
    C_YELLOW        = "#FFF394";
    C_TURQUOISE     = "#18FFBF";
    C_BLUE_LIGHT    = "#97D3CB";

    col         = C_COOLGREEN;        
    col_ref     = C_BLUE; 
    col_extra   = C_SLIME;
    
    c_other     = C_ORANGE;
    c_other_r   = C_BLUE_LIGHT;
    % Select figures to plot
    fig1_prop       = true;    
    fig3_quat_angvel = true; 
    fig5_posvel_enu = true;

    % Extract data
    t   = t_simout;
    x   = x_simout(:,1);  % Position x
    y   = x_simout(:,2);  % Position y
    z   = x_simout(:,3);  % Position z
    u   = x_simout(:,4);  % Velocity u
    v   = x_simout(:,5);  % Velocity v
    w   = x_simout(:,6);  % Velocity w
    qw  = x_simout(:,7);  % Quaternion component w
    qx  = x_simout(:,8);  % Quaternion component x
    qy  = x_simout(:,9);  % Quaternion component y
    qz  = x_simout(:,10); % Quaternion component z
    p   = x_simout(:,11); % Angular velocity p
    q   = x_simout(:,12); % Angular velocity q
    r   = x_simout(:,13); % Angular velocity r

    % Reference signal
    xr = xref_simout(:,1);  % Position x
    yr = xref_simout(:,2);  % Position y
    zr = xref_simout(:,3);  % Position z
    ur = xref_simout(:,4);  % Velocity u
    vr = xref_simout(:,5);  % Velocity v
    wr = xref_simout(:,6);  % Velocity w
    qwr = xref_simout(:,7);  % Quaternion component w
    qxr = xref_simout(:,8);  % Quaternion component x
    qyr = xref_simout(:,9);  % Quaternion component y
    qzr = xref_simout(:,10); % Quaternion component z
    pr = xref_simout(:,11); % Angular velocity p
    qr = xref_simout(:,12); % Angular velocity q
    rr = xref_simout(:,13); % Angular velocity r


    % Convert quaternions to Euler angles (if you wanna do that sorta thing)
    roll = [];
    pitch = [];
    yaw = [];

    for i = 1 : length(qw)
        quat = [qw(i); qx(i); qy(i); qz(i)];
        [roll(i), pitch(i), yaw(i)] = q2euler(quat);
    end
    q_norm = sqrt(qw.^2 + qx.^2 + qy.^2 + qz.^2);

    % Calculate speed magnitude
    U = sqrt(u.^2 + v.^2 + w.^2);

    % Extract Control Inputs
    linac_pitch    = u_simout(:,1); % Linear actuator pitch
    linac_yaw      = u_simout(:,2); % Linear actuator yaw
    prop_speed_avg = u_simout(:,3); % Average propeller speed
    prop_speed_diff= u_simout(:,4); % Differential propeller speed

    %% Propeller plots
    if fig1_prop
        figure(1); 
        set(gcf, 'Position', [10 100 1000 300]);
        tiledlayout(2,1,'TileSpacing','tight','Padding','none');
        nexttile,plot(t,prop_speed_avg, 'color', col_ref), hold on

        plot( t,prop_speed_diff, 'color', col), hold off
        title('Propeller speed'),grid, %xlabel('time (s)')
        legend('P_{avg}','P_{diff}')

        nexttile,plot(t,rad2deg(linac_pitch),'color', col_ref),hold on
        plot( t,rad2deg(linac_yaw), 'color', col), hold off
        title('Linear actuators angle (deg)'),xlabel('time [s]'),grid
        legend('\delta_{pitch}','\delta_{yaw}')

        set(findall(gcf,'type','line'),'linewidth',2)
        set(findall(gcf,'type','text'),'FontSize',14)
        set(findall(gcf,'type','legend'),'FontSize',12)

        saveas(gcf, [figpath 'prop'])
        saveas(gcf, [figpath 'prop.png'])

    end

    %% Quaternion and angular velocity
    if fig3_quat_angvel
        quat_lim = [-1.1 1.1];
        figure(3)
        figure(gcf)
        set(gcf, 'Position', [10 100 1000 400]);

        tiledlayout(4,4,'TileSpacing','tight','Padding','none');
        nexttile(1, [1,2]), plot(t,qwr,':', 'color', col_ref), hold on

        plot( t,qw, 'color', col), hold off
        title('Quaternion Body to NED')
        ylabel('q_w'),ylim(quat_lim),grid
        nexttile(5, [1,2]),plot(t,qxr,':', 'color', col_ref),hold on
        plot( t,qx, 'color', col), hold off
        ylabel('q_x'),ylim(quat_lim),grid
        nexttile(9, [1,2]),plot(t,qyr,':', 'color', col_ref),hold on
        plot( t,qy, 'color', col), hold off
        ylabel('q_y'),ylim(quat_lim),grid
        nexttile(13, [1,2]),plot(t,qzr,':', 'color', col_ref),hold on
        plot( t,qz, 'color', col), hold off
        ylabel('q_z'),xlabel('time [s]'),ylim(quat_lim),grid
    
        nexttile(3, [1,2]), plot(t,rad2deg(pr),':', 'color', col_ref),hold on
        plot( t,rad2deg(p), 'color', col), hold off
        ylabel('\omega_x^b'),grid
        title('Angular velocities in Body [deg/s]')
        nexttile(7, [1,2]),plot(t,rad2deg(qr),':', 'color', col_ref),hold on
        plot( t,rad2deg(q), 'color', col), hold off
        ylabel('\omega_y^b'),grid
        nexttile(11, [1,2]),plot(t,rad2deg(rr),':', 'color', col_ref),hold on
        plot( t,rad2deg(r), 'color', col), hold off
        ylabel('\omega_z^b'),xlabel('time [s]'),grid

        set(findall(gcf,'type','line'),'linewidth',2)
        set(findall(gcf,'type','text'),'FontSize',14)
        set(findall(gcf,'type','legend'),'FontSize',12)

        saveas(gcf, [figpath 'quat_angvel'])
        saveas(gcf, [figpath 'quat_angvel.png'])
    end




    if fig5_posvel_enu
        R_un = Rquat(euler2q(pi, 0, pi/2));     % from NED to R=eye(1)
    
        % Rotate 3xN NED matrix to new frame
        pos_u = R_un * [x y z]'; 
        pos_ref_u = R_un * [xr yr zr]';
        x_u = pos_u(1, :)'; % Extract new x coordinates
        y_u = pos_u(2, :)'; % Extract new y coordinates
        z_u = pos_u(3, :)'; % Extract new z coordinates
        xr_u = pos_ref_u(1, :)'; % Extract new x coordinates
        yr_u = pos_ref_u(2, :)'; % Extract new y coordinates
        zr_u = pos_ref_u(3, :)'; % Extract new z coordinates

        vel_u = R_un * [u v w]';
        vel_ref_u = R_un * [ur vr wr]';
        u_u = vel_u(1, :)';
        v_u = vel_u(2, :)';
        w_u = vel_u(3, :)'; 
        ur_u = vel_ref_u(1, :)'; 
        vr_u = vel_ref_u(2, :)'; 
        wr_u = vel_ref_u(3, :)';

        % Determine limits
        % -2,2 to scale nicely with 10m apogee and the figsize
        x_min = min([-2, min([min(x_u) min(u_u)])]);
        x_max = max([2, max([max(x_u) max(u_u)])]);
        y_min = min([-2, min([min(y_u) min(v_u)])]);
        y_max = max([2, max([max(y_u) max(v_u)])]);
        z_min = 0;
        z_max = max(z_u);
        
        figure(4)
        set(gcf, 'Position', [10 100 1000 400]);
        tiledlayout(3,3,'TileSpacing','tight','Padding','none');
        nexttile(1, [3 1]), plot3(x_u,y_u,z_u, '-', 'color', col), hold on
        plot3(xr_u,yr_u,zr_u,'*', 'color', c_other), hold off
        xlabel('x East'), ylabel('y North'), zlabel('z Up')
        xlim([x_min x_max]), ylim([y_min y_max]), zlim([z_min z_max])
        view([-23,12]);
        labels = {'pos','ref'};
        legend(labels,'Location','northeast')    
        title('3D Position [m]')
        grid
        
        % 2D Plots for x, y, z positions and velocities
        % Ensure minimum y-axis value of 1e-6
        x_min = min([-1e-6, min([min(x_u) min(u_u)])]);
        x_max = max([1e-6, max([max(x_u) max(u_u)])]);
        y_min = min([-1e-6, min([min(y_u) min(v_u)])]);
        y_max = max([1e-6, max([max(y_u) max(v_u)])]);
        z_min = min([0, min([min(z_u) min(w_u)])]);
        %z_max = max(w_u); & just plot v

        nexttile(2, [1 2]), plot(t,xr_u, ':', 'color', col),hold on
        plot( t,ur_u, ':', 'color', col_ref) 
        plot( t,u_u, 'color', col_ref)
        plot( t,x_u, 'color', col)
        ylim([x_min x_max])

        labels = {'p_r [m]', 'v_r [m/s]', 'v [m/s]', 'p [m]'};
        legend(labels,'Location','northeast')    
        hold off,title('East [m]'),grid

        nexttile(5, [1 2]), plot(t,yr_u, ':', 'color',  col),hold on
        plot( t,vr_u, ':', 'color', col_ref)
        plot( t,v_u, 'color', col_ref)
        plot( t,y_u, 'color', col)
        ylim([y_min y_max])
        hold off,title('North [m]'),grid
            ylim([z_min z_max])

        nexttile(8, [1,2]), plot(t,zr_u, ':', 'color', col),hold on
        plot( t,wr_u, ':', 'color', col_ref)
        plot( t,w_u, 'color', col_ref)
        plot( t,z_u, 'color', col)
        ylim([z_min z_max])
        hold off,xlabel('time [s]'),title('Up [m]'),grid
        
        set(findall(gcf,'type','line'),'linewidth',2)
        set(findall(gcf,'type','text'),'FontSize',14)
        set(findall(gcf,'type','legend'),'FontSize',12)
        
        saveas(gcf, [figpath 'pos_vel_enu'])
        saveas(gcf, [figpath 'pos_vel_enu.png'])

    end
   
end

