function update_sim_figure(Hopper, pos_n, q_nb, w_b)
    ScaleFrame = 12;   % Scaling factor for adjusting the frame size (cosmetic)
    FS         = 15;  % Fontsize for text
    SW         = 0.035; % Arrows size
    scale_coord = 10;
    
    R_un = Rquat(euler2q(pi, 0, pi/2));     % from NED to R=eye(1)
    R_nb = Rquat(q_nb);                     
    draw_pos_n = scale_coord*pos_n;
    
    % global r_hopper_prop_g r_hopper_nose_g d_hopper_radius_g 
    
    r_hopper_prop_n     = R_nb * Hopper.r_hopper_prop;
    r_hopper_nose_n     = R_nb * Hopper.r_hopper_nose;
    draw_pos_prop_n     = draw_pos_n + scale_coord*r_hopper_prop_n;
    draw_pos_nose_n     = draw_pos_n + scale_coord*r_hopper_nose_n;

	figure(1);
	clf;
	hold on;
	
    DrawArrow(zeros(3,1), [0,0,100]', FS,SW,'up','color','green');
	DrawRectangle(zeros(3,1), diag([100,100,0]), 'color', 'green'); % Touch some grass
	DrawFrame(zeros(3,1), R_un, ScaleFrame,FS,SW,'n', 'color', 'blue') % NED
	DrawFrame(R_un*draw_pos_n, R_un*R_nb, ScaleFrame,FS,SW,'b', 'color', 'r') % Body
	DrawArrow(R_un*draw_pos_n, R_un*R_nb*w_b, FS,SW,'$$\omega$$', 'color', 'g') % angvel
	DrawCylinder(R_un*draw_pos_prop_n, R_un*draw_pos_nose_n, scale_coord * Hopper.radius); % Rocket
	DrawFormatPicture(R_un*draw_pos_n, 1*[100 100 100]')

	drawnow; % Force MATLAB to render the figure
end