clear all; close all; clc

% ===== PARAMETERS ========================================================
Params = SetParameters();
h = Params.Init.MPC.h;
N = Params.Init.MPC.N;
PLOT = true;

% =========================================================================

% MPC_1 - Setpoint 10m, down  ---------------------------------------------
DATA_FOLDER = 'fig/'; 
DATA_NAME ='mpc_1/';
FIGPATH = [DATA_FOLDER DATA_NAME];
%RunMPCSetpointMultShoot(h, N, Params.Init.MPC1, Params.MPC.MPC1, ...
%    Params.Guidance.MPC1, Params.Hopper, FIGPATH);
%plot_figpath(FIGPATH, 1);

% MPC_1_tuning
run_all_mpc1_tuning = false; % this takes so long dont do it
if run_all_mpc1_tuning
    w_xy_new = [125 150 175];
    w_z_new = [150 200 225];
    w_dxy_new = [2 5 7];
    w_dz_new = [5 10 15];
    n_xy=3; n_z=3; n_dxy=3; n_dz=3;
    
    n_runs = 1;
    for i = 1:n_xy      % 1-28
        for j = 1:n_z   % 1-9, 10-19, 20-28
            for k = 1:n_dxy % 1-3 4-6 7-9 ...
                for l = 1:n_dz
                    DATA_NAME = ['mpc_1_tuning/nr_' num2str(n_runs) '/'];  
                    FIGPATH = [DATA_FOLDER DATA_NAME];
    
                    Q_new = Params.MPC.MPC1.Q;
                    Q_new(1,1) = w_xy_new(i);
                    Q_new(2,2) = w_xy_new(i);
                    Q_new(3,3) = w_z_new(j);
                    Q_new(4,4) = w_dxy_new(k);
                    Q_new(5,5) = w_dxy_new(k);
                    Q_new(6,6) = w_dz_new(l);
                    Params.MPC.MPC1.Q = Q_new;
    
                    RunMPCSetpointMultShoot(h, N, Params.Init.MPC1, Params.MPC.MPC1, ...
                        Params.Guidance.MPC1, Params.Hopper, FIGPATH);
                    plot_figpath(FIGPATH, 1);
    
                    n_runs = n_runs + 1;
                end
            end
        end
    end
end
%%
DATA_NAME = 'mpc_1/';  
FIGPATH = [DATA_FOLDER DATA_NAME];
RunMPCSetpointMultShoot(h, N, Params.Init.MPC1, Params.MPC.MPC1, ...
    Params.Guidance.MPC1, Params.Hopper, FIGPATH);
plot_figpath(FIGPATH, 1);
%plot_mpc_data_multiple(FIGPATH);

% MPC_2 - Thrust up, no gimbal --------------------------------------------
DATA_NAME ='mpc_2/';
FIGPATH = [DATA_FOLDER DATA_NAME];
RunMPCSetpointMultShoot(h, N, Params.Init.MPC2, Params.MPC.MPC2, ...
    Params.Guidance.MPC2, Params.Hopper, FIGPATH);

% MPC_5 - Thrust and YAW +-1deg sin ---------------------------------------
DATA_NAME ='mpc_5/';
FIGPATH = [DATA_FOLDER DATA_NAME];
%RunMPCSetpointMultShoot(h, N, Params.Init.MPC5, Params.MPC.MPC5, ...
 %   Params.Guidance.MPC5, Params.Hopper, FIGPATH);
%plot_figpath(FIGPATH, 1);

%% Some more tests
n_pos=3; n_vel=3; n_angvel = 3;

angvel_roll = [5 10 50];
angvel_pitchyaw = [50 75 100];


n_runs = 1;
for i = 1:3
    DATA_NAME = ['mpc_5_tuning_angvel/nr_' num2str(n_runs) '_' num2str(angvel_roll(i)) '_' num2str(angvel_pitchyaw(i)) '/'];  
    FIGPATH = [DATA_FOLDER DATA_NAME];

    Q_new = Params.MPC.MPC5.Q;
    Q_new(11,11) = angvel_roll(i);
    Q_new(12,12) = angvel_roll(i);
    Q_new(13,13) = angvel_pitchyaw(i);
    Params.MPC.MPC1.Q = Q_new;

    RunMPCSetpointMultShoot(h, N, Params.Init.MPC5, Params.MPC.MPC5, ...
        Params.Guidance.MPC5, Params.Hopper, FIGPATH);
    plot_figpath(FIGPATH, 1);

    n_runs = n_runs + 1;
end
plot_mpc_data_multiple(FIGPATH);

w_xy_new = [125 150 175];
w_z_new = (4/3)*w_xy_new;

n_runs = 1;
for i = 1:3
    DATA_NAME = ['mpc_5_tuning_pos/nr_' num2str(n_runs) '_' num2str(w_xy_new(i)) '_' num2str(w_z_new(i)) '/'];  
    FIGPATH = [DATA_FOLDER DATA_NAME];

    Q_new = Params.MPC.MPC5.Q;
    Q_new(1,1) = w_xy_new(i);
    Q_new(2,2) = w_xy_new(i);
    Q_new(3,3) = w_z_new(i);
    Params.MPC.MPC1.Q = Q_new;

    RunMPCSetpointMultShoot(h, N, Params.Init.MPC5, Params.MPC.MPC5, ...
        Params.Guidance.MPC5, Params.Hopper, FIGPATH);
    plot_figpath(FIGPATH, 1);

    n_runs = n_runs + 1;
end
%plot_mpc_data_multiple(FIGPATH);


w_dxy_new = [2 5 7];
w_dz_new = 2*w_dxy_new;

n_runs = 1;

for i = 1:3
    DATA_NAME = ['mpc_5_tuning_vel/nr_' num2str(n_runs) '_' num2str(w_dxy_new(i)) '_' num2str(w_dz_new(i)) '/'];  
    FIGPATH = [DATA_FOLDER DATA_NAME];

    Q_new = Params.MPC.MPC5.Q;
    Q_new(4,4) = w_dxy_new(i);
    Q_new(5,5) = w_dxy_new(i);
    Q_new(6,6) = w_dz_new(i);
    Params.MPC.MPC1.Q = Q_new;

    RunMPCSetpointMultShoot(h, N, Params.Init.MPC5, Params.MPC.MPC5, ...
        Params.Guidance.MPC5, Params.Hopper, FIGPATH);
    plot_figpath(FIGPATH, 1);

    n_runs = n_runs + 1;
end
%plot_mpc_data_multiple(FIGPATH);

