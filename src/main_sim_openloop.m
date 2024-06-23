clear all; close all; clc

% ===== PARAMETERS ========================================================
Params = SetParameters();
h = Params.Init.Sim.h;
N = Params.Init.Sim.N;
PLOT = true;
% =========================================================================

% SIM_1 - Free-fall from 100m ---------------------------------------------
DATA_FOLDER = 'fig/';
DATA_NAME ='sim_1/';
FIGPATH = [DATA_FOLDER DATA_NAME];
RunOpenloopSim(h, N, Params.Init.Sim1, Params.Hopper, FIGPATH, PLOT);
if PLOT
    plot_mpc(mpc_timestamps, ...
			mpc_states, ...
			mpc_controls, ...
			mpc_state_reference,   ...
				storepath);
end

% SIM_2 - Thrust up, no gimbal --------------------------------------------
DATA_NAME ='sim_2/';
FIGPATH = [DATA_FOLDER DATA_NAME];
RunOpenloopSim(h, N, Params.Init.Sim2, Params.Hopper, FIGPATH, PLOT);

% SIM_3 - Thrust up with ROLL MOMENT  -------------------------------------
DATA_NAME ='sim_3/';
FIGPATH = [DATA_FOLDER DATA_NAME];
RunOpenloopSim(h, N, Params.Init.Sim3, Params.Hopper, FIGPATH, PLOT);

% SIM_4 - Thrust and PITCH +-1deg sin -------------------------------------
DATA_NAME ='sim_4/';
FIGPATH = [DATA_FOLDER DATA_NAME];
RunOpenloopSim(h, N, Params.Init.Sim4, Params.Hopper, FIGPATH, PLOT);

% SIM_5 - Thrust and YAW +-1deg sin ---------------------------------------
DATA_NAME ='sim_5/';
FIGPATH = [DATA_FOLDER DATA_NAME];
RunOpenloopSim(h, N, Params.Init.Sim5, Params.Hopper, FIGPATH, PLOT);

% SIM_6 - SIM_2 with disturbance ------------------------------------------
DATA_NAME ='sim_6/';
FIGPATH = [DATA_FOLDER DATA_NAME];
RunOpenloopSim(h, N, Params.Init.Sim6, Params.Hopper, FIGPATH, PLOT);


