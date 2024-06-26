close all; 

% SIM=0     MPC=1    
PLOT_SIM_MPC = 0;           

DATA_FOLDER = 'fig/';
DATA_NAME = 'sim_1/';  
FIGPATH = [DATA_FOLDER DATA_NAME];

%plot_figpath(FIGPATH, PLOT_SIM_MPC);
DATA_NAME = 'mpc_1_tuning/';  
FIGPATH = [DATA_FOLDER DATA_NAME];
%plot_mpc_data_multiple('fig/mpc_1_tuning/nr');

n_figs = 5;
for i = 1:n_figs
    figpath = [DATA_FOLDER 'mpc_' num2str(i) '/'];
    plot_figpath(figpath, 1);
end
