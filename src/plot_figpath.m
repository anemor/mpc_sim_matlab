function plot_figpath(figpath, choose_datatype) 
    % Uses specific plotting functions for Openloop Sim or Closedloop MPC
    % to plot the state and controls (.png and matlab figure).
    %
    % Input:
    %   figpath (str)         : path to data and resulting figures
    %   choose_datatype (int) : choose if data is from Sim=0 or MPC=1

    if choose_datatype == 0
        load([figpath 'sim_timestamps'])
        load([figpath 'sim_states'])
        load([figpath 'sim_controls'])
    
        plot_sim_data(sim_timestamps, ...
                sim_states, ...
                sim_controls, ...
                figpath);

    elseif choose_datatype == 1
        load([figpath 'mpc_timestamps'])
        load([figpath 'mpc_states'])
        load([figpath 'mpc_controls'])
        load([figpath 'mpc_predicted_states'])
        load([figpath 'mpc_predicted_controls'])
        load([figpath 'mpc_state_reference'])
        
        plot_mpc_data(mpc_timestamps, ...
                    mpc_states, ...
                    mpc_controls, ...
                    mpc_state_reference,   ...
                    figpath);
    else 
        disp('Unknown datatype, use SIM=0 or MPC=1.')
    end
end