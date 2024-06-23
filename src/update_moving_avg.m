function [pos_err_vec, avg] = update_moving_avg(pos_err_vec, pos_err)
    % updates vector for moving average position error
    pos_err_vec = [pos_err, pos_err_vec(1:end-1)];
    avg = mean(pos_err_vec);
end