function [x_err] = state_error(x, x_ref)
    x_err = [ x(1:3) - x_ref(1:3);
                x(4:6) - x_ref(4:6);
                quat_error(x(7:13), x_ref(7:13));
                x(11:13) - x(11:13)];
end