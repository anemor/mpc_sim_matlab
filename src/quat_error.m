function [q_err] = quat_error(q1, q2)
    % q1-q2
	q_err = quatprod(quat_conj(q2), q1);
end

