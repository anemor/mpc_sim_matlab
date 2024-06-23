function q_conj = quat_conj(q)
    q_conj = [q(1); -q(2:4)];
end