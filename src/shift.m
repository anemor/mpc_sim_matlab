function [t0, x0, u0] = shift(dt, t0, x0, u_traj,f)
    u0 = [  u_traj(2:size(u_traj,1),:); 
            u_traj(size(u_traj,1),:)];

    st = x0;
    con = u_traj(1,:)';
    dx = f(st,con);    % f uses RK4 to integrate
    
    st = st + (dt*dx);
    x0 = full(st);
    x0(1) = x0(1)+ 0.5/13; % add wind force a = F/m
    x0(7:10) = x0(7:10) / norm(x0(7:10));
    
    t0 = t0 + dt;
end