function v_rot = v_quatrotate(q, v)
    import casadi.*
    % Performs qvq-1 to rotate vector v by quaternion q 
    % active rotation = the vector is rotated with respect to the coordinate system 

    q = q / sqrt(q'*q);

    v_quat = [0; v];
    qv = quatprod(q, v_quat);
    v_rot_quat = quatprod(qv, quat_conj(q));
    v_rot = v_rot_quat(2:4);
end