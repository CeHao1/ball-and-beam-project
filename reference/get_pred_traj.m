function [p_pred, v_pred, a_pred] = get_pred_traj(t, wave_pattern, A, omega)
    if (wave_pattern == 0)
        p_pred = A;
        v_pred = 0;
        a_pred = 0;
    elseif (wave_pattern == 1)
        p_pred = A * sin(omega * t);
        v_pred = A * omega * cos(omega * t);
        a_pred = - A * omega^2 * sin(omega * t);
    end
end