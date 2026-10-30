function [u, pid_state] = pid_controller(error, pid_state, gains, Ts, u_min, u_max)
    Kp = gains(1); Ki = gains(2); Kd = gains(3);

    p_term = Kp * error;
    i_candidate = pid_state.integral + Ki * Ts * error;
    d_term = Kd * (error - pid_state.prev_error) / Ts;

    u_unsat = p_term + i_candidate + d_term;
    u = max(u_min, min(u_max, u_unsat));

    saturated = (u_unsat > u_max) || (u_unsat < u_min);
    same_sign = (error > 0 && u_unsat > 0) || (error < 0 && u_unsat < 0);

    if ~(saturated && same_sign)
        pid_state.integral = i_candidate;
    end
    
    % Hard clamp to avoid infinite accumulation
    int_max = 0.05;
    pid_state.integral = max(-int_max, min(int_max, pid_state.integral));

    pid_state.prev_error = error;
end
