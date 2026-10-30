function main_trajectory()
    params = get_params();
    x = zeros(12, 1);
    x(3) = 1.0;
    duration = 50.0;
    num_steps = round(duration / params.Ts);

    time_arr = zeros(num_steps+1, 1);
    ref_arr = zeros(num_steps+1, 4);
    state_arr = zeros(num_steps+1, 12);
    error_arr = zeros(num_steps+1, 4);
    control_arr = zeros(num_steps+1, 4);
    omega_arr = zeros(num_steps+1, 4);

    pid_x = init_pid();
    pid_y = init_pid();
    pid_z = init_pid();
    pid_phi = init_pid();
    pid_theta = init_pid();
    pid_psi = init_pid();


    int_max = 0.05;

    % Log row k = time (k-1)*Ts; the last row computes control without stepping the plant.
    for k = 1:num_steps+1
        t = (k-1) * params.Ts;
        time_arr(k) = t;
        state_arr(k, :) = x';

        R = 0.5;
        omega_t = 0.5;
        ref = [R*cos(omega_t*t); R*sin(omega_t*t); 1.0 + 0.2*sin(t); 0];

        ex_world = ref(1) - x(1);
        ey_world = ref(2) - x(2);
        ez = ref(3) - x(3);
        epsi = ref(4) - x(9);

        psi = x(9);
        ex_body = cos(psi)*ex_world + sin(psi)*ey_world;
        ey_body = -sin(psi)*ex_world + cos(psi)*ey_world;

        ex_sat = max(-params.F_sat, min(params.F_sat, ex_body));
        ey_sat = max(-params.F_sat, min(params.F_sat, ey_body));

        % Perbaikan bug: 
        % theta mengontrol sumbu X (Pitch down -> +X acc), jadi theta_des = Kp*ex - Kd*u
        % phi mengontrol sumbu Y (Roll right -> -Y acc), jadi phi_des = -(Kp*ey - Kd*v)
        theta_des = params.gain_pos * ex_sat - params.gain_vel * x(4);
        phi_des = -params.gain_pos * ey_sat + params.gain_vel * x(5);

        % Limit angle to 0.3 rad to keep linear model valid
        phi_des = max(-0.3, min(0.3, phi_des));
        theta_des = max(-0.3, min(0.3, theta_des));

        if x(3) < 0.5
            F_cmd = params.hover_thrust + params.K_tf * ez - 0.3 * x(6);
            F_cmd = max(0, min(2.0 * params.hover_thrust, F_cmd));
        else
            F_cmd = params.hover_thrust + 0.8 * ez - 0.3 * x(6);
            F_cmd = max(0, min(2.0 * params.hover_thrust, F_cmd));
        end

        ephi = phi_des - x(7);
        etheta = theta_des - x(8);

        tau_x_unsat = 0.01 * ephi + 0.01 * pid_phi.integral - 0.0028 * x(10);
        tau_y_unsat = 0.013 * etheta + 0.01 * pid_theta.integral - 0.002 * x(11);
        tau_z_unsat = 0.004 * epsi - 0.00012 * x(12);

        sat_x = abs(tau_x_unsat) > params.tau_max;
        sat_y = abs(tau_y_unsat) > params.tau_max;
        sat_z = abs(tau_z_unsat) > params.tau_max;

        if ~(sat_x && sign(ephi) == sign(tau_x_unsat))
            pid_phi.integral = pid_phi.integral + params.Ts * ephi;
        end
        if ~(sat_y && sign(etheta) == sign(tau_y_unsat))
            pid_theta.integral = pid_theta.integral + params.Ts * etheta;
        end
        if ~(sat_z && sign(epsi) == sign(tau_z_unsat))
            pid_psi.integral = pid_psi.integral + params.Ts * epsi;
        end

        % Integral limit untuk mencegah windup padahal control signal belum saturasi
        pid_phi.integral = max(-int_max, min(int_max, pid_phi.integral));
        pid_theta.integral = max(-int_max, min(int_max, pid_theta.integral));
        pid_psi.integral = max(-int_max, min(int_max, pid_psi.integral));

        tau_x = max(-params.tau_max, min(params.tau_max, tau_x_unsat));
        tau_y = max(-params.tau_max, min(params.tau_max, tau_y_unsat));
        tau_z = max(-params.tau_max, min(params.tau_max, tau_z_unsat));

        U = [F_cmd; tau_x; tau_y; tau_z];
        omega_ref = mixer(U, params);

        ref_arr(k, :) = ref';
        error_arr(k, :) = [ex_world, ey_world, ez, epsi];
        control_arr(k, :) = U';
        omega_arr(k, :) = omega_ref';

        if k <= num_steps
            x = quadcopter_model(x, omega_ref, params);
        end
    end


    T = table(time_arr, ref_arr(:,1), ref_arr(:,2), ref_arr(:,3), ref_arr(:,4), ...
        state_arr(:,1), state_arr(:,2), state_arr(:,3), ...
        state_arr(:,7), state_arr(:,8), state_arr(:,9), ...
        state_arr(:,10), state_arr(:,11), state_arr(:,12), ...
        error_arr(:,1), error_arr(:,2), error_arr(:,3), ...
        control_arr(:,1), control_arr(:,2), control_arr(:,3), control_arr(:,4), ...
        omega_arr(:,1), omega_arr(:,2), omega_arr(:,3), omega_arr(:,4), ...
        'VariableNames', {'time','x_ref','y_ref','z_ref','psi_ref','x','y','z','phi','theta','psi','p','q','r','ex','ey','ez','F_total','tau_x','tau_y','tau_z','omega1','omega2','omega3','omega4'});
    writetable(T, 'results/trajectory.csv');

    rmse_x = sqrt(mean(error_arr(:,1).^2));
    rmse_y = sqrt(mean(error_arr(:,2).^2));
    rmse_z = sqrt(mean(error_arr(:,3).^2));
    norm_x = norm(error_arr(:,1));
    norm_y = norm(error_arr(:,2));
    norm_z = norm(error_arr(:,3));

    fprintf('Trajectory Tracking Metrics:\n');
    fprintf('  RMSE X: %.6f m\n', rmse_x);
    fprintf('  RMSE Y: %.6f m\n', rmse_y);
    fprintf('  RMSE Z: %.6f m\n', rmse_z);
    fprintf('  2-Norm X: %.6f\n', norm_x);
    fprintf('  2-Norm Y: %.6f\n', norm_y);
    fprintf('  2-Norm Z: %.6f\n', norm_z);

    figure('Name', 'Trajectory Tracking');
    subplot(3,2,1); plot(time_arr, ref_arr(:,1), 'r--', time_arr, state_arr(:,1), 'b'); legend('ref','actual'); title('X Position'); xlabel('time (s)'); ylabel('x (m)');
    subplot(3,2,2); plot(time_arr, ref_arr(:,2), 'r--', time_arr, state_arr(:,2), 'b'); legend('ref','actual'); title('Y Position'); xlabel('time (s)'); ylabel('y (m)');
    subplot(3,2,3); plot(time_arr, ref_arr(:,3), 'r--', time_arr, state_arr(:,3), 'b'); legend('ref','actual'); title('Z Position'); xlabel('time (s)'); ylabel('z (m)');
    subplot(3,2,4); plot(time_arr, ref_arr(:,4), 'r--', time_arr, state_arr(:,9), 'b'); legend('ref','actual'); title('Yaw'); xlabel('time (s)'); ylabel('\psi (rad)');
    subplot(3,2,5); plot(time_arr, state_arr(:,7), 'b'); title('Roll'); xlabel('time (s)'); ylabel('\phi (rad)');
    subplot(3,2,6); plot(time_arr, state_arr(:,8), 'b'); title('Pitch'); xlabel('time (s)'); ylabel('\theta (rad)');
end

function params = get_params()
    params.m = 0.068;
    params.arm_length = 0.062;
    params.g = 9.81;
    params.Ix = 6.86e-5;
    params.Iy = 9.2e-5;
    params.Iz = 1.366e-4;
    params.kF = 0.01;
    params.kM = 7.8263e-4;
    params.Ts = 0.005;
    params.hover_thrust = params.m * params.g;
    params.arm_dist = params.arm_length / sqrt(2);
    params.tau_max = 0.02;
    params.omega_max = 100.0;
    params.theta_limit = 1.45;
    params.F_sat = 3.0;
    params.gain_pos = 0.24;
    params.gain_vel = 0.1;
    params.K_tf = 0.45;
end

function pid_state = init_pid()
    pid_state.integral = 0;
    pid_state.prev_error = 0;
end