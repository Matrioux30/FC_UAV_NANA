function x_next = quadcopter_model(x, u, params)
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10); q = x(11); r = x(12);
    u_vel = x(4); v_vel = x(5); w_vel = x(6);

    F_total = params.kF * (u(1)^2 + u(2)^2 + u(3)^2 + u(4)^2);
    tau_x = params.arm_dist * params.kF * (-u(1)^2 + u(2)^2 + u(3)^2 - u(4)^2);
    tau_y = params.arm_dist * params.kF * (-u(1)^2 - u(2)^2 + u(3)^2 + u(4)^2);
    tau_z = params.kM * (u(1)^2 - u(2)^2 + u(3)^2 - u(4)^2);

    theta_c = theta;
    if theta_c > params.theta_limit
        theta_c = params.theta_limit;
    elseif theta_c < -params.theta_limit
        theta_c = -params.theta_limit;
    end

    dx = zeros(12, 1);

    dx(1) = cos(theta_c)*cos(psi)*u_vel + (sin(phi)*sin(theta_c)*cos(psi) - cos(phi)*sin(psi))*v_vel + (cos(phi)*sin(theta_c)*cos(psi) + sin(phi)*sin(psi))*w_vel;
    dx(2) = cos(theta_c)*sin(psi)*u_vel + (sin(phi)*sin(theta_c)*sin(psi) + cos(phi)*cos(psi))*v_vel + (cos(phi)*sin(theta_c)*sin(psi) - sin(phi)*cos(psi))*w_vel;
    dx(3) = -sin(theta_c)*u_vel + sin(phi)*cos(theta_c)*v_vel + cos(phi)*cos(theta_c)*w_vel;

    dx(4) = r*v_vel - q*w_vel + params.g*sin(theta_c);
    dx(5) = p*w_vel - r*u_vel - params.g*cos(theta_c)*sin(phi);
    dx(6) = q*u_vel - p*v_vel - params.g*cos(theta_c)*cos(phi) + F_total/params.m;

    tan_theta = tan(theta_c);
    dx(7) = p + sin(phi)*tan_theta*q + cos(phi)*tan_theta*r;
    dx(8) = cos(phi)*q - sin(phi)*r;
    dx(9) = sin(phi)/cos(theta_c)*q + cos(phi)/cos(theta_c)*r;

    dx(10) = (q*r*(params.Iy - params.Iz) + tau_x) / params.Ix;
    dx(11) = (p*r*(params.Iz - params.Ix) + tau_y) / params.Iy;
    dx(12) = (p*q*(params.Ix - params.Iy) + tau_z) / params.Iz;

    x_next = x + params.Ts * dx;
end
