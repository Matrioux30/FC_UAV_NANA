function omega_ref = mixer(U, params)
    d = params.arm_dist;
    c = params.kM / params.kF;

    M = [1,  1,  1,  1;
        -d,  d,  d, -d;
        -d, -d,  d,  d;
         c, -c,  c, -c];

    forces = [U(1)/params.kF; U(2)/params.kF; U(3)/params.kF; U(4)/params.kF];
    omega_sq = M \ forces;

    omega_ref = zeros(4, 1);
    for i = 1:4
        if omega_sq(i) > 0
            omega_ref(i) = sqrt(omega_sq(i));
        end
    end

    omega_ref = max(0, min(params.omega_max, omega_ref));
end
