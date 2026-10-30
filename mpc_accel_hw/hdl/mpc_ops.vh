// Lane op codes shared by mpc_core (issuer) and mpc_lane (executor).
localparam [3:0] OP_NOP    = 4'd0,
                 OP_MAC0   = 4'd1,   // acc  = coef * data
                 OP_MAC    = 4'd2,   // acc += coef * data
                 OP_RND_GX = 4'd3,   // gx   = R(acc, FRAC_MX)
                 OP_RND_GR = 4'd4,   // g    = gx - R(acc, FRAC_MR)
                 OP_RHS    = 4'd5,   // rhs  = R(rho * (z - y), FRAC_P) - g
                 OP_OMZ    = 4'd6,   // t    = R((1 - alpha) * z, FRAC_P)
                 OP_RND_U  = 4'd7,   // u    = R(acc, FRAC_W)
                 OP_AU     = 4'd8,   // u_hat = R(alpha * u, FRAC_P) + t
                 OP_UPD    = 4'd9;   // z = clamp(u_hat + y), y = y + u_hat - z
