function unit_vec = circular_line_vector_field(Circular, position, K1, K2)
    % circular_line_vector_field gives unit vector in R3 for reference
    % input of velocity direction.

    % Circular.center : 3x1 position vector of Circular path's center
    % Circular.lambda_h : 1 for CCW, -1 for CW
    % Circular.rho_h : radius of Circular-line path
    % position : 3x1 position vector of UAV
    % K1, K2 : 3x3 symmetrix gain matrices

    tilde = position - Circular.center;

    alpha_cyl = tilde(1)^2 + tilde(2)^2 - Circular.rho_h^2;
    alpha_pl = tilde(3);

    p_alpha_cyl = tilde*2;
    p_alpha_cyl(3) = 0;

    p_alpha_pl = [0; 0; 1];

    if Circular.lambda_h == 1
        u_line = -K1*(alpha_cyl*p_alpha_cyl + alpha_pl*p_alpha_pl) + K2*cross(p_alpha_pl, p_alpha_cyl, 1);
        unit_vec = u_line./norm(u_line);
    elseif Circular.lambda_h == -1
        u_line = -K1*(alpha_cyl*p_alpha_cyl + alpha_pl*p_alpha_pl) + K2*cross(p_alpha_cyl, p_alpha_pl, 1);
        unit_vec = u_line./norm(u_line);
    else
        error("Only -1 or 1 can be value of Circular.lambda_h")
    end
end

