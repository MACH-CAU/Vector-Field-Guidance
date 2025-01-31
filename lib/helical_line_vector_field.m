function unit_vec = helical_line_vector_field(Helical, position, K1, K2)
    % helical_line_vector_field gives unit vector in R3 for reference
    % input of velocity direction.

    % Helical.center : 3x1 position vector of helical path's center
    % Helical.gamma_h : flight path angle(climb angle)
    % Helical.lambda_h : 1 for CW, -1 for CCW
    % Helical.rho_h : radius of helical-line path
    % Helical.psi : start angle of helical-line path
    % position : 3x1 position vector of UAV
    % K1, K2 : 3x3 symmetrix gain matrices

    tilde = position - Helical.center;

    alpha_cyl = (tilde(1)/Helical.rho_h)^2 + (tilde(2)/Helical.rho_h)^2 - 1;
    alpha_pl = tilde(3)/Helical.rho_h - tan(Helical.gamma_h)/Helical.lambda_h*(atan(tilde(2)/tilde(1)) - Helical.psi);

    p_alpha_cyl = [2*tilde(1)/Helical.rho_h;...
                   2*tilde(2)/Helical.rho_h;...
                   0];

    p_alpha_pl = [tan(Helical.gamma_h) / Helical.lambda_h * tilde(2) / (tilde(1)^2 + tilde(2)^2);...
                  -tan(Helical.gamma_h) / Helical.lambda_h * tilde(1) / (tilde(1)^2 + tilde(2)^2);...
                  1/Helical.rho_h];

    u_line = -K1*(alpha_cyl*p_alpha_cyl + alpha_pl*p_alpha_pl) + K2*cross(p_alpha_pl, p_alpha_cyl, 1)*Helical.lambda_h;
    unit_vec = u_line./norm(u_line);
end

