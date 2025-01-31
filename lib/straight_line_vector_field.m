function unit_vec = straight_line_vector_field(start_point, end_point, position, K1, K2)
    % straight_line_vector_field gives unit vector in R3 for reference
    % input of velocity direction.

    % start_point : 3x1 position vector where straight-line path starts
    % end_point : 3x1 position vector where straight-line path ends
    % position : 3x1 position vector of UAV
    % K1, K2 : 3x3 symmetrix gain matrices

    % unit vector from start to end point
    ell = end_point - start_point;
    n_ell = ell ./ norm(ell);
    
    % Longitudinal Manifold's unit normal vector
    n_lon = zeros([3, 1]);
    n_lon(1) = -ell(2);
    n_lon(2) = ell(1);
    n_lon = n_lon./norm(n_lon);
    
    % Lateral Manifold's unit normal vector
    n_lat = cross(n_lon, n_ell, 1);
    n_lat = n_lat ./ norm(n_lat);
        
    % Distance between Point and Straight-Line
    a_lon = n_lon'*(position - start_point);
    a_lat = n_lat'*(position - start_point);
    u_line = -K1*(a_lon*n_lon + a_lat*n_lat) + K2*n_ell;
    unit_vec = u_line./norm(u_line);
end