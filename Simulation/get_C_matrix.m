function y = get_C_matrix(x,previous_u,scan_parameters)
    u2 = previous_u(:,1);
    u3 = previous_u(:,2);
    
    scan_radius = scan_parameters(1);
    bias = scan_parameters(2);
    phi = scan_parameters(3);
    
    scaling_coefficient = 1;
    alpha_biases = scan_radius*sind([bias,bias-phi, bias-2*phi]);
    beta_biases = scan_radius*cosd([bias,bias-phi, bias-2*phi]); 
    
	x1 = x(1);
	x2 = x(2) + beta_biases(1);
	x3 = x(3) + alpha_biases(1);
    
	%Current values
    % 	g = gaussian_value(acosd(cosd(x2/18)*cosd(x3)));
    % 	g_d = gaussian_derivative(acosd(cosd(x2/18)*cosd(x3)));
    % 	der_acosd = (1/sqrt(1-(cosd(x2/18)*cosd(x3))^2));
    %     der_acosd = sign(der_acosd)*min(abs(der_acosd),der_limit);
	%C(1,:) = [g, x1*g_d*der_acosd*cosd(x3)*(-1)*sind(x2/18)/18, x1*g_d*der_acosd*cosd(x2/18)*(-sind(x3))];
    C(1,:) = [g(x1)*g(x2), x1*g_d(x2)*g(x3), x1*g(x2)*g_d(x3)];

	%previous values
	x1 = x(1);
	x2 = x(2)-u2(1) + beta_biases(2);
	x3 = x(3)-u3(1) + alpha_biases(2);

	C(2,:) = [g(x1)*g(x2), x1*g_d(x2)*g(x3), x1*g(x2)*g_d(x3)];

	%Previous to preious values
	x1 = x(1);
	x2 = x(2)-u2(1)-u2(2) + beta_biases(3);
	x3 = x(3)-u3(1)-u3(2) + alpha_biases(3);

	C(3,:) = [g(x1)*g(x2), x1*g_d(x2)*g(x3), x1*g(x2)*g_d(x3)];
    
	y = C*[1,0,0; 0,scaling_coefficient,0 ; 0,0,scaling_coefficient];


end