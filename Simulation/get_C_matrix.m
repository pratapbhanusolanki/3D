function y = get_C_matrix(x,u2,u3,scan_radius, bias, phi)
    der_limit = 1000;
    scaling_coefficient = 1;
    alpha_biases = scan_radius*sind([bias,bias-phi, bias-2*phi]);
    beta_biases = scan_radius*cosd([bias,bias-phi, bias-2*phi]); 
	x1 = x(1);
	x2 = x(2) + beta_biases(1)*18;
	x3 = x(3) + alpha_biases(1);
    acosd(cosd(x2/18)*cosd(x3));
    

	%Current values
	g = gaussian_value(acosd(cosd(x2/18)*cosd(x3)));
	g_d = gaussian_derivative(acosd(cosd(x2/18)*cosd(x3)));
	der_acosd = (1/sqrt(1-(cosd(x2/18)*cosd(x3))^2));
    der_acosd = sign(der_acosd)*min(abs(der_acosd),der_limit);

	C(1,:) = [g, x1*g_d*der_acosd*cosd(x3)*(-1)*sind(x2/18)/18, x1*g_d*der_acosd*cosd(x2/18)*(-sind(x3))];

	%previous values
	x1 = x(1);
	x2 = x(2)-u2(1) + beta_biases(2)*18 ;
	x3 = x(3)-u3(1) + alpha_biases(2);

	g = gaussian_value(acosd(cosd(x2/18)*cosd(x3)));
	g_d = gaussian_derivative(acosd(cosd(x2/18)*cosd(x3)));
	der_acosd = (1/sqrt(1-(cosd(x2/18)*cosd(x3))^2));
    der_acosd = sign(der_acosd)*min(abs(der_acosd),der_limit);

	C(2,:) = [g, x1*g_d*der_acosd*cosd(x3)*(-1)*sind(x2/18)/18, x1*g_d*der_acosd*cosd(x2/18)*(-sind(x3))];

	%Previous to preious values
	x1 = x(1);
	x2 = x(2)-u2(1)-u2(2) + beta_biases(3)*18;
	x3 = x(3)-u3(1)-u3(2) + alpha_biases(3);
	g = gaussian_value(acosd(cosd(x2/18)*cosd(x3)));
	g_d = gaussian_derivative(acosd(cosd(x2/18)*cosd(x3)));
	der_acosd = (1/sqrt(1-(cosd(x2/18)*cosd(x3))^2));
    der_acosd = sign(der_acosd)*min(abs(der_acosd),der_limit);

	C(3,:) = [g, x1*g_d*der_acosd*cosd(x3)*(-1)*sind(x2/18)/18, x1*g_d*der_acosd*cosd(x2/18)*(-sind(x3))];

	y = C*[1,0,0; 0,scaling_coefficient,0 ; 0,0,scaling_coefficient];


end