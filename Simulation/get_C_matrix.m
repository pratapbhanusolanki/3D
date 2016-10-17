function y = get_C_matrix(x,scan_params,u2,u3)
	x1 = x(1);
	x2 = x(2);
	x3 = x(3);

	%Current values
	g = gaussian_value(acosd(cosd(x2/18)*cosd(x1)));
	g_d = gaussian_derivative(acosd(cosd(x2/18)*cosd(x1)));
	der_acosd = (1/sqrt(1-(cosd(x2/18)*cosd(x3))^2));

	C(1,:) = [g, x1*g_d*der_acosd*cosd(x3)*(-1)*sind(x2/18)/18,x1*g_d*der_acosd*cosd(x2/18)*(-sind(x3))];

	%previous values
	x1 = x(1);
	x2 = x(2)-m_command2(1);
	x3 = x(3)-m_command3(1);

	g = gaussian_value(acosd(cosd(x2/18)*cosd(x1)));
	g_d = gaussian_derivative(acosd(cosd(x2/18)*cosd(x1)));
	der_acosd = (1/sqrt(1-(cosd(x2/18)*cosd(x3))^2));

	C(2,:) = [g, x1*g_d*der_acosd*cosd(x3)*(-1)*sind(x2/18)/18,x1*g_d*der_acosd*cosd(x2/18)*(-sind(x3))];

	%Previous to preious values
	x1 = x(1);
	x2 = x(2)-m_command2(1)-m_command2(2);
	x3 = x(3)-m_command3(1)-m_command3(2);

	g = gaussian_value(acosd(cosd(x2/18)*cosd(x1)));
	g_d = gaussian_derivative(acosd(cosd(x2/18)*cosd(x1)));
	der_acosd = (1/sqrt(1-(cosd(x2/18)*cosd(x3))^2));

	C(2,:) = [g, x1*g_d*der_acosd*cosd(x3)*(-1)*sind(x2/18)/18,x1*g_d*der_acosd*cosd(x2/18)*(-sind(x3))];

	y = C;


end