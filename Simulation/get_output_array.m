function y = get_C_matrix(x,scan_params,u2,u3)
	
	%Current values
	x1 = x(1);
	x2 = x(2);
	x3 = x(3);
	g1 = x1*gaussian_value(acosd(cosd(x2/18)*cosd(x1)));
	
	%Previous values
	x1 = x(1);
	x2 = x(2)-m_command2(1);
	x3 = x(3)-m_command3(1);
	g2 = x1*gaussian_value(acosd(cosd(x2/18)*cosd(x1)));

	%Previous to previous values
	x1 = x(1);
	x2 = x(2)-m_command2(1)-m_command2(2);
	x3 = x(3)-m_command3(1)-m_command3(2);
	g3 = x1*gaussian_value(acosd(cosd(x2/18)*cosd(x1)));

	y = [g1;g2;g3];


end