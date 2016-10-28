function y = get_output_array(x,u2,u3,scan_radius, bias,phi)

    alpha_biases = scan_radius*sind([bias,bias-phi, bias-2*phi]);
    beta_biases = scan_radius*cosd([bias,bias-phi, bias-2*phi]) ;
	
	%Current values
	x1 = x(1);
	x2 = x(2) + beta_biases(1)*18;
	x3 = x(3) + alpha_biases(1);
	g1 = x1*gaussian_value(acosd(cosd(x2/18)*cosd(x3)));
	
	%Previous values
	x1 = x(1);
	x2 = x(2)-u2(1) + beta_biases(2)*18;
	x3 = x(3)-u3(1) + alpha_biases(2);
	g2 = x1*gaussian_value(acosd(cosd(x2/18)*cosd(x3)));

	%Previous to previous values
	x1 = x(1);
	x2 = x(2)-u2(1)-u2(2) + beta_biases(3)*18;
	x3 = x(3)-u3(1)-u3(2) + alpha_biases(3);
	g3 = x1*gaussian_value(acosd(cosd(x2/18)*cosd(x3)));

	y = [g1;g2;g3];


end