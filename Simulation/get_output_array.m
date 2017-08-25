function y = get_output_array(x,previous_u,scan_parameters)

    u2 = previous_u(:,1);
    u3 = previous_u(:,2);
    
    scan_radius = scan_parameters(1);
    bias = scan_parameters(2);
    phi = scan_parameters(3);
    alpha_biases = scan_radius*sind([bias,bias-phi, bias-2*phi]);
    beta_biases = scan_radius*cosd([bias,bias-phi, bias-2*phi]) ;
	
	%Current values
	x1 = x(1);
	x2 = x(2) + beta_biases(1);
	x3 = x(3) + alpha_biases(1);
	y1 = x1*g(x2)*g(x3);
	
% 	%Previous values
% 	x1 = x(1);
% 	x2 = x(2)-u2(1) + beta_biases(2);
% 	x3 = x(3)-u3(1) + alpha_biases(2);
% 	y2 = x1*g(x2)*g(x3);
%     
% 	%Previous to previous values
% 	x1 = x(1);
% 	x2 = x(2)-u2(1)-u2(2) + beta_biases(3);
% 	x3 = x(3)-u3(1)-u3(2) + alpha_biases(3);
	y3 = x1*g(x2)*g(x3);

	%y = [y1;y2;y3];
    y = y1;


end