function y = exact_measurement_model(x1,x2,x3)
    y = x1*gaussian_value(acosd(cosd(x2).*cosd(x3)));
end