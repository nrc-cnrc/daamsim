%AxisAngleRotation.m
% Copyright (c) 2022 National Research Council Canada
%n is a 3x1 vector, theta is specified in degrees.


function R = AxisAngleRotation(n, theta)

	ncross = skew(n);
	R = (1 - cosd(theta))*(n*n') + ...
        cosd(theta)*eye(3,3) + ... 
        sind(theta)*ncross;	

end

function R = skew(n)

	R = [0, -n(3), n(2);
		 n(3), 0, -n(1);
		 -n(2), n(1), 0];
end
