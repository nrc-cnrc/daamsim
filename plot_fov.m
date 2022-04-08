% Copyright (c) 2022 National Research Council Canada
function P = plot_fov(a,b,h,k,r)
% Plot a sensor fov as a pie wedge.
% a is start of pie wedge, 
% b is end, both in radians, 
% (h,k) is the center of the circle.
% r is the radius.
t = linspace(a,b);
x = r*cos(t) + h;
y = r*sin(t) + k;
x = [x h x(1)];
y = [y k y(1)];
P = fill(x,y,'w', 'linestyle', '--', 'edgecolor','green');
set(P,'facealpha',0)
%axis([h-r-1 h+r+1 k-r-1 k+r+1]) 
axis square;
if ~nargout
    clear P
end

