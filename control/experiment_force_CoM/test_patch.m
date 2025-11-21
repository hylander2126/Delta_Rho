% From image
f = figure();
axis equal
x_poly = -7.6*[  0     1     2   2.6  2.2   3   4    5   5.75  5.9  6.1 5.9  5.7   5   4  3.75  3.5  3.75  4    4.2  4   3   2    1   0 ...
    -1  -2  -3  -4 -5 -5.2 -5.4 -5.2  -5  -4 -3.5 -3.15 -3.25 -3.7   -4     -5    -6  -6.6   -6.9    -6.6    -6    ...
    -5    -4   -3   -2     -1];
y_poly = -7.6*[-14.9 -14.5 -13.3 -10  -7   -3  -2  -1.2  -0.3 0.2 0.9  1.1  1.2  1.8  3  3.3    4   4.8   5.2  6.2  7  7.5 7.7 7.65 ...
    7.45 7  6.3 5.3  4  2  1.7  0.9  0.2   0  -1  -2   -4     -5    -6   -6.6  -7.5 -8.65 -9.75 -10.8  -11.6  -12.4 ...
    -13.3 -14 -14.5 -14.8 -15.1];


p = patch(x_poly,y_poly, 'r');

% Ensure the polygon is closed by repeating the first point at the end
x = [x, x(1)];
y = [y, y(1)];

% Calculate the area using the shoelace formula
n = length(x) - 1; % Number of vertices in the closed loop
area = 0;
for i = 1:n
    area = area + (x(i)*y(i+1) - x(i+1)*y(i));
end
area = abs(area) / 2;

disp('Total Surface Area:');
disp(area);
