syms p 

b1 = -3*pi/4;
b2 = 3*pi/4;
b3 = pi;

th1 = pi/4;
th2 = 3*pi/4;
th3 = -pi/2;

l11 = Rz2(b1);
l12 = Rz2(b2);
l13 = Rz2(b3);

l21 = [-sin(th1)*p 1 0 ; cos(th1)*p 0 1];
l22 = [-sin(th2)*p 1 0 ; cos(th2)*p 0 1];
l23 = [-sin(th3)*p 1 0 ; cos(th3)*p 0 1];

h1 = [1 0] * l11*l21;
h2 = [1 0] * l12*l22;
h3 = [1 0] * l13*l23;

H = [h1; h2; h3];

H = round(subs(H, p, 1), 3)

