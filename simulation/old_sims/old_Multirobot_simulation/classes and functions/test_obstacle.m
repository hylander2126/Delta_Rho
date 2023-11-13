clc; clear; close all;


V = [0.25 -0.25; 0.5 -0.25; 0.95 0.4; 0.25 0.8; -0.4 0.8;
    -0.95 0.4; -0.6 0; -0.95 -0.6; -0.5 -0.5; -0.4 -0.7; 0.15 -0.6];


Q = obstacle('Vertices',V,'bodyColor',rand(1,3));

Q.plot
hold on; box on; 
axis equal
axis([-2 2 -2 2]);


s0 = [-1;-1];

s0 = [-1.2;-1.2];

r = 1;

qq = linspace(0,2*pi); qq(end) = [];
S = [s0(1)+r*cos(qq)' s0(2)+r*sin(qq)'];

patch('Vertices',S,'Faces',1:99,'FaceColor','c','FaceAlpha',0.25);


[inter, x, cone] = Q.sphereXo(s0,r);

if(inter)
    plot(x(:,1),x(:,2),'or','MarkerFaceColor','r');

    quiver([s0(1);s0(1)],[s0(2);s0(2)],cone(:,1),cone(:,2),'r');
end

