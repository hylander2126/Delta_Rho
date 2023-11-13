clc; clear; close;


normals = [2 0; 0 2; 0.1 0.3; 0.1 0.2; 0.3 0.4; -0.1 0.5];
N  = size(normals,1);


for i=1: N
    normals(i,:) = normals(i,:)/norm(normals(i,:));
end

quiver(zeros(N,1),zeros(N,1),normals(:,1),normals(:,2));
hold on

q = 1;
for i=1:N
    for j=i+1:N
        q_ = normals(i,:)*normals(j,:)';
        if(q_ < q)
            q = q_;
            I = i;
            J = j;
        end
    end
end

quiver(zeros(2,1),zeros(2,1),normals([I J],1),normals([I J],2),'r');
axis equal