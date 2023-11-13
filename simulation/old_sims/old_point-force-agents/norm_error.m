function E = norm_error(X)

global Xd


N = size(X,1);

for n=1:N
    E(n) = norm(X(n,1:3)' - Xd);
end