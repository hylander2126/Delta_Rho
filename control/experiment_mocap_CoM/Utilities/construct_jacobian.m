function J = construct_jacobian(P)


N = size(P,2);

if(N == 1)
    
    % r = sqrt(P(1)^2 + P(2)^2)/2;
    % q = atan2(P(2),P(1));
    % J = [1 0 1 0; 0 1 0 1; -P(2) P(1) -r*sin(q+pi/10) r*cos(q+pi/10)];
    
    J = [1 0 1 0; 0 1 0 1; -P(2) P(1) P(2) -P(1)];
else
    
    J = zeros(3,2*N,N);
    
    for i=1:N
        P_(:,1) = P(:,i);
        P_(:,2:i) = P(:,1:i-1);
        P_(:,i+1:N) = P(:,i+1:N);
        
        for j=1:N
            J(:,2*j-1:2*j,i) = [1 0; 0 1; -P_(2,j)  P_(1,j)];
        end
    end
    
end