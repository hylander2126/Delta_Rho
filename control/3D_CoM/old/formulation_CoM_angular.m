clc; clear; close all

%% Trying to deconstruct tau = rxf to isolate r (vector to the CoM of the object)
tic

%% Let's first try Newton-Raphson Method
initial_guess = [20; 3; 0];
givens = {[1;1;0] , [0;0;2]};

% disp('Newton Raphson Method:')
[sol, ~] = NewtonRaphson(initial_guess, givens);
toc
tic
% disp('Levenberg Marquadt Method:')
sol = LevenbergMarquadt(initial_guess, givens);
toc
tic
% disp('Gradient Descent Method:')
sol = GradientDescent(initial_guess, givens);
toc
tic
% disp('Constrained Optimization Method:')
sol = ConstrainedOptim(initial_guess, givens);
toc

return
%% Now trying to recurseivle use NR method by providing solution as initial guess
for i = 1:200
    fprintf('Iteration %i. \n', i);
    [sol,solved] = NewtonRaphson(sol);

    if solved
        break
    end
end

function r = ConstrainedOptim(initial, givens)
    f = givens{1};
    tau = givens{2};
    
    objective = @(r) norm([
        r(2)*f(3) - r(3)*f(2) - tau(1);
        r(3)*f(1) - r(1)*f(3) - tau(2);
        r(1)*f(2) - r(2)*f(1) - tau(3)
    ]);
    
    % Additional magnitude constraint (sphere payload with r = sqrt(2)
    constraint = @(r) deal(r(1)^2 + r(2)^2 + r(3)^2 - sqrt(2), []); % Equality constraint
    
    options = optimoptions('fmincon', 'Algorithm', 'sqp'); % , 'Display', 'iter');
    r = fmincon(objective, initial, [], [], [], [], [], [], constraint, options);
    disp('Solution using Constrained Optimization (fmincon and sqp)');
    disp(r);
end


function r = GradientDescent(initial, givens)
    alpha = 0.1; % Step size
    
    f = givens{1};
    tau = givens{2};
    
    tol = 1e-6;
    max_iter = 1000;
    r = initial;
    
    for iter = 1:max_iter
        F = [
            r(2)*f(3) - r(3)*f(2) - tau(1);
            r(3)*f(1) - r(1)*f(3) - tau(2);
            r(1)*f(2) - r(2)*f(1) - tau(3);
            r(1)^2 + r(2)^2 + r(3)^2 - sqrt(2) % Additional magnitude constraint (sphere payload with r = sqrt(2)
            ];
    
        J = [0 f(3) -f(2) 0;
            -f(3) 0 f(1) 0;
            f(2) -f(1) 0 0;
            2*r(1) 2*r(2) 2*r(3) 0 % Mag constraint
            ];
    
        grad = J' * F; % Gradient of the residual
        r = r - alpha * grad(1:3); % Update step
        if norm(grad) < tol
            fprintf('Converged in %d iterations.\n', iter);
            break;
        end
    end
    disp('Solution using Gradient Descent:');
    disp(r);
end


function r = LevenbergMarquadt(initial, givens)
    % Levenberg-Marquadt specifically for this problem NOT GENERALIZED
    options = optimoptions('lsqnonlin');% , 'Display', 'iter');
    
    f = givens{1};
    tau = givens{2};
    
    F = @(r) [
        r(2)*f(3) - r(3)*f(2) - tau(1);
        r(3)*f(1) - r(1)*f(3) - tau(2);
        r(1)*f(2) - r(2)*f(1) - tau(3);
        r(1)^2 + r(2)^2 + r(3)^2 - sqrt(2) % Additional magnitude constraint (sphere payload with r = sqrt(2)
        ];
    
    r = lsqnonlin(F, initial, [], [], options);
    
    fprintf('Solution using Levenberg Marquadt:\n');
    disp(r)
end



function [r, solutionFound] = NewtonRaphson(initial, givens)
    % Newton-Raphson method specifically for this problem. NOT GENERALIZED N.R. APPROACH
    
    solutionFound = false;
    
    % Givens
    f = givens{1};
    tau = givens{2};
    
    % Initial guesses
    r = initial;
    
    % Tolerance and max iterations
    tol = 1e-6;
    max_iter = 10000;
    
    % Newton-Raphson
    for iter = 1:max_iter
        % Equations
        F = [
            r(2)*f(3) - r(3)*f(2) - tau(1);
            r(3)*f(1) - r(1)*f(3) - tau(2);
            r(1)*f(2) - r(2)*f(1) - tau(3);
            r(1)^2 + r(2)^2 + r(3)^2 - sqrt(2) % Additional magnitude constraint (sphere payload with r = sqrt(2)
            ];
    
        % Jacobian matrix J(r)
        % J = skew(r);
        J = [0 f(3) -f(2) 0;
            -f(3) 0 f(1) 0;
            f(2) -f(1) 0 0;
            2*r(1) 2*r(2) 2*r(3) 0 % Mag constraint
            ];
    
        % Update step
        if cond(J) > 1e12 % check for poor conditioning
            delta_r = -pinv(J)*F;
        else
            delta_r = -J\F;
        end
        r = r + delta_r(1:3);
    
        % Check convergence
        if norm(delta_r) < tol
            solutionFound = true;
            break;
        end
    end
    
    if iter == max_iter
        fprintf('Did not converge. Final value:\n');
        disp(r)
        return
    end
    
    % Solution
    fprintf('Solution using Newton Raphson in %d iterations:\n', iter);
    disp(r);
end