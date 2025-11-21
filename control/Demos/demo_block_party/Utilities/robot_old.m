classdef robot < handle
    % We make this class inherit the "handle" superclass so that we can
    % update the class within the callback function inside "getKey"...
    % crazy IK
    
    properties
    ID 
    Kp 
    u
    stop = false;
    
    %% Decentralized Construct Jacobian from Agent Position "P" on Object
    J % = construct_jacobian(P(:,1));
    pJ_ % = pinv(robot(n).J);   
    end
    
    methods
        function obj = robot(ID, K)
            obj.ID = ID;
            obj.Kp = K;
            obj.u = uClass;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

