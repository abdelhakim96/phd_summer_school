classdef IBSF
    properties
        sys
        K
        P
    end
    
    methods
        function obj = IBSF(sys)
        obj.sys = sys;
        
        % SDP Variables
        E = sdpvar(sys.n);
        Y = sdpvar(sys.m,sys.n);
        
        
% --------- Start Modifying Code Here -----------
%         % Please use the provided variables
%         
%         % Objective: Maximize ellipsoidal volume
%         objective = [];
% 
%         % Constraints
%         % Positive Definite and Lyapunov Decrease
%         constraints = [[TODO, TODO; TODO, TODO]>=0];
% 
%         % State constraints
%         for i=1:obj.sys.nx
%             constraints=[constraints, [[TODO, TODO; TODO, TODO]>=0]];
%         end
%         % Input constraints
%         for j=1:obj.sys.nu
%             constraints=[constraints, [[TODO, TODO; TODO, TODO]>=0]];
%         end
% --------- End Modifying Code Here -----------

        % Solve
        opts = sdpsettings('verbose',1,'solver','sedumi');
        optimize(constraints, objective,opts);

% --------- Start Modifying Code Here -----------
%         obj.P = TODO; %value(.) to access value of optimization variable
%         obj.K = TODO;
% --------- End Modifying Code Here -----------        

        end
        
        function u_s = filter(obj, x, uL)
            % --------- Start Modifying Code Here -----------
%             
%            % Check if predicted state is in the safe set and if input constraints are verified
%             if TODO
%               TODO;                
%             else TODO
%                 TODO;
%             end
            % --------- End Modifying Code Here -----------
        end
    end
end

