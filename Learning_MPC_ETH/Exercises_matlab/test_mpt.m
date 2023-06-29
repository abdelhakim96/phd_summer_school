% EECI Graduate School: Learning-Based Predictive Control
% Lukas Hewing, Alexandre Didier, Jérôme Sieber
% Copyright (C) 2020, ETH Zurich, {lhewing, jsieber, adidier}@ethz.ch
% This code is only made available for students taking the EECI Learning-Based Predictive Control graduate school and is NOT to be distributed.
% Requires MPT Toolbox: https://www.mpt3.org/
% Requires MATLAB Optimization Toolbox
% 

%% Test QP
x = sdpvar(2,1);
objective = (x - [2;1])'*(x - [2;1]);
constraints = [[1,0; -1,0; 0,-1; 0,1]*x <= 1];

options = sdpsettings('verbose',1,'solver','quadprog');
%options = sdpsettings('verbose',1,'solver','mosek');

% Solve the problem
sol = optimize(constraints,objective,options);

if sol.problem~=0
    warning(yalmiperror(sol.problem))
else
    value(x)
end

%% Test SOCP
x = sdpvar(2,1);
objective = (x - [2;1])'*(x - [2;1]);
constraints = [norm(x,2) <= 1];

options = sdpsettings('verbose',1,'solver','sedumi');
%options = sdpsettings('verbose',1,'solver','mosek');

% Solve the problem
sol = optimize(constraints,objective,options);

if sol.problem~=0
    warning(yalmiperror(sol.problem))
else
    value(x)
end

%% Test SDP
P = sdpvar(2);
objective = -logdet(P);
constraints = [P <= eye(2)];

options = sdpsettings('verbose',1,'solver','sedumi');
%options = sdpsettings('verbose',1,'solver','mosek');

% Solve the problem
sol = optimize(constraints,objective,options);

if sol.problem~=0
    warning(yalmiperror(sol.problem))
else
    value(P)
end

%% Test MPT Polyhedra
P1 = Polyhedron([eye(2); -eye(2)],ones(4,1));
P2 = Polyhedron([eye(2); -eye(2)],0.5* ones(4,1));
plot(P1-P2)