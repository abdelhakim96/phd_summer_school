function [x,u] = SimulateMSD(sys,cntr, x_init, T, w)
if nargin < 5
    w = zeros(size(x_init,1),T);
end

% Set up variables
x = zeros(sys.n,T+1);
u = zeros(sys.m,T);
x(:,1)=x_init;

for i=1:T
    u(:,i) = cntr(x(:,i));
    x(:,i+1) = sys.step(x(:,i),u(:,i),w(:,i));
end

% ==============
% Plot System Trajectory within Constraints
% ==============

figure()
% Plot trajectory 
plot(x(1,:),x(2,:))
hold on
% Plot state constraints
plot(sys.Px,'alpha',0.1)
hold off
xlabel('Position')
ylabel('Velocity')

figure()
subplot(3,1,1)
plot(x(1,1:T))
hold on
plot([1,T],[sys.xmax(1), sys.xmax(1)],'k--')
plot([1,T],[sys.xmin(1), sys.xmin(1)],'k--')
hold off
xlim([1,T])
ylabel('Position')

subplot(3,1,2)
plot(x(2,:))
hold on
plot([1,T],[sys.xmax(2), sys.xmax(2)],'k--')
plot([1,T],[sys.xmin(2), sys.xmin(2)],'k--')
hold off
xlim([1,T])
ylabel('Velocity')

subplot(3,1,3)
plot(u(1,:))
hold on
plot([1,T],[sys.umax(1), sys.umax(1)],'k--')
plot([1,T],[sys.umin(1), sys.umin(1)],'k--')
hold off
xlim([1,T])
ylabel('Input')
xlabel('Time steps')

end

