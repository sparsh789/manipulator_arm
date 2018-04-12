clear all
clc
robot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',3);
L1 = .75;
L2 = .75;
body = robotics.RigidBody('link1');
joint = robotics.Joint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');
body = robotics.RigidBody('link2');
joint = robotics.Joint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');
body = robotics.RigidBody('tool');
joint = robotics.Joint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');
showdetails(robot)
%trajectory

% center = [0.3 0.1 0];
% radius = 0.15;
% theta = t*(2*pi/t(end));
% points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];
t = (0:0.2:10);
[x1,y1] = getpts;
[x2,y2] = getpts;
xi = [x1,y1,0];
xf = [x2,y2,0];
count = length(t);
d = t(end);
N = length(t);
a = repmat((xf - xi), N, 1);
b = repmat((10 * (t/d).^3 - 15 * (t/d).^4 + 6 * (t/d).^5)',1,3) ;
points = repmat(xi, N, 1) + a .* b;
 
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);

ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-1 1 -1 1])

framesPerSecond = 15;
r = robotics.Rate(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end

