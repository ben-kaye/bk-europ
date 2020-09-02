% * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
% *                                                               *
% *                 Program by Ben Kaye (c) 2020                  *
% *                         EUROP Project                         *
% *                                                               *
% * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
% *                                                               *
% *        Using CrazyFlie model provided by Aren Karapet         *
% *                        and OSQP Solver                        *
% *                                                               *
% * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

g = 9.81;
m = 27e-3;
Ts = 1e-1;
c2 = 1.3385e-10;
c1 = 6.4870e-6;
maxthrust = 0.1597;

% LTI continuous-time system
A = [   zeros(3),   eye(3),     zeros(3);
        zeros(3),   zeros(3),   [0, g, 0; -g, 0, 0; 0, 0, 0];
        zeros(3),   zeros(3),   zeros(3) ];
    
B = [   zeros(3,4);
        [ [ 0; 0; 1/m ], zeros(3) ];
        [ zeros(3,1), eye(3)] ];
     
% C = [ diag(ones(1,3)), zeros(3,6) ];
C = eye(9);

D = 0;

plant = ss(A,B,C,D);
plant.InputName = 'u';
plant.OutputName = 'y';

% get velocity controller dynamics
[ Ac, Bc, Cc, Dc ] = linmod('crazyflie_controller_ak');

controller = ss(Ac,Bc,Cc,Dc);
controller.InputName = 'e';
controller.OutputName = 'u';

sum = sumblk('e = r-y', 9);

system = connect(plant, controller, sum, 'r', 'y');

d_system = c2d(system, Ts);

%Get state space matrices corresponding to x, y, xdot, ydot, pitch and roll
Ad = d_system.A([1:2, 4:5, 7:8], [1:2, 4:5, 7:8]);

%Input corresponding to desired xdot and ydot
Bd = d_system.B([1:2, 4:5, 7:8], [4, 5]);

