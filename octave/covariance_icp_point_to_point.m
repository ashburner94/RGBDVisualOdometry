% clear everything
clc;
clear;

% Import the needed packages if running in octave 
if(exist('OCTAVE_VERSION'))
  pkg load symbolic;
end

% Translation vector
x = sym('x', 'real');
y = sym('y','real');
z = sym('z','real');
T = [x; y; z];

% Rotation matrix 
roll = sym('roll', 'real');
pitch = sym('pitch', 'real');
yaw = sym('yaw', 'real');
r00 = cos(pitch)*cos(yaw);                              r01 = cos(pitch)*sin(yaw);                              r02 = -sin(pitch);
r10 = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw); r11 = sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw); r12 = sin(roll)*cos(pitch);
r20 = cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw); r21 = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); r22 = cos(roll)*cos(pitch);
R = [r00, r01, r02;
     r10, r11, r12;
     r20, r21, r22];

% Datapoint 
dataix = sym('dataix', 'real');
dataiy = sym('dataiy', 'real');
dataiz = sym('dataiz', 'real');
data = [dataix; dataiy; dataiz];

% Model point 
modelix = sym('modelix', 'real');
modeliy = sym('modeliy', 'real');
modeliz = sym('modeliz', 'real');
model = [modelix; modeliy; modeliz];

% Calculate G and its derivatives
G = (R * data + T - model);
dG_dx = jacobian(G, [x, y, z, roll, pitch, yaw]);

% Calculate the derivatives of J
dJ_dx = dG_dx' * 2 * G;
d2J_dx2 = jacobian(dJ_dx, [x, y, z, roll, pitch, yaw]);
d2J_dzdx = jacobian(dJ_dx, [data; model]);

% Print out the C-code for d2J_dx2
file = 'd2J_dx2.txt';
delete(file);
diary(file);
diary on;
for row=1:6
  for col=1:6
    disp(sprintf('d2J_dx2_temp(%d, %d) = %s;', row-1, col-1, ccode(d2J_dx2(row, col))));
  end;
end;
diary off;

% Print out the C-code for d2J_dzdx
file = 'd2J_dzdx.txt';
delete(file);
diary(file);
diary on;
for row=1:6
  for col=1:6
    disp(sprintf('d2J_dzdx_temp(%d, %d) = %s;', row-1, col-1, ccode(d2J_dzdx(row, col))));
  end;
end;
diary off;
