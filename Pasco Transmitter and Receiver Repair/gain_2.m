% define original index vector and sequence
clc
clear
nx = -3:7
x = zeros(size(nx));
x(nx==0) = 2;
x(nx==2) = 1;
x(nx==3) = -1;
x(nx==4) = 3

% define transformed index vectors and signals
% y1[n] = x[n-2]  -> ny1 = nx + 2,  y1 = x
ny1 = nx + 2
y1  = x

% y2[n] = x[n+4]  -> ny2 = nx - 4,  y2 = x
ny2 = nx - 4
y2  = x

% y3[n] = x[-n]   -> ny3 = -nx,     y3 = x
ny3 = -nx
y3  = x

% y4[n] = x[-n+1] -> ny4 = -nx + 1,  y4 = x
ny4 = -nx + 1
y4  = x

% display the vectors (optional)
disp('nx = ');  disp(nx);
disp('x  = ');  disp(x);

disp('ny1 = '); disp(ny1);
disp('y1  = '); disp(y1);

disp('ny2 = '); disp(ny2);
disp('y2  = '); disp(y2);

disp('ny3 = '); disp(ny3);
disp('y3  = '); disp(y3);

disp('ny4 = '); disp(ny4);
disp('y4  = '); disp(y4);

% plotting
figure;
subplot(3,2,1); stem(nx, x, 'filled'); xlabel('n'); ylabel('x[n]'); title('x[n]');
subplot(3,2,2); stem(ny1, y1, 'filled'); xlabel('n'); ylabel('y1[n]'); title('y1[n] = x[n-2]');
subplot(3,2,3); stem(ny2, y2, 'filled'); xlabel('n'); ylabel('y2[n]'); title('y2[n] = x[n+4]');
subplot(3,2,4); stem(ny3, y3, 'filled'); xlabel('n'); ylabel('y3[n]'); title('y3[n] = x[-n]');
subplot(3,2,5); stem(ny4, y4, 'filled'); xlabel('n'); ylabel('y4[n]'); title('y4[n] = x[-n+1]');
