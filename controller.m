%feedback linearization
%cheng, sheng-wen

close all;

iteration=10000
dt = 0.001

%system constants
a = 2
b = 4

%gain of feedback linearization controller
k_fb_linerize =50;
%gain of high gain controller
k_high_gain = 50;
%gains of high frequency controller
k_high_freq_1 = 50;
k_high_freq_3 = 10;

%control input
u = zeros(1, iteration);
u_feedback_linearize = zeros(1, iteration);
u_high_gain = zeros(1, iteration);
u_high_freq = zeros(1, iteration);

%system dynamics
t = 1:iteration;
x = zeros(1, iteration);
x_dot = zeros(1, iteration);
x_double_dot = zeros(1, iteration);

%desired trajectory
x_desired = zeros(1, iteration);
x_desired_dot = zeros(1, iteration);
x_desired_double_dot = zeros(1, iteration);

%initial conditions
x(1) = 0;
x_desired(1) = 0;
x_desire_dot(1) = 0;
x_desire_double_dot(1) = 0;

alpha = 1; %gain of filter tracking error

e_dot = zeros(1, iteration); %error = x - x_desired

for i=2:iteration-1
    %generate desired trajectory
    x_desired(i) = sin(i*dt);
    x_desire_dot(i) = cos(i*dt);
    x_desire_double_dot(i) = -sin(i*dt);
   
    e_dot(i) = x_dot(i) - x_desire_dot(i);
    r = e_dot(i) + alpha*e_dot(i);
    
    %calculate control input using feedback linearization method
    u_feedback_linearize(i) = ...%-(a*x(i) + b*cos(x(i))) 
        + (x_desire_double_dot(i)) ...
         -(alpha*e_dot(i)) - (k_fb_linerize*r);
     
    %calculate control input using high gain controller
    u_high_gain(i) = -(k_high_gain*r);
    
    %calculate control input using high frequency controller
    if r == 0
        sgn = 0;
    elseif r > 0
        sgn = 1;
    elseif r < 0
        sgn = -1;
    end    
    u_high_freq(i) = -(k_high_freq_1*r + k_high_freq_3*sgn);
    
    %select one controlling strategy
    u(i) = u_feedback_linearize(i);
    u(i) = u_high_gain(i);
    u(i) = u_high_freq(i);
    
    %dynamics of the system
    x_double_dot(i+1) = u(i);%a*x(i) + b*cos(x(i)) + u(i);
    x_dot(i+1) = x_dot(i) + x_double_dot(i+1) * dt;
    x(i+1) = x(i) + x_dot(i+1) * dt;
end

%plot(t, x_desire_dot)
%plot(t, x_desire_double_dot)

figure(1)
plot(t, x)
hold on;
plot(t, x_desired)

figure(2)
plot(t, u)