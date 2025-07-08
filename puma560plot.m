% Define the transformation matrices
P1 = @(t1) [1, 0, 0, 0;
             0, cosd(t1), -sind(t1), sind(t1);
             0, sind(t1), cosd(t1), 1 - cosd(t1);
             0, 0, 0, 1];

P2 = @(t2) [1, 0, 0, 0;
             0, cosd(t2), -sind(t2), 1 - cosd(t2) + sind(t2);
             0, sind(t2), cosd(t2), 1 - cosd(t2) - sind(t2);
             0, 0, 0, 1];

P3 = [0, 0, -1, -1;
      0, 1, 0, 1;
      1, 0, 0, 0;
      0, 0, 0, 1];

P4 = @(t1) [cosd(t1), -sind(t1), 0, -sind(t1)+cosd(t1)-1;
            sind(t1),  cosd(t1), 0, sind(t1)+cosd(t1)-1;
            0, 0, 1, 0;
            0, 0, 0, 1];

P5 = @(t2) [cosd(t2), -sind(t2), 0, cosd(t2)-1;
            sind(t2), cosd(t2), 0, sind(t2);
            0, 0, 1, 0;
            0, 0, 0, 1];

P6 = eye(4, 4);

% Generate values for t1 and t2
t1_values = deg2rad(-45:5:90);  % Range: [-π/4, π/2] in 5-degree increments
t2_values = deg2rad(-90:5:90);  % Range: [-π/2, π/2] in 5-degree increments

% Create a 3D plot in the y-z plane
figure;
xlabel('X-axis');
ylabel('Y-axis');
zlable('Z-axis');
hold on;

% Plot the origin of the tool frame for each combination of t1 and t2
for t1 = t1_values
    for t2 = t2_values
        P = P1(rad2deg(t1)) * P2(rad2deg(t2)) * P3;
        tool_origin = P(1:3, 4);  % Extracting the translation part from the matrix
        plot( ,tool_origin(2), tool_origin(3), 'bo');
        
        Pn = P4(rad2deg(t1)) * P5(rad2deg(t2)) * P6;
        tool_origin2 = Pn(1:3, 4);  % Extracting the translation part from the matrix
        plot(tool_origin2(1), tool_origin2(2),  'rx');
    end
end

hold off;
