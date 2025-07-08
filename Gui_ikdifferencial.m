% Communicaiton between Arduino and MATLAB
%   @author         Alejandro Granados
%   @organisation   King's College London
%   @module         Medical Robotics Hardware Development
%   @year           2023

%close all
clear all

% declare global variables
%   s           serial port communication
%   hInput1     input widget 1
%   hInput1     input widget 2
%   hPlot       plot widget

%   hFig        figure widget
%   hTimer      continuous timer
%   c           command type
%   y1          data stream series 1
%   y2          data stream series 2
global s hInput1 hInput2 hPlot hFig hTimer c y1 y2

%% Set up
% Create serial port object
s = serialport('/dev/cu.usbmodem1101', 115200);
configureTerminator(s,"CR/LF");
s.UserData = struct("Data",[],"Count",1)

% Create GUI
hFig = figure;
% Define the new coordinate range
xMin = 0;
xMax = 156;
yMin = 0;
yMax = 156;

% Create input field for sending commands to microcontroller
hInput1 = uicontrol('Style', 'edit', 'Position', [20, 20, 100, 25]);
hInput2 = uicontrol('Style', 'edit', 'Position', [120, 20, 100, 25]);

% Create button for sending commands
hSend = uicontrol('Style', 'pushbutton', 'String', 'Send', 'Position', [20, 50, 100, 25], 'Callback', @sendCommand);

% Create plot area
hPlot = axes('Position', [0.2, 0.2, 0.6, 0.6],'XLim', [xMin, xMax], 'YLim', [yMin, yMax]);
hPlot.ButtonDownFcn = @plotClickCallback;

% Set up variables for real-time plotting
c = [];
y1 = [];
y2 = [];
t0 = now;
t1 = 0;
t2 = 0;
% Set up timer for continuously receiving data from microcontroller
hTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, 'TimerFcn', @readDataTimer);
start(hTimer);
hFig.CloseRequestFcn = @closeGUI;



%% Callback function for sending commands
function sendCommand(~, ~)
    global s hInput1 hInput2 t1 t2


    % Get values from input fields as strings but convert to numbers
    px = str2num(get(hInput1, 'String'));
    py = str2num(get(hInput2, 'String'));

    % Validate input
    if numel(px) ~= 1 || numel(py) ~= 1
        return; % input field must contain one value
    end
    
    Ik = ik_differential(t1, t2, px, py);
    for i = 1:size(Ik, 1)
        angle1 = Ik(i, 1);
        angle2 = Ik(i, 2);
        
        % Format command values as a string, e.g., C40.0,3.5;
        cmdStr = sprintf("C%.2f,%.2f;", angle1, angle2);

        % Send command string to microcontroller
        write(s, cmdStr, "string");
        disp(cmdStr);
        s.UserData.Count = s.UserData.Count + 1;
        
        % Wait until joint angles are close to the target
        targetAngles = [angle1, angle2];
        while ~areJointAnglesClose(s, targetAngles)
            pause(0.1);
        end
        
    end
end
%% Callback function for capturing 2D position on plot click
function plotClickCallback(~, eventdata)
global s t1 t2

    % Get the clicked point
    clickedPoint = eventdata.IntersectionPoint;
    
    % Extract x and y coordinates
    px = clickedPoint(1);
    py = clickedPoint(2);
    
    % Validate input
    if numel(px) ~= 1 || numel(py) ~= 1
        return; % input field must contain one value
    end
    
    Ik = ik_differential(0, 0, px, py);
    for i = 1:size(Ik, 1)
        angle1 = Ik(i, 1);
        angle2 = Ik(i, 2);

        % Format command values as a string, e.g., C40.0,3.5;
        cmdStr = sprintf("C%.2f,%.2f;", t1, t2);

        % Send command string to microcontroller
        write(s, cmdStr, "string");
        disp(cmdStr);
        s.UserData.Count = s.UserData.Count + 1;
        
        % Wait until joint angles are close to the target
        targetAngles = [angle1, angle2];
        while ~areJointAnglesClose(s, targetAngles)
            pause(0.1);
        end
    end
end


%% Function to read joint angles from the serialport object
function [c, current_t1, current_t2] = readJointAngles(s)
    % Read the ASCII data from the serialport object.
    dataStr = readline(s);
    if isempty(dataStr)
        pause(0.1);
        c = [];
        current_t1 = NaN;
        current_t2 = NaN;
    else
        % Parse data values from string
        data = sscanf(dataStr, "%c%f,%f");
        c = data(1);
        current_t1 = data(2);
        current_t2 = data(3);
    end
end
%% Callback function fo reading time series values from microcontroller
function readDataTimer(~, ~)
global s hPlot c y1 y2
    [c, current_t1, current_t2] = readJointAngles(s);
   
    t1 = current_t1;
    t2 = current_t2;
   
    T = forward_kinematics(r1,r2,t1,t2);
    y1 = [y1, T(1,4)];
    y2 = [y2, T(2,4)];

    % configure callback 
    configureCallback(s, "off");
    clear data;
    % Downsampling
    downsampling_factor = 200;  
     if mod(length(y1), downsampling_factor) == 0
     % Plot only every nth data point      
      plot(hPlot, y1(end-downsampling_factor+1:end), y2(end-downsampling_factor+1:end), 'o', 'MarkerSize', 16, 'MarkerFaceColor', 'red');
      grid on
     % Adjust tick values and labels to match the new coordinate range
   
  end
end
%% Function to check if joint angles are close to the targe
function close = areJointAnglesClose(s, target_t1, target_t2)
    [c, current_t1, current_t2] = readJointAngles(s);
    angleThreshold = 0.2;

    % Check if joint angles are close to the target
    close = abs(current_t1 - target_t1) < angleThreshold && abs(current_t2 - target_t2) < angleThreshold;
end

%% Callback function for closing the GUI
function closeGUI(~, ~)
global s hFig hTimer

    % Stop timer
    stop(hTimer);
    delete(hTimer);
    
    % Close serial port
    delete(s);
    
    % Close GUI
    delete(hFig);
end