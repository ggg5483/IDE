% Real time data collection example
%
% This script is implemented as a function so that it can
%   include sub-functions
%
% This script can be modified to be used on any platform by changing the
% serialPort variable.
% Example:-
% On Linux:     serialPort = '/dev/ttyS0';
% On MacOS:     serialPort = '/dev/tty.KeySerial1';
% On Windows:   serialPort = 'COM1';
%
% To run: 
% plot_cameras_serial()
%
% TODO: Complete these sections
%

function plot_cameras_serial

trace = zeros(1, 128);  % Stored Values for Raw Input
plt = tiledlayout(3,1); % Plot Layout 

ax1 = nexttile;
ax2 = nexttile;
ax3 = nexttile;

try
    while (true)
        trace = readData(trace);
        smoothtrace = smoothData(trace);  % Smoothed data
        bintrace = edgeData(smoothtrace); % Edge detection

        if ~isvalid(plt), break;   end
        if isvalid(ax1), cla(ax1); end
        if isvalid(ax2), cla(ax2); end
        if isvalid(ax3), cla(ax3); end

        plotdata(trace, smoothtrace, bintrace, plt, ax1, ax2, ax3);
    end
catch
    close(plt.Parent);
end

disp('Exiting...');

end % plot_cameras_serial

%*****************************************************************************************************************
%*****************************************************************************************************************
function trace = readData(trace)
    % Initialize Serial Object
    persistent camera
    serialPort = "COM9";
    serialBaudrate = 9600;

    if isempty(camera) || ~isvalid(camera)
        camera = serialport(serialPort, serialBaudrate);
        camera.FlowControl = "software";
        camera.configureTerminator("CR/LF");
    end

    count = 1;

    % Read data from serial object for trace
    while(true)
        % disp("Searching for start..");
        val = strtrim(readline(camera));
        if (strcmp(val, "-1") == 0) % if not the start
            % disp(val); % words, not numbers
            continue;
        end
        % disp("FOUND START!");
        while (true)
            val = strtrim(readline(camera));
            if strcmp(val, "-2")
                break;
            else
                num = str2double(val);
                if ~isnan(num) && count <= 128
                    trace(count) = num;
                    count = count + 1;
                end
            end
        end
        break; % hit from val=-2
    end
end

% TODO: Complete the functions below

function data = smoothData(data)
    % TODO: 5-point Averager
    %   For loop or movmean()

    sm = data;
    for i = 3:126
        sm(i) = mean(data(i-2:i+2));
    end
    data = sm;

end

function data = edgeData(data)
    for i = 1:128
        % TODO: Edge detection (binary 0 or 1)

        if i == 1
            deriv = 0;
        else
            deriv = data(i) - data(i-1);
        end

        if deriv > max(data)*0.15
            data(i) = 1;
        else
            data(i) = 0;
        end

    end
end

function plotdata(trace, smoothtrace, bintrace, plt, ax1, ax2, ax3)
    % TODO: Plot data
    %   plot(ax, trace)

    plot(ax1, trace, 'b');
    ylim(ax1, [0 4095]);

    plot(ax2, smoothtrace, 'r');
    ylim(ax2, [0 4095]);

    stem(ax3, bintrace, 'k', 'Marker', 'none');
    ylim(ax3, [-0.2 1.2]);

    refreshdata
    drawnow
end
