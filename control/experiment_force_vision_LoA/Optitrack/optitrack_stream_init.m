% Using natnet.p wrapper for MATLAB
addpath('Optitrack\');

optiClient = natnet();

%% Provide host IP and streaming type
optiClient.HostIP = '130.215.182.65';
optiClient.ConnectionType = 'Unicast';

%% Get the current IP (of this PC). Alternatively, update this manually.
[~,result] = system('ipconfig'); % Only works on Windows
ipLine = regexp(result, 'IPv4 Address[^\n]*', 'match');
ipv4 = regexp(ipLine{1}, '(?<=:\s)\d+\.\d+\.\d+\.\d+', 'match', 'once');

%% UNCOMMENT FOR MANUAL ENTRY
ipv4 = '130.215.222.12';

optiClient.ClientIP = ipv4;

fprintf('Client IP: %s \n',  string(optiClient.ClientIP))

clear result ipLine ipv4

%% Connect using provided connection info

disp('If asked for dll file, choose "NatNetML.dll" from proper MATLAB folder')

optiClient.connect
optiClient.getFrameRate

if optiClient.IsConnected
    fprintf('\n\nOptitrack Wrapper Connected!\n')
else
    fprintf("\n\nSomething went wrong while connecting to Motive... Please try again.\n")
end
