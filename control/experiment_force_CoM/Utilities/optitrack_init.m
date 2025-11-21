%% Initializing the reading OptiTrack data in Matlab
% Please turn on the system and set Rigid body first
% And enable "stream rigid body" 

%% Connection init
% Add NatNet .NET assembly so that Matlab can access its methods, delegates, etc.
% Note : The NatNetML.DLL assembly depends on NatNet.dll, so make sure they
% are both in the same folder and/or path if you move them.
dllPath = fullfile('\\research.wpi.edu\srl\OptiTrack\MATLAB Connection\NatNet_SDK_2.8\NatNetSDK\lib\x64\NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath);
theClient = NatNetML.NatNetClientML(0); % Input = iConnectionType: 0 = Multicast, 1 = Unicast

% Connect to an OptiTrack server (Motive)
HostIP = char('130.215.181.152');
theClient.Initialize(HostIP, HostIP);

disp('OptiTrack NatNet connected!')