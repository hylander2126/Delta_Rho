%% Initializing the reading OptiTrack data in Matlab
% Please turn on the system and set Rigid body first
% And enable "stream rigid body" 

%% Connection init
% Add NatNet .NET assembly so that Matlab can access its methods, delegates, etc.
% Note : The NatNetML.DLL assembly depends on NatNet.dll, so make sure they
% are both in the same folder and/or path if you move them.
dllPath = fullfile('\\research.wpi.edu\srl\OptiTrack\MATLAB Connection\NatNet_SDK_2.8\NatNetSDK\lib\x64\NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath);
theClient = NatNetML.NatNetClientML(1); % Input = iConnectionType: 0 = Multicast, 1 = Unicast

% Connect to an OptiTrack server (Motive)
HostIP = char('srl-06.admin.wpi.edu'); %'130.215.182.65');
theClient.Initialize(HostIP, HostIP);


test = theClient.GetLastFrameOfData;
if ~test.iFrame
    disp('FAILED TO CONNECT!')
else
    disp('OptiTrack NatNet connected!')
end