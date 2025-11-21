% !!! Important !!! Disconnect from optitrack
% If your code fails before executing this section, you have to run this
% part by it self otherwise the system will crash

if exist("theClient",'var')
    theClient.Uninitialize; 
    theClient.delete
    clear("theClient")
    disp("Successfully Disconnected OptiTrack")
end