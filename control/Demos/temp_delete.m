clc; clear; close all;

s1 = serialport('COM4',9600,'DataBits',8,'StopBits',1);

rob = Robot(5);
u_r = [0; 0; 3000];

rob = rob.setTorques(u_r);

while(true)

    SerialCommunication(s1, rob, 192, 'u')

end