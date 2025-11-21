% Function to pulse the motors in the direction of *f*
function fPulse(s1,robot,f,strength)
    pulse = [0;0;0];

    % If any f value is less than 'stick' value, send a pulse to robot
    if any(abs(f) < 60) % 60 is the approximate 'stick' value
        pulse = strength*sign(f);

        for k=1:3
            if(pulse(k) >= 0)
                robot.u.data(2*k-1,1) = 0;
                robot.u.data(2*k,1) = uint8(abs(pulse(k)));
            else
                robot.u.data(2*k-1,1) = uint8(abs(pulse(k)));
                robot.u.data(2*k,1) = 0;
            end
        end
    
        SerialCommunication(s1,robot,192,'u');
    end
end