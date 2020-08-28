path_number = size(stack,1)
            for h = 1 : 1 : path_number
             lynxServo(stack(h,:));
             pause(1);
            end