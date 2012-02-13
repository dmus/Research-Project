function convert(filename)
%CONVERT Convert log of race to a states and action matrices saved in a
%file
%   FILE (without extension, assumed extension is '.log') is read,
%   resulting matrices are saved to FILE.mat
    fid = fopen(strcat(filename, '.log'), 'r');

    States = zeros(0, 79);
    Actions = zeros(0, 7); 

    % Read first two lines
    line1 = fgetl(fid);
    line2 = fgetl(fid);

    % File pointer is now at beginning of third line
    line = fgetl(fid);
    while ~strcmp(line, 'Received: ***shutdown*** ')
        mode = sscanf(line, '%s', 1);

        if strcmp(mode, 'Received:')
            s = sscanf(line, 'Received: (angle %f)(curLapTime %f)(damage %f)(distFromStart %f)(distRaced %f)(fuel %f)(gear %f)(lastLapTime %f)(opponents %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f)(racePos %f)(rpm %f)(speedX %f)(speedY %f)(speedZ %f)(track %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f)(trackPos %f)(wheelSpinVel %f %f %f %f)(z %f)(focus %f %f %f %f %f)');
            States = [States; s'];
        elseif strcmp(mode, 'Sending:')
            a = sscanf(line, 'Sending: (accel %f) (brake %f) (clutch %f) (gear %f) (steer %f) (meta %f) (focus %f)');
            Actions = [Actions; a'];
        end

        line = fgetl(fid);
    end

    fclose(fid);

    save(strcat(filename, '.mat'), 'States', 'Actions');
end

