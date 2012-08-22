function Track = buildMap(S, Sensors)
%BUILDMAP Summary of this function goes here
%   Detailed explanation goes here
    Track = zeros(size(S,1) - 1,3);

    for t = 1:size(Track,1) - 1
        dt = times(t+1) - times(t);
        orientation = T(t,3);
        move = sqrt(sum((S(t,1:2) * dt) .^ 2));
        Track(t+1,1:2) = T(t,1:2) + [cos(orientation) * move sin(orientation) * move];
        Track(t+1,3) = T(t,3) + S(t,3) * dt;

        if States(t,69) > 0
            angle = States(t,1) + 0.5 * pi;
            Rot = [cos(-angle) -sin(-angle); sin(-angle) cos(-angle)];
        else
            angle = States(t,1) - 0.5 * pi;
            Rot = [cos(angle) -sin(angle); sin(angle) cos(angle)];
        end

        temp = Track(t,1:2) + [States(t,69)*12 * cos(orientation) States(t,69)*12 * sin(orientation)];
        temp = Rot * (temp - Track(t,1:2))';
        R(t,1:2) = temp' + Track(t,1:2);

        %R(t,:) = T(t,1:2) + [States(t,69) * 12 * cos(rotation) States(t,69) * 12 * sin(rotation)];
    end

end

