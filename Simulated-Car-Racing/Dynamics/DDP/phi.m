function f = phi(s, map, ref)
%PHI Summary of this function goes here
%   Detailed explanation goes here

    mapSize = size(map,1);

    trackWidth = 12;
    trackPos = ref(3) * (trackWidth / 2);
    alpha = -ref(2);

    if trackPos > 0 
        track = [abs(trackPos) * cos(alpha - 0.5*pi); abs(trackPos) * sin(alpha - 0.5*pi)];
    else
        track = [abs(trackPos) * cos(alpha + 0.5*pi); abs(trackPos) * sin(alpha + 0.5*pi)];
    end
    
    index = find(map(:,1) > ref(1), 1);
    if size(index,1) == 0
        index = 1;
    end
    prev_index = index - 1;
    
    if prev_index == 0
        prev_index = size(map,1);
    end
    distance_to_p1 = ref(1) - map(prev_index,1);
    distance_to_p2 = map(index,1) - ref(1);
    
    p1 = track + [distance_to_p1 * cos(alpha + pi); distance_to_p1 * sin(alpha + pi)];
    p2 = track + [distance_to_p2 * cos(alpha); distance_to_p2 * sin(alpha)];
    
    p = s(1:2);
    r = p - p1;
    dp = p2 - p1;
    u = (r' * dp) / (dp' * dp);
    
    while u > 1
        alpha = alpha + map(prev_index,3);
        p1 = p2;
        p2 = p2 + [map(index,2) * cos(alpha); map(index,2) * sin(alpha)];
        
        r = p - p1;
        dp = p2 - p1;
        u = (r' * dp) / (dp' * dp);
    
        index = index + 1;
        
        if index > size(map,1)
            index = 1;
        end
        
        prev_index = index - 1;
        if prev_index == 0
            prev_index = size(map,1);
        end
    end
    
    f = zeros(1,3);
    
    % Distance from track axis
    relative_cte = (r(2) * dp(1) - r(1) * dp(2)) / (dp' * dp);
    absolute_cte = relative_cte * sqrt(dp' * dp);
    f(3) = absolute_cte / (0.5 * trackWidth);
    
    % Angle with respect to track axis
    f(2) = s(3) - (alpha + u * map(prev_index,3));
    
    % Distance to go along track axis
    %f(1) = map(prev_index,1) + u * (map(index,1) - map(prev_index,2));
    f(1) = map(prev_index,1) + u * map(prev_index,2);
end

