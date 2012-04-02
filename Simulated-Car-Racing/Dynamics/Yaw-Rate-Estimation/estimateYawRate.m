function yawrates = estimateYawRate(States)
%ESTIMATEYAWRATE Summary of this function goes here
%   Detailed explanation goes here
    
    wheelRadius = 0.3179;
    LeftSpeed = States(:,71) .* wheelRadius;
    RightSpeed = States(:,70) .* wheelRadius;

    %[LeftSpeed RightSpeed States(:,47) ./ 3.6]

    L1 = LeftSpeed .* 0.02;
    L2 = RightSpeed .* 0.02;

    r2 = 1.94 ./ (1 - L2 ./ L1);
    r1 = r2 - 1.94;

    angle1 = L1 ./ r1;
    angle2 = L2 ./ r2;

    Rot = [angle1 .* 50 angle2 .* 50];

    yawrates = mean(Rot,2);

end

