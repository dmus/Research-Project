function plotScans(C, R, rotation)
%PLOTSCANS Current scan C and reference scan R are plotted in the same
%Cartesian coordinate system.
%   Specify ROTATION in radians (optional) to rotate the current scan around its
%   position.
    
    if nargin > 2
        Rot = [cos(rotation) -sin(rotation); sin(rotation) cos(rotation)];
    else
        Rot = eye(2);
    end

    % First plot the reference position and the current position
    plot(0,0,'dm', C.position(1),C.position(2),'dc');
    hold on;
    
    % Now the edges for the reference scan, first the right
    first = R.segments(1,1);
    last = R.segments(1,2);
    plot(R.points(first:last,1),R.points(first:last,2),'+r');
    
    % Interpolate to get nice curve
    xx = R.points(first,1):.01:R.points(last,1);
    yy = spline(R.points(first:last,1),R.points(first:last,2),xx);
    plot(xx,yy,'-r');
    
    % Same for left edge of reference scan
    first = R.segments(2,1);
    last = R.segments(2,2);
    plot(R.points(first:last,1),R.points(first:last,2),'+r');
    
    % Interpolate to get nice curve
    xx = R.points(last,1):.01:R.points(first,1);
    yy = spline(R.points(first:last,1),R.points(first:last,2),xx);
    plot(xx,yy,'-r');
    
    % Translate and rotate current scan
    currentScan = bsxfun(@plus,C.points,C.position);
    currentScan = (Rot * currentScan')';
    
    % Now the edges for the current scan, first the right
    first = C.segments(1,1);
    last = C.segments(1,2);
    plot(currentScan(first:last,1),currentScan(first:last,2),'+b');
    
    % Interpolate to get nice curve
    xx = currentScan(first,1):.01:currentScan(last,1);
    yy = spline(currentScan(first:last,1),currentScan(first:last,2),xx);
    plot(xx,yy,'-b');
    
    % Same for left edge of current scan
    first = C.segments(2,1);
    last = C.segments(2,2);
    plot(currentScan(first:last,1),currentScan(first:last,2),'+b');
    
    % Interpolate to get nice curve
    xx = currentScan(last,1):.01:currentScan(first,1);
    yy = spline(currentScan(first:last,1),currentScan(first:last,2),xx);
    plot(xx,yy,'-b');
    
    % Now we are finished
    hold off;
end

