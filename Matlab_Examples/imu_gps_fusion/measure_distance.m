% function velocities = measure_distance(lat1, lon1, alt1, lat2, lon2, alt2)
%     time = 1;
%     latVelocity = (lat2-lat1)/time;
%     lngVelocity = (lon2-lon1)/time;
%     altVelocity = (alt2-alt1)/time;
%     velocities = [latVelocity, lngVelocity, altVelocity];
% end

function velocities = measure_distance(lat1, lon1, alt1, lat2, lon2, alt2, time)
    dLat = lat2 * pi / 180 - lat1 * pi / 180;
    dLon = lon2 * pi / 180 - lon1 * pi / 180;
    latVelocity = dLat * time;
    lngVelocity = dLon * time;
    altVelocity = (alt2-alt1) * time;
    velocities = [latVelocity, lngVelocity, altVelocity];
end