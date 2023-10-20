function [df_sx, df_sy, df_sz] = Gradient(c1, c2, ax, ay, az, sx, sy, sz, ux, uy, uz)
    % Calculate df_sx
    df_sx = (c1 + c2 / (sqrt((ax - sx)^2 + (ay - sy)^2 + (az - sz)^2) * sqrt((sx - ux)^2 + (sy - uy)^2 + (sz - uz)^2))) * (2 * c2 * (ay - sy) / ((sqrt((ax - sx)^2 + (ay - sy)^2 + (az - sz)^2)^(3/2)) * sqrt((sx - ux)^2 + (sy - uy)^2 + (sz - uz)^2)) + 2 * c2 * (-sy + uy) / (sqrt((ax - sx)^2 + (ay - sy)^2 + (az - sz)^2) * (sx - ux)^2 + (sy - uy)^2 + (sz - uz)^2)^(3/2));

    % Calculate df_sy
    df_sy = (c1 + c2 / (sqrt((ax - sx)^2 + (ay - sy)^2 + (az - sz)^2) * sqrt((sx - ux)^2 + (sy - uy)^2 + (sz - uz)^2))) * (2 * c2 * (ay - sy) / ((sqrt((ax - sx)^2 + (ay - sy)^2 + (az - sz)^2)^(3/2)) * sqrt((sx - ux)^2 + (sy - uy)^2 + (sz - uz)^2)) + 2 * c2 * (-sy + uy) / (sqrt((ax - sx)^2 + (ay - sy)^2 + (az - sz)^2) * (sx - ux)^2 + (sy - uy)^2 + (sz - uz)^2)^(3/2));

    % Calculate df_sz
    df_sz = (c1 + c2 / (sqrt((ax - sx)^2 + (ay - sy)^2 + (az - sz)^2) * sqrt((sx - ux)^2 + (sy - uy)^2 + (sz - uz)^2))) * (2 * c2 * (az - sz) / ((sqrt((ax - sx)^2 + (ay - sy)^2 + (az - sz)^2)^(3/2)) * sqrt((sx - ux)^2 + (sy - uy)^2 + (sz - uz)^2)) + 2 * c2 * (-sz + uz) / (sqrt((ax - sx)^2 + (ay - sy)^2 + (az - sz)^2) * (sx - ux)^2 + (sy - uy)^2 + (sz - uz)^2)^(3/2));
end
