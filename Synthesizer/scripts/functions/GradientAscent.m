function [x, y, z, ind] = GradientAscent(range,c1, c2, ax, ay, az, ux, uy, uz, sx, sy, sz, precision, max_iters, rate)
    x = 0;
    y = 0;
    z = 0;
    maximum = 10000000;

    % Wall [0, y, z]
    iters = 0;
    previous_step_size = 1.0;
    total_gradient_sx = 0.0;
    total_gradient_sy = 0.0;
    total_gradient_sz = 0.0;

    % Gradient Ascent algorithm
    while previous_step_size > precision && iters < max_iters
        prev_sx = sx;
        prev_sy = sy;
        prev_sz = sz;

        % Calculate gradients
        [gradient_sx, gradient_sy, gradient_sz] = Gradient(c1, c2, ax, ay, az, sx, sy, sz, ux, uy, uz);

        total_gradient_sx = total_gradient_sx + gradient_sx;
        total_gradient_sy = total_gradient_sy + gradient_sy;
        total_gradient_sz = total_gradient_sz + gradient_sz;

        % Update variables
        sx = -range/2;
        sy = max(0.0, min(sy + rate * total_gradient_sy, range));
        sz = max(0.0, min(sz + rate * total_gradient_sz, 2.0));

        % Calculate step size
        previous_step_size = abs(sx - prev_sx) + abs(sy - prev_sy) + abs(sz - prev_sz);
        iters = iters + 1;
    end

    % Calculate the final value of f
    f = (c1 + (c2 / (sqrt((ax - sx)^2 + (ay - sy)^2 + (az - sz)^2) * sqrt((sx - ux)^2 + (sy - uy)^2 + (sz - uz)^2))))^2;
    if f < maximum
        ind=3;
        maximum = f;
        x = sx;
        y = sy;
        z = sz;
    end

    % Wall [15, y, z]
    iters = 0;
    previous_step_size = 1.0;
    total_gradient_sx = 0.0;
    total_gradient_sy = 0.0;
    total_gradient_sz = 0.0;

    % Gradient Ascent algorithm
    while previous_step_size > precision && iters < max_iters
        prev_sx = sx;
        prev_sy = sy;
        prev_sz = sz;

        % Calculate gradients
        [gradient_sx, gradient_sy, gradient_sz] = Gradient(c1, c2, ax, ay, az, sx, sy, sz, ux, uy, uz);

        total_gradient_sx = total_gradient_sx + gradient_sx;
        total_gradient_sy = total_gradient_sy + gradient_sy;
        total_gradient_sz = total_gradient_sz + gradient_sz;

        % Update variables
        sx = range/2;
        sy = max(0.0, min(sy + rate * total_gradient_sy, range));
        sz = max(0.0, min(sz + rate * total_gradient_sz, 2.0));

        % Calculate step size
        previous_step_size = abs(sx - prev_sx) + abs(sy - prev_sy) + abs(sz - prev_sz);
        iters = iters + 1;
    end

    % Calculate the final value of f
    f = (c1 + (c2 / (sqrt((ax - sx)^2 + (ay - sy)^2 + (az - sz)^2) * sqrt((sx - ux)^2 + (sy - uy)^2 + (sz - uz)^2))))^2;
    if f < maximum
        maximum = f;
        ind=4;
        x = sx;
        y = sy;
        z = sz;
    end

    % Wall [z, 0, z]
    iters = 0;
    previous_step_size = 1.0;
    total_gradient_sx = 0.0;
    total_gradient_sy = 0.0;
    total_gradient_sz = 0.0;

    % Gradient Ascent algorithm
    while previous_step_size > precision && iters < max_iters
        prev_sx = sx;
        prev_sy = sy;
        prev_sz = sz;

        % Calculate gradients
        [gradient_sx, gradient_sy, gradient_sz] = Gradient(c1, c2, ax, ay, az, sx, sy, sz, ux, uy, uz);

        total_gradient_sx = total_gradient_sx + gradient_sx;
        total_gradient_sy = total_gradient_sy + gradient_sy;
        total_gradient_sz = total_gradient_sz + gradient_sz;

        sx = max(-range/2, min(sy + rate * total_gradient_sx, range/2));
        sy = 0.0;
        sz = max(0.0, min(sz + rate * total_gradient_sz, 2.0));

        % Calculate step size
        previous_step_size = abs(sx - prev_sx) + abs(sy - prev_sy) + abs(sz - prev_sz);
        iters = iters + 1;
    end

    % Calculate the final value of f
    f = (c1 + (c2 / (sqrt((ax - sx)^2 + (ay - sy)^2 + (az - sz)^2) * sqrt((sx - ux)^2 + (sy - uy)^2 + (sz - uz)^2))))^2;
    if f < maximum
        maximum = f;
        ind=1;
        x = sx;
        y = sy;
        z = sz;
    end

    maximum = sqrt(maximum);
    % Wall [x, 15, z]
    iters = 0;
    previous_step_size = 1.0;
    total_gradient_sx = 0.0;
    total_gradient_sy = 0.0;
    total_gradient_sz = 0.0;
    
    % Gradient Ascent algorithm
    while previous_step_size > precision && iters < max_iters
        prev_sx = sx;
        prev_sy = sy;
        prev_sz = sz;
    
        % Calculate gradients
        [gradient_sx, gradient_sy, gradient_sz] = Gradient(c1, c2, ax, ay, az, sx, sy, sz, ux, uy, uz);
    
        total_gradient_sx = total_gradient_sx + gradient_sx;
        total_gradient_sy = total_gradient_sy + gradient_sy;
        total_gradient_sz = total_gradient_sz + gradient_sz;
    
        % Update variables
        sx = max(-range/2, min(sx + rate * total_gradient_sx, range/2));
        sy = range;
        sz = max(0.0, min(sz + rate * total_gradient_sz, 2.0));
    
        % Calculate step size
        previous_step_size = abs(sx - prev_sx) + abs(sy - prev_sy) + abs(sz - prev_sz);
        iters = iters + 1;
    end
    
    % Calculate the final value of f
    f = (c1 + (c2 / (sqrt((ax - sx)^2 + (ay - sy)^2 + (az - sz)^2) * sqrt((sx - ux)^2 + (sy - uy)^2 + (sz - uz)^2))))^2;
    if f < maximum
        ind=2;
        maximum = f;
        x = sx;
        y = sy;
        z = sz;
    end
end
