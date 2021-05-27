function stop = logOptSteps(z, optimValues, pack, scaler)
stop = false;

fn = ['it_log.mat'];

g = optimValues.gradient;

z = scaler.funscalez(z);

[t, x, u] = unPackDecVar_dc(z, pack);
[gt, gx, gu] = unPackDecVar_dc(g, pack);


if exist(fn, 'file')
    load(fn, 'Z','G','X','U','T','GX','GU','GT');
    
    Z(end+1,:) = z';
    G(end+1,:) = g';
    X(:,:,end+1) = x;
    U(:,:,end+1) = u;
    T(:,:,end+1) = t;
    GX(:,:,end+1) = gx;
    GU(:,:,end+1) = gu;
    GT(:,:,end+1) = gt;
   
    
else
    Z = zeros([1 size(z, 1)]);
    G = zeros([1 size(g, 1)]);
    X = zeros([size(x) 1]);
    U = zeros([size(u) 1]);
    T = zeros([size(t) 1]);
    GX = zeros([size(gx) 1]);
    GU = zeros([size(gu) 1]);
    GT = zeros([size(gt) 1]);
    
    Z(1, :) = z';
    G(1, :) = g';
    X(:,:,1) = x;
    U(:,:,1) = u;
    T(:,:,1) = t;
    GX(:,:,1) = gx;
    GU(:,:,1) = gu;
    GT(:,:,1) = gt;
    
end

save(fn, 'Z','G','X','U','T','GX','GU','GT');

end