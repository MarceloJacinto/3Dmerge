function rgbd = get_rgbd(xyz, rgb, R, T, K_rgb)

Kx = K_rgb(1,1);
Cx = K_rgb(1,3);
Ky = K_rgb(2,2);
Cy = K_rgb(2,3);

xyz_rgb = R * xyz';
xyz_rgb = [xyz_rgb(1,:) + T(1); xyz_rgb(2,:) + T(2); xyz_rgb(3,:) + T(3)];

x = xyz_rgb(1,:);
y = xyz_rgb(2,:);
z = xyz_rgb(3,:);

u = round(Kx * x./z + Cx);
v = round(Ky * y./z + Cy);

rgb_size = size(rgb);
n_pixels = numel(rgb(:,:,1));

v(v > rgb_size(1)) = 1;
v(v < 1) = 1;
u(u > rgb_size(2)) = 1;
u(u < 1) = 1;

rgb_inds = sub2ind(rgb_size, v, u);

rgbd = zeros(n_pixels,3,'uint8');
rgb_aux = reshape(rgb,480*640,3);

rgbd((1:n_pixels)',:) = rgb_aux(rgb_inds,:);

rgbd(xyz(:,1) == 0 & xyz(:,2) == 0 & xyz(:,3) == 0,:) = 0;

rgbd=uint8(reshape(rgbd,rgb_size));


end

