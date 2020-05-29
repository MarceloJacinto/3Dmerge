function xyz_ret = get_xyzasus(im_vec, im_orig_size, good_inds, K, alpha, beta)
% im_vec - depth image vectorized (Nx1)
% im_orig_size - original image size (HxW) : [H, W]
% goot_inds - indexes of the image that are valid, i.e., different from 0.

persistent u;
persistent v;
persistent im_size;
persistent xyz;
persistent z;

Kx = K(1,1);
Cx = K(1,3);
Ky = K(2,2);
Cy = K(2,3);

if isempty(im_size)
    %     im_size = size(im);
    im_size = im_orig_size;
    
    u = repmat(1:im_size(2),im_size(1),1);
    u = u(:)-Cx;
    v = repmat((1:im_size(1))',im_size(2),1);
    v=v(:)-Cy;
    xyz=zeros(length(u),3);
end

% tmp = im(:);
xyz(:,3) = double(im_vec)*0.001; % Convert to meters
xyz(good_inds,3) = alpha*xyz(good_inds,3) + beta;
xyz(:,1) = (xyz(:,3)/Kx) .* u ;
xyz(:,2) = (xyz(:,3)/Ky) .* v;

%plot3(x,y,z,'.');axis equal
xyz_ret = xyz;

end

