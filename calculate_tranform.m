function [ R, T ] = calculate_tranform( A, B )
%CALCULATE_TRANFORM function to calculate rigid transformation that aligns
%two sets of corrsponding points
% Transforms A into B if multiplied by rotation and applied the translation

assert(length(A) == length(B));

% number of points
[~,N]= size(A);

% Calculate the centroids
centroid_A = mean(A, 2);
centroid_B = mean(B, 2);

% Centered points
centered_A = A - repmat(centroid_A, 1, N);
centered_B = B - repmat(centroid_B, 1, N);

H = centered_A * centered_B';

% Matrix decomposition on rotation, scaling and rotation matrices
[U, ~, V] = svd(H);

% Rotation Matrix
R = V * U';

% Problem with the sign in rotation matrix before R calculation
if det(R) < 0
    % Multiply 3rd column o V by -1
    V(:, 3) = -1 * V(:,3);
    R = V * U';
end    

%Translation Vector
T = centroid_B - R*centroid_A;

end

