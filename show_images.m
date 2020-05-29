function show_images(image_list, n, j, features_vector, matches, ok)
%SHOW_IMAGES Summary of this function goes here
%   Detailed explanation goes here

% ----TO BE REMOVED LATER - VISUALIZE THE FEATURES IN THE IMAGE----
    im1 = imread(char(image_list(n)));
    im2 = imread(char(image_list(j)));
    %im1 = imresize(im1, [640 480]);
    %im2 = imresize(im2, [640 480]);
    im1 = im2single(im1) ;
    im2 = im2single(im2) ;
    
    numMatches = size(matches,2);

    dh1 = max(size(im2,1)-size(im1,1),0) ;
    dh2 = max(size(im1,1)-size(im2,1),0) ;

    figure(); clf ;
    subplot(2,1,1) ;
    imagesc([padarray(im1,dh1,'post') padarray(im2,dh2,'post')]) ;
    o = size(im1,2) ;
    line([features_vector{n}(1,matches(1,:));features_vector{j}(1,matches(2,:))+o],[features_vector{n}(2,matches(1,:));features_vector{j}(2,matches(2,:))]) ;
    title(sprintf('%d tentative matches', numMatches)) ;
    axis image off ;

    subplot(2,1,2) ;
    imagesc([padarray(im1,dh1,'post') padarray(im2,dh2,'post')]) ;
    o = size(im1,2) ;
    line([features_vector{n}(1,matches(1,ok));features_vector{j}(1,matches(2,ok))+o],[features_vector{n}(2,matches(1,ok));features_vector{j}(2,matches(2,ok))]) ;
    title(sprintf('Images %d - %d \n %d (%.2f%%) inliner matches out of %d', n, j,sum(ok), 100*sum(ok)/numMatches,numMatches)) ;
    axis image off ;

    drawnow ;
    % -----------------------------------------------------------------

end

