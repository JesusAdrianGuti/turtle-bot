%% Shukui Zhang
% This code is used to obtain the threshold for h.s.v after having the
% sample images by running camera_example.m

img = imread('image1.bmp');
figure(1)
imshow(img)

hsv = rgb2hsv(img); % convert to HSV
figure(4)
surf(hsv(:,:,1))
shading interp
title('Hue')

figure(5)
surf(hsv(:,:,2))
shading interp
title('Saturation')

figure(6)
surf(hsv(:,:,3))
shading interp
title('Value')


orange_binary_hue = (hsv(:,:,1) > 0.02).* (hsv(:,:,1) < 0.045);

%% Optional: using bwareafilt to leave only the largest object in binary image
% (object), while remove all the smaller ones (outliers)
% orange_binary_hue  = bwareafilt(imbinarize(orange_binary_hue),1);

% apply thresholding for hsv
green_binary_hue = (hsv(:,:,1) > 0.29).* (hsv(:,:,1) < 0.39);
green_binary_satu = (hsv(:,:,2) > 0.37).* (hsv(:,:,2) < 0.80);
green_binary_value = (hsv(:,:,3) > 0.3).* (hsv(:,:,3) < 0.60);

green_binary_hsv = green_binary_hue.* green_binary_satu.* green_binary_value;

figure(10)
imshow(orange_binary_hue)
title('Binary orange cylinder - Hue')

figure(20)
imshow(green_binary_hue)
title('Binary green cylinder - Hue')

figure(30)
imshow(green_binary_hsv)
title('Binary green cylinder - H.S.V.')

%% as seen in binary image from hue, orange cylinder has more outliers than the green one.
% Properly using information from additional channel, e.g. Saturation and value does improve
% performance (less outliers exist by adding saturation and value information), but
% may also accidentally remove pixels belong to the object




