% Clear, close figures, and load images
clc; clear; close all;

% Set the directory containing images and create imageDatastore
imageDirectory = imageDatastore("data/BrickWall_data");
% imageDirectory = imageDatastore("data/Calib_data");
% imageDirectory = imageDatastore("data/LSC_data");
% imageDirectory = imageDatastore("data/Third_50_data");
montage(imageDirectory.Files)

% Initialize variables
numImages = numel(imageDirectory.Files);
transforms(numImages) = projective2d;
imageSizes = zeros(numImages, 2);

% Read the first image and initialize features
firstImg = readimage(imageDirectory, 1);
grayFirstImg = rgb2gray(firstImg);
filterSize = 1000;
% Change here to 800 and 1100
[harrisY, harrisX, ~] = harris(grayFirstImg, filterSize, 'disp');
[features, points] = extractFeatures(grayFirstImg, [harrisX, harrisY]);

% Iterate over the image pairs
for n = 2:numImages
    % Store points and features for the previous image
    prevPoints = points;
    prevFeatures = features;

    % Read the current image and convert to grayscale
    currentImg = readimage(imageDirectory, n);
    grayCurrentImg = rgb2gray(currentImg);

    % Save the current image size
    imageSizes(n, :) = size(grayCurrentImg);

    % Detect and extract Harris corners
    [harrisY, harrisX, ~] = harris(grayCurrentImg, filterSize, 'disp');
    [features, points] = extractFeatures(grayCurrentImg, [harrisX, harrisY]);

    % Find correspondences between current and previous images
    indexPairs = matchFeatures(features, prevFeatures, 'Unique', true);

    % Get matched points for the current and previous images
    matchedPoints = points(indexPairs(:, 1), :);
    matchedPointsPrev = prevPoints(indexPairs(:, 2), :);

    % Estimate the transformation between the images
    transforms(n) = estimateGeometricTransform(matchedPoints, matchedPointsPrev, ...
        'projective', 'Confidence', 99.9, 'MaxNumTrials', 2000);

    % Accumulate transformations
    transforms(n).T = transforms(n - 1).T * transforms(n).T;
end

% Compute output limits for each transformation
for i = 1:numel(transforms)
    [xlim(i, :), ylim(i, :)] = outputLimits(transforms(i), [1 imageSizes(i, 2)], [1 imageSizes(i, 1)]);
end

% Sort the transformations based on average X limits
avgXLimits = mean(xlim, 2);
[~, indices] = sort(avgXLimits);
centerIndex = floor((numel(transforms) + 1) / 2);
centerImgIndex = indices(centerIndex);

% Invert the transformation of the center image
invertedT = invert(transforms(centerImgIndex));
for i = 1:numel(transforms)
    transforms(i).T = invertedT.T * transforms(i).T;
end

% Update output limits based on new transformations
for i = 1:numel(transforms)
    [xlim(i, :), ylim(i, :)] = outputLimits(transforms(i), [1 imageSizes(i, 2)], [1 imageSizes(i, 1)]);
end

% Determine the minimum and maximum output limits
xMin = min([1; xlim(:)]);
xMax = max([max(imageSizes(:, 2)); xlim(:)]);
yMin = min([1; ylim(:)]);
yMax = max([max(imageSizes(:, 1)); ylim(:)]);

% Calculate the width and height of the panorama
panoWidth = round(xMax - xMin);
panoHeight = round(yMax - yMin);

% Initialize an empty panorama
panoramicView = zeros([panoHeight panoWidth 3], 'like', firstImg);

% Create a binary mask blender for image overlaying
blender = vision.AlphaBlender('Operation', 'Binary mask', 'MaskSource', 'Input port');

% Create a 2-D spatial reference object defining the size of the panorama
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([panoHeight panoWidth], xLimits, yLimits);

% Create the panorama by transforming and overlaying each image
for i = 1:numImages
    currentImg = readimage(imageDirectory, i);

    % Transform the current image into the panorama
    warpedImage = imwarp(currentImg, transforms(i), 'OutputView', panoramaView);

    % Generate a binary mask
    mask = imwarp(true(size(currentImg, 1), size(currentImg, 2)), transforms(i), 'OutputView', panoramaView);

    % Overlay the warped image onto the panorama
    panoramicView = step(blender, panoramicView, warpedImage, mask);
end

% Display the final panorama
figure
imshow(panoramicView)