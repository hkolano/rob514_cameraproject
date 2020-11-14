clc
clearvars

ptCloud1 = pcread('ptCloudRaw.ply');

maxDistance = 0.075;
roi = [-inf, 0.5, 0.2, 0.4, 0.1, inf];
sampleIndices = findPointsInROI(ptCloud1, roi);
[model, inlierIndices] = pcfitsphere(ptCloud1,maxDistance, 'SampleIndices', sampleIndices);
globe = select(ptCloud1, inlierIndices);

pcshow(globe)

mby3array = globe.Location;
[ center, radii, evecs, v, chi2 ] = ellipsoid_fit( mby3array, '0' )

hold on
ellipsoid(center(1), center(2), center(3), abs(radii(3)), abs(radii(2)), abs(radii(1)))