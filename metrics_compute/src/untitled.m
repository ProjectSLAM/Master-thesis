filename = ['hector_real_1.','png'];
SLam = imread(filename);
Ground_map = imread('Ground_truth_1.png');
[counts,x] = imhist(SLam,16);
T = otsuthresh(counts);
BW_slam = imbinarize(SLam,T);
[counts,x] = imhist(Ground_map,16);
T = otsuthresh(counts);
BW_ground = imbinarize(Ground_map,T);
BW_slam = double(BW_slam(:,:,1));
BW_ground = double(BW_ground(:,:,1));

%imshow(imfuse(BW_slam,BW_ground))
[optimizer, metric] = imregconfig('monomodal');
movingRegistered = imregister(BW_slam, BW_ground, 'similarity', optimizer, metric);
C = imfuse(BW_ground,movingRegistered,'blend','Scaling','joint');
imshow(C)
imwrite(movingRegistered, 'map_move.jpg');