% Tutorial questions
clear all
close all

%%

% 7.6.1 & 7.6.2 & 7.6.3

% set camera parameters
W = 0.05; H = 0.03; % retina size
Nx = 1200; Ny = 1000; % retina pixels
f = 0.075; % focal length

% plot the retina
figure
plot([0,0,Nx,Nx,0],[0,Ny,Ny,0,0],'r','Linewidth',4);
set(gca, 'YDir','reverse') % flip y axis to follow retina coordinates
hold on
grid on

% make the vertices of the cube relative to frame {0}
cube(1,:) = [-1,1,-1]; cube(2,:) = [-1,-1,-1]; cube(3,:) = [1,-1,-1]; cube(4,:) = [1,1,-1];
cube(5,:) = [-1,1,1]; cube(6,:) = [-1,-1,1]; cube(7,:) = [1,-1,1]; cube(8,:) = [1,1,1];


% Form the camera matrix
dw = W/Nx;
dh = H/Ny;
% centre of retina
u0 = round(Nx/2);
v0 = round(Ny/2);
CM = [f/dw 0 u0 0 ; 0 f/dh v0 0 ; 0 0 1 0];

% set the position of the camera
% 7.6.1
T0C = [1 0 0 0; 0 1 0 0; 0 0 1 -10 ; 0 0 0 1];
% 7.6.2
T0C = [1 0 0 -1; 0 1 0 -1; 0 0 1 -10 ; 0 0 0 1];
% 7.6.3
d = 10; % set the distance of the camera
ur = [1, 0, 0]; % set the unit vector of revolution (x-axis)
%  these lines below apply only for 7.6.3     ****************************
% for ad = 0 : 10 : 360 % rotate the camera, in steps of 10 degrees
% a = ad*pi/180;  % convert in radian
% % Calculate new position of camera relative to {0}:
% % apply Eq. 33 (Chapter 1), to determine the rotation matrix. Add
% % translation too.
% c = (1-cos(a));
% T0C = [ur(1)^2*c+cos(a), ur(1)*ur(2)*c-ur(3)*sin(a), ur(1)*ur(3)*c+ur(2)*sin(a), 0; ...
%        ur(1)*ur(2)*c+ur(3)*sin(a), ur(2)^2*c+cos(a), ur(2)*ur(3)*c-ur(1)*sin(a), d*sin(a); ...
%        ur(1)*ur(3)*c-ur(2)*sin(a), ur(2)*ur(3)*c+ur(1)*sin(a), ur(3)^2*c+cos(a), -d*cos(a); ...
%        0, 0, 0, 1];
%  these lines above apply only for 7.6.3     ****************************


TC0 = inv(T0C); % inverse of T0C
% project each point of the cube into the retina
for i = 1 : 8
    % get one vertex
    xp = cube(i,1); yp = cube(i,2); zp = cube(i,3);
    % Point P desxribed in homogenous form
    Pt0 = [xp yp zp 1]';
    % projected point on the 2D retina, in homogenous form
    pt = CM * TC0 * Pt0;
    % determine actual coordinates of projected point, from homogenous form
    u(i) = floor(pt(1)/pt(3));
    v(i) = floor(pt(2)/pt(3));
    % check if the projection falls outside the retina
    if u(i)<0 | u(i)>Nx | v(i)<0 | v(i)>Ny
        disp(strcat('Warning: vertex-',num2str(i),' is out of the retina'));
	end
end

% plot the eight projections
scatter(u,v,'bo','Linewidth',3)
title(strcat('Camera lens ',num2str(f)))
% print names of vertices
for i = 1 : 8
    text(u(i),v(i),num2str(i),'FontSize',20)
end

%  these lines below apply only for 7.6.3     ****************************
% pause(1)
% end
%  these lines above apply only for 7.6.3     ****************************

%% 7.6.4
img = imread('Dover.jpg'); % read in the image
img = rgb2gray(img);
figure
imshow(img)
title('Original picture')
S = size(img)
figure
histogram(img,[0:1:255]) % count and show the number of pixels for each integer level of grey [0:1:255]
title('Pixel counting of grayscale')
grid on
% a)
cliff = img(:,:)>180; % select the pixels with level of grey > 180. Pixels will have values: 0 if img<180, or 1 if img>180
figure
imshow(cliff)
title('Image with levels > 180')
Ncliff = sum(sum(cliff)) % count the number of ones by summing them all
Percentage = Ncliff / (S(1)*S(2))
% b
% we need to remove all pixels with values <180, whilst preserving the remaining
CliffOnly = img.*uint8(cliff); % we retain the value of grey for selected pixels of cliff (with ones) and set to zero all the others
figure
imshow(CliffOnly)
title('Cliff only retained, with original grey levels')
figure
histogram(CliffOnly)
title('Pixel counting cliff only, with original grey levels')
grid on
% c
% band the cliff, ie pixels in the range 180-255, into 4 levels
Res = (255-180)/3;
Banded = ( floor((CliffOnly-180)/Res)*Res +180).*uint8(cliff);
figure
imshow(Banded)
title('Cliff only with only four levels of gray')
figure
histogram(Banded)
title('Pixel counting cliff only, with only four levels of gray')
grid on
% d
Negative = 255-img;
figure
imshow(Negative)
title('Negative image of original greylevel')
figure
histogram(Negative)
title('Pixel counting of negative image of original greylevel')
grid on

%% 7.6.5
img = imread('Tiger.jpg'); % read in the image
img = rgb2gray(img);
figure
imshow(img)
title('Original image')
[nr,nc] = size(img);
figure
histogram(img)
title('Pixel counting of original image')
grid on
[n,h]=hist(img,[0:1:255]);
n = cumsum(n); % cumulative sum of bars in the histogram
n = n/max(n); % normalise to 1
ni = interp1(h, n, double(img(:)),'nearest'); % interpolate and determine new pixel levels, as dictated by interpolating values h and n
ni = cast(ni*double(intmax(class(img))), class(img)); % this is just a type variable re-adjustement: it does nothing mathematically
ni = reshape(ni,nr,nc); % re-organise data as for teh retina matrix size
figure
imshow(ni)
title('Image with redistributed levels of grey')
figure
histogram(ni)
title('Pixel counting of image with redistributed levels of grey')
grid on

%% 7.6.6
% open the video
vid = VideoReader('CarsMoving.mp4');
vid
% a
background = readFrame(vid); % grab the first frame
background = rgb2gray(background);
sigma = 1; % set the threshold
for i = 1 : 200 % there are 574 frames, but it takes a while to run all of them.
    % for every frame
    frame = readFrame(vid); % grab a new frame
    frame = rgb2gray(frame);
    diff = frame - background; % I - B
    F = max(min(diff,sigma),-sigma); % evaluate F
    background = background + F; % apply the recursive formula
end
figure % show the extracted background
imshow(background);
title('Background')
% b
% prepare for a new video making
writerObj = VideoWriter('CarsMotion.mp4','MPEG-4');
writerObj.FrameRate = 10;
open(writerObj);
% open the original video
vid = VideoReader('CarsMoving.mp4');
for i = 1 : 200
    % for every frame
    frame = readFrame(vid); % grab a new frame
    frame = rgb2gray(frame);
    motion = frame - background; % subtract background from original frame
    writeVideo(writerObj, motion); % cast the motion frame into a movie
end
close(writerObj);
% c
% read in the new desert background
newbackground = imread('Mondrian.jpg');
newbackground = rgb2gray(newbackground);
figure
imshow(newbackground)
% prepare for a new video making
writerObj = VideoWriter('CarsMotiondesert.mp4','MPEG-4');
writerObj.FrameRate = 10;
open(writerObj);
vid = VideoReader('CarsMotion.mp4');
for i = 1 : 200
    % for every motion frame
    frame = readFrame(vid); % grab a new frame
    frame = rgb2gray(frame);
    % remove the pixelx in the new background where the car will be located
    mask = frame > 10;
    newbkemptied = newbackground.*uint8(1-mask);
    % relocate the cars into the new background
    finalrelocation = newbkemptied+frame.*uint8(mask);
    % cast into the video
    writeVideo(writerObj, finalrelocation);
end
close(writerObj);
%% 7.6.7
%smoothing
img = imread('LadyFace.jpg');
% Since we need to convolute with a non integer kernel
img = im2double(rgb2gray(img));
imshow(img)
% a Uniform kernel
m = 21; % size of the filtering window
K = ones(m,m)/m^2; % window of ones
sm = conv2(img, K); % convolute image with kernel
figure
imshow(sm)
%b
sigma = 7; % standard deviation of the Gaussian  bell shape
w = 7; % size of the filtering window
[x,y] = meshgrid(-w:w, -w:w);  % set the grid for the filtering window
K = 1/(2*pi*sigma^2) * exp( -(x.^2 + y.^2)/2/sigma^2); % Gaussian kernel
sm = conv2(img, K); % convolute image with kernel
figure
imshow(sm)

%% 7.6.8
img = imread('StreetSign.jpg');
img = im2double(rgb2gray(img));
imshow(img)
K = [0.5 0 -0.5]; % set the kernel to represent a central difference differentiation
edh = conv2(img, K,'same');  % differentiate horizontally 
edv = conv2(img, K','same');  % differentiate vertically
edges = 1-sqrt(edh.^2+edv.^2); % combine horizontal + vertical edge detection
figure
imshow(edges)
%% 7.6.9
% resize
img = imread('QueenSmall.jpg');
img = rgb2gray(img);
m = 2; % scaling factor
figure
imshow(img)
[nr, nc] = size(img); % size of original image
% replicate the rows
ir = uint8(zeros(m*nr,nc)); % set an enlarged matrix, with more rows
for i=1:m
    ir(i:m:end,:) = img; % replicate the rows
end
% continuing on the job:
% replicate the columns
ir2 = uint8(zeros(m*nr,m*nc)); % set an enlarged matrix, with more columns
for i=1:m
    ir2(:,i:m:end) = ir; % replicate the rows
end
big = ir2;
figure
imshow(big)
%% 7.6.10
% Morphology: erosion and dilation
img = imread('Circuit.jpg');
img = rgb2gray(img);
size(img)
figure
imshow(img)
% set the sought shape: a circle of radius 5
R = 5;
K = zeros(2*R+1,2*R+1); % set the window matrix as all zeros
[x,y] = meshgrid(1:2*R+1,1:2*R+1); 
l = find(round((x-R).^2 + (y-R).^2 - R^2) <= 0); % set the equationf of the circle
K(l) = 1; % set value 1 in the window grid wherever there is a circle
Kp(:,:) = uint8(K*255); % set the level to maximum (white)

% opening: erosion + dilation
% erosion
morf = imerode(img,K); % erode the image with the set kernel
figure
imshow(morf);
hold on
imshow(Kp) % plot the kernel, as term of reference
% dilation
morf = imdilate(morf,K); % dilate the image with the set kernel
figure
imshow(morf);
hold on
imshow(Kp) % plot the kernel, as term of reference

%% 7.6.11
% a
img = imread('Shapes.jpg');
img = rgb2gray(img);
%img = morf; %for Ex 7.6.12
imshow(img)
[nr, nc] = size(img);
figure
histogram(img)
grid on
sigma = 150;  % threshold:  Ex 7.6.11 set to 150; Ex 7.6.12 set to 0.6
C = zeros(nr,nc);
C = img>sigma; % select only pixellevels > 150 (these would correspond to the shapes)
figure
imshow(C)
% b
% search for perimeters points
Per = zeros(nr,nc);
for r = 1 : nr
    for c = 1 : nc
        % check if this pixel is part of a shape and not an interior point
        if C(r,c) == 1 &  ~( C(r-1,c)==1 & C(r+1,c)==1 & C(r,c-1)==1 & C(r,c+1)==1 )
            % it is not an interior pixel, hence flag it as perimetral pixel
            Per(r,c) = 1;   
        end
    end
end
figure
imshow(Per)
% c
Ns = 0; % number of shapes/perimeters
rnblist = [-1, -1, -1, 0, 0, 1, 1, 1]; % list of neighbour rows
cnblist = [-1, 0, 1, -1, 1, -1, 0, 1]; % list of neighbour columns
Taken = zeros(nr,nc); % set all the pixels as not being considered yet
% scroll all pixels
for r = 1 : nr
    for c = 1 : nc
        % check if this pixel is perimetral and has not been considered yet
        if Per(r,c)==1 & Taken(r,c)==0 
            % start a new shape/perimeter
            Ns = Ns + 1; % increment the number of known shapes/perimeters
            npix = 1; % number of pixels in this shape
            % include this pixel in the current perimeter
            Labels(Ns,npix,1) = r;
            Labels(Ns,npix,2) = c;
            % flag this pixel as been considered, so it won't be
            % examinedanymore in future.
            Taken(r,c) = 1;
            % now start walking through the perimeter
            rn = r+1; cn = c+1; % set random new step
            ro = r; co = c; % set current positionas old position
            % walk until we get back to the initital point (r,c)
            while rn~=r | cn~=c
                % find the next perimetal point
                % look at the 8 neighbouring pixels
                for i = 1 : 8
                    rn = ro + rnblist(i);
                    cn = co + cnblist(i);  
                    % check if this neighbour is perimetral and if not
                    % considered
                    if Per(rn,cn)==1 & Taken(rn,cn)==0
                        % this pixel is part of teh perimeter and not
                        % considered yet, hence we include it in the
                        % current shape
                        npix = npix + 1; % increment the number of pixels in this shape
                        % include this pixel in the current perimeter
                        Labels(Ns,npix,1) = rn;
                        Labels(Ns,npix,2) = cn; 
                        Taken(rn,cn) = 1; % flag this pixel as been considered
                        ro = rn; co = cn; % set current positionas old position
                        if npix==3, Taken(r,c)=0;, end % release the seed, so we can walk back into it at the end of the walk (closure)
                        break
                    end
                end
            end
            npix = npix - 1; % since we walked back into the seed, which was included already in the shape, take it away
            Npixels(Ns) = npix; % register teh number of pixels for the shape just formed
        end
    end
end
disp(Ns)
% d
figure
imshow(Per)
hold on
% for each shape
for i = 1 : Ns
    % determine the min and max coordinates for both u and v
    umin = min(Labels(i,1:Npixels(i),2)); umax = max(Labels(i,1:Npixels(i),2));
    vmin = min(Labels(i,1:Npixels(i),1)); vmax = max(Labels(i,1:Npixels(i),1));
    % plot the encompassing box
    plot([umin,umin,umax,umax,umin],[vmin,vmax,vmax,vmin,vmin],'r','Linewidth',2);
end
% e
for i = 1 : Ns
    m00 = nnz(Labels(i,1:Npixels(i),1)); % compute momentum 00, by summing all non zero elements (all coordinates)
    m10 = sum(Labels(i,1:Npixels(i),2)); % compute momentum 10, by summing u coordinates only
    m01 = sum(Labels(i,1:Npixels(i),1)); % compute momentum 01, by summing v coordinates only
    uc = m10/m00; vc = m01/m00; % compute the centroid
    scatter(uc,vc,'g','Linewidth',4);
end

%% 7.6.13
img = imread('Mondrian.jpg');
img = im2double(rgb2gray(img));
figure
imshow(img)

% detect edges with horizontal/vertical differentiation 
K = [0.5 0 -0.5];
edh = conv2(img, K,'same');
edv = conv2(img, K','same');
edges = sqrt(edh.^2+edv.^2);
figure
imshow(edges)
% apply Hough transform
[H,T,R] = hough(edges);
figure
imshow(imadjust(rescale(H)))