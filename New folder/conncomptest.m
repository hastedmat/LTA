 a = imread('map8.bmp');
 bw = a < 100;
imshow(bw)
title('Image with Circles')
hold on;
stats = regionprops('table',bw,'Extrema')%...
 % 'top-left', 'top-right', 'right-top', 'right-bottom', 'bottom-right',' bottom-left', 'left-bottom', 'left-top')  %'MajorAxisLength','MinorAxisLength')
t=size(stats.Extrema)
stats.Extrema{1}
vertex=[];
for i=1:t
   centers = stats.Extrema{i}
   for g=1:length(centers)
       vertex=[vertex;centers(g,2),centers(g,1)]
       plot(centers(g,2),centers(g,1));
       i
%   centers2 = stats.Extrema{2,1}
%   centers3 = stats.Extrema{3,1}
%   centers4 = stats.Extrema{4,1}
%   centers5 = stats.Extrema{5,1}
%   centers6 = stats.Extrema{6,1}
   end
end
 plot(vertex(:,2),vertex(:,1), '*r');
 hold off;
% diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
% radii = diameters/2;
% hold on
% viscircles(centers,radii);
% hold off