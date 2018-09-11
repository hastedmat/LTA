

clear all;
close all;
clc;
NumberTimeStamps = 100;
incdist=0;
dist = 0;
kount=0;
tb=[];
 img2 = imread('Conc.bmp');% read Map
img=img2;
%img = imtranslate(img,[-85, -85])
 Iblur1 = imgaussfilt(img,2);

 map = im2bw(Iblur1);
 img1=img;
 figure;
 imshow(img);
        goal=[490 490];
 source=[10 10]
 pin=1; button = 0;
 landmarks=0;
 vertex=[];
 count=0;


 display=true;
hold on;
% Slelect start Point 
while button ~= 3
   [x,y,button]=ginput(1);
   pin= ~isempty(x);
   if pin && button ~= 3
     
      plot(x,y,'r*')
     % plot(goal(2),goal(1),'r*')
      
      vertex=[vertex;y,x];
     tb=vertex;
   end
end
tic;
tb=vertex(1,:);

goal(1,1)=int16(y);
goal(1,2)=int16(x);
 figure();
 imshow(map);
 title('LTA* ')
%rectangle('position',[1 1 size(map)-1],'edgecolor','k')
 hold on;
%waitforbuttonpress; 
while norm(tb - goal)>3
     vertex=[];
     vertex=[vertex;tb];
    if count==10
        [x,y,button]=ginput(1);% Select goal. It repeatedly takes goal after 10 iterations to check if goal location has been changed
        count=0;
       goal(1,1)=int16(y);
       goal(1,2)=int16(x);
       %plot(goal,'r*')
    end

    vertex=[vertex;goal];    
   





%% Corner detection part

p = size(img);
 for i=1:1:p(1,1)
    for j=1:1:p(1,2)
        if img(i,j)==0
            img(i,j) =1;
        else
            img(i,j)= 0;
        end
    end
end

 p=0;
 for i=1:size(img1,1)
     for j=1:size(img1,2)
       
 u=0;
v=0;           
            if  img1(i,j)>0 
                
                if j+1<size(img,2)
                if img1(i,j+1)==0 
                    kount=kount+1;
                    u=i;
                    v=j-p;
                end
                end
                if  j-1>0
                if img1(i,j-1)==0;
                    kount=kount+1;
                     u=i;
                    v=j+p;
                end
                end
                if  i+1<size(img,1)&& j-1>0
                if img1(i+1,j-1)==0;
                    kount=kount+1;
                     u=i-p;
                    v=j+p;
                end   
                end
                if  i+1<size(img,1)
                if img1(i+1,j)==0;
                    kount=kount+1;
                     u=i-p;
                    v=j;
                end
                end
                if  i+1<size(img,1)&& j<size(img,2)
                if img1(i+1,j+1)==0;
                    kount=kount+1;
                     u=i-p;
                    v=j-p;
                end
                end
                if  i-1>0&& j-1>0
                    if img1(i-1,j-1)==0;
                    kount=kount+1;
                     u=i+p;
                    v=j+p;
                    end
                end
               if  j+1<size(img,2)&& i-1>0
                    if img1(i-1,j+1)==0;
                    kount=kount+1;
                     u=i+p;
                    v=j-p;
                    end
               end
               if i-1>0
                      if img1(i-1,j)==0;
                    kount=kount+1;
                     u=i+p;
                    v=j;
                      end
               end
            if  kount==1;
            vertex=[vertex;[u,v]];
            checker=[u,v];
            if feasiblePoint(checker,map)
             plot(v,u,'b*');
            end
            end
        
            end 
        kount=0;
     end
 end
%xlabel(['Time Taken= ',num2str(toc),'s Number of Corners= ',num2str(length(vertex))],'FontSize',10,'FontWeight','bold','Color','b')
  %  disp('click/press any key');
   % waitforbuttonpress; 
 t1 = size(vertex);
 k = t1(1,1)-2;

  if display, rectangle('Position',[vertex(1,2)-5,vertex(1,1)-5,10,10],'Curvature',[1,1],'FaceColor','b'); end
   if display, rectangle('Position',[goal(2)-5,goal(1)-5,10,10],'Curvature',[1,1],'FaceColor','b'); end

% if display 
%     disp('click/press any key');
%     waitforbuttonpress; 
% end
%%    Start of A* algorithm  
s=0;
source=round(tb);
Q=[source 0  precost(vertex(1,:),goal) 0+precost(vertex(1,:),goal)   -1]; % the processing queue of A* algorihtm, open list
 closed=ones(size(map)); % the closed list taken as a list
 pathFound=false;
 closedList=[];
 while size(Q,1)>0
      [A, I]=min(Q,[],1);
      n=Q(I(5),:); % smallest cost element to process
      Q=[Q(1:I(5)-1,:);Q(I(5)+1:end,:)]; % delete element under processing
     % if display, rectangle('Position',[n(2)-5,n(1)-5,10,10],'Curvature',[1,1],'FaceColor','g'); end
      if n(1)==goal(1) && n(2)==goal(2) % goal test
         pathFound=true;break;
      end
%       if display 
%      disp('click/press any key');
%      waitforbuttonpress; 
%  end

      for mv=2:length(vertex) %iterate through all corners to check for local tangents
         
          newPos=vertex(mv,:); % Vertex from where tangents are taken on other vertices.
            
         if checkallPath(n(1:2),newPos,map) %if path from n to newPos is collission-free
              %if display, rectangle('Position',[newPos(2)-5,newPos(1)-5,10,10],'Curvature',[1,1],'FaceColor','b'); end
               %line([n(2);newPos(2)],[n(1);newPos(1)],'color','r'); % Display Tangents
               s=s+1;
              if closed(newPos(1),newPos(2))~=0 % not already in closed
                   
                  historicCost=n(3)+postcost(n(1:2),newPos);
                  heuristicCost=precost(newPos,goal);
                  totalCost=historicCost+heuristicCost;
                  add=true; % not already in queue with better cost
                  if length(find((Q(:,1)==newPos(1)) .* (Q(:,2)==newPos(2))))>=1
                      I=find((Q(:,1)==newPos(1)) .* (Q(:,2)==newPos(2)));
                      if Q(I,5)<totalCost, add=false;
                      else Q=[Q(1:I-1,:);Q(I+1:end,:);];
                          
                          
                          add=true;
                      end
                  end
                  if add
                      
                      Q=[Q;newPos historicCost heuristicCost totalCost size(closedList,1)+1]; % add new nodes in queue
                 
                  end
              end
         end           
      end
      
            closed(n(1),n(2))=0;
            closedList=[closedList;n]; % update closed lists
        

 end
%% Final Display Part
e=length(vertex)
if ~pathFound
    error('no path found')
end
 % hold off;
%fprintf('processing time=%d \n', toc); 
path=[n(1:2)]; %retrieve path from parent information
prev=n(6);
while prev>0
    path=[closedList(prev,1:2);path];
    prev=closedList(prev,6);
end
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+postcost(path(i,:),path(i+1,:)); end
fprintf('processing time=%d', toc,e);


% rectangle('position',[1 1 size(map)-1],'edgecolor','k')
% waitforbuttonpress;
 hold on;
line(path(:,2),path(:,1),'color','b','LineWidth',2);

xlabel(['Time Taken= ',num2str(toc),'s Path Length= ',num2str(pathLength)],'FontSize',10,'FontWeight','bold','Color','b')
%  
% disp('click/press any key');
%     waitforbuttonpress; 
npoints =0;    
  %  plot(path(i,1),path(i,2),'r*');
  for i=1:length(path)
     npoints = npoints + 1;
    t(:,npoints)=[path(i,1);path(i,2)];
 
   if npoints > 1
       plot(t(2,npoints-1:npoints),t(1,npoints-1:npoints),'g+');
      
       dist = dist + norm(t(:,npoints) - t(:,npoints-1));
   end
  end
  point = 2; dist2=0; incdist=5;
tb=t(:,1);
d1=0;

    tb=tb+ incdist*((t(:,point)-t(:,point-1))/norm(t(:,point)-t(:,point-1))); % tx,ty trajectories

    plot(tb(2),tb(1),'b.');
    tb=tb';
    pause(.0001);
    count=count+1;
    dist2 = dist2 + incdist;
    if (dist2 + incdist) > norm(t(:,point)-t(:,point-1)) && abs((dist2 + incdist)-norm(t(:,point)-t(:,point-1))) > abs(dist2-norm(t(:,point)-t(:,point-1)))
        point = point + 1; dist2 = 0;
    end

end

hold off;
