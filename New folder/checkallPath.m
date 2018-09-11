%© Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Probabilistic Roadmap, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

function feasible=checkPath1(n,newPos,map)
feasible=true;
dir=atan2(newPos(1)-n(1),newPos(2)-n(2));
for r=1:10
posCheck=newPos+r.*[sin(dir) cos(dir)];

%if display, rectangle('Position',[posCheck(2)-5,posCheck(1)-5,10,10],'Curvature',[1,1],'FaceColor','r'); end
    if ~(feasiblePoint(round(posCheck),map))
            %feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
      
            feasible=false;
            break;
    % waitforbuttonpress; 
    end
end
for r=0:1:sqrt(sum((n-newPos).^2))
    posCheck=newPos-r.*[sin(dir) cos(dir)];
    %if display, rectangle('Position',[posCheck(2)-5,posCheck(1)-5,10,10],'Curvature',[1,1],'FaceColor','g'); end
    if ~(feasiblePoint(round(posCheck),map))
            %feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;
        break;
    end
    if ~feasiblePoint(newPos,map), feasible=false; end
     %waitforbuttonpress; 
end

    
