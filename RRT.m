clc
clear all
load("image_map.mat")

imshow(image_map)
m=image_map;

start = [20,20];
goal = [125,400];

internal=0; 
delta=15; 
range_goal=10; 
line_check=100; 
increase=10;


ADJ_M=[1]; 
LISTNODE=[start]; 
N=1;
found=0;
i=1;

k=0;
success=0;
check=0;
while(k<increase & success==0)
    internal=internal+25; 
    k=k+1; 
    while (i<internal & found==0)
        
        x_rand=round(rand*428);
        y_rand=round(rand*171);

        q_rand=[x_rand y_rand];
        
        
        q_near_index=near(LISTNODE,q_rand,N);


        
        Dx=x_rand-LISTNODE(q_near_index,1);
        Dy=y_rand-LISTNODE(q_near_index,2);
        
        leng=length(LISTNODE,q_rand,q_near_index,delta);
        
        Vx=Dx/norm([LISTNODE(q_near_index,:)-q_rand]);
        Vy=Dy/norm([LISTNODE(q_near_index,:)-q_rand]);
   
        q_new=[round(LISTNODE(q_near_index,1)+leng*Vx) round(LISTNODE(q_near_index,2)+leng*Vy)];

        check=checkfun(q_new,m,check,q_near_index,leng,line_check,Vx,Vy,LISTNODE);

        if check==1 

            ADJ_M=[ADJ_M zeros(N,1)];
            ADJ_M=[ADJ_M ; zeros(1,N+1)];
            
            ADJ_M(N+1,q_near_index)=1;
            ADJ_M(q_near_index,N+1)=1;
            ADJ_M(N+1,N+1)=1;
            N=N+1;
            
            LISTNODE=[LISTNODE;q_new];
            
            [ADJ_M,N,found,LISTNODE]=last(ADJ_M,N,LISTNODE,q_new,goal,range_goal,found);

        end  
               
        i=i+1;
    end
    if(found==1)
        fprintf('The goal has been achieved with %d iterations \n',i);
        success=1;
        for i=1:N-1
            if(ADJ_M(i,N)==1)
                    line([LISTNODE(i,1) goal(1,2)],[LISTNODE(i,2) goal(1,1)],'Color','magenta')
            end
        end
      
    else
        fprintf('with %d iterations is not possible to achieve the goal\n',internal);
    end 
end



 
hold on
plot(LISTNODE(2:N,1),LISTNODE(2:N,2),'o')
plot(start(1,1),start(1,2),'o','Color',[0 1 0])
plot(goal(1,2),goal(1,1),'o','Color',[1 0 0])
hold on



for i=1:N-2
    for j=i+1:N-1
        if ADJ_M(i,j)==1
            line([LISTNODE(i,1) LISTNODE(j,1)],[LISTNODE(i,2) LISTNODE(j,2)])
        end
    end
end

%calculation of the allowable distance
function leng = length(LISTNODE,q_rand,q_near_index,delta)
   if(norm([LISTNODE(q_near_index,:)-q_rand])<delta)
        leng=norm([LISTNODE(q_near_index,:)-q_rand]);
   else
        leng=delta;
   end
end

%connection to the goal
function [ADJ_M,N,found,LISTNODE]=last(ADJ_M,N,LISTNODE,q_new,goal,range_goal,found)

        if(norm([q_new-[goal(1,2) goal(1,1)]])<range_goal)
            found=1;

            ADJ_M=[ADJ_M zeros(N,1)];
            ADJ_M=[ADJ_M ; zeros(1,N+1)];
            ADJ_M(N+1,N)=1;
            ADJ_M(N,N+1)=1;
            ADJ_M(N+1,N+1)=1;
            N=N+1;
            LISTNODE=[LISTNODE;goal];                
        end
end

%check function
function check=checkfun(q_new,m,check,q_near_index,leng,line_check,Vx,Vy,LISTNODE)
        if(q_new(1,1)>0 & q_new(1,1)<428 & q_new(1,2)>0 & q_new(1,2)<171 & m(q_new(1,2),q_new(1,1))==1)
            check=1;
            for k=1:line_check
                
                coord_pt=[round(LISTNODE(q_near_index,1)+leng/line_check*k*Vx) round(LISTNODE(q_near_index,2)+leng/line_check*k*Vy)];  
                if(m(coord_pt(1,2),coord_pt(1,1))==0)
                    check=0;
                end
            end
        else
            check=0;
        end
end
%connection with the nearest node
function q_near_index=near(LISTNODE,q_rand,N)
        upper=10000; 
        
        for j=1:N 
            if(norm([LISTNODE(j,:)-q_rand])<upper)
                upper=norm([LISTNODE(j,:)-q_rand]);
                q_near_index=j;
            end
        end
end


