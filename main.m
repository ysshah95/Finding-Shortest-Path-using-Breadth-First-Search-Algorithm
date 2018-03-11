clc
clear all
close all

% Defining the whole plot from the given coordinates

x1 = [0 250 250 0];
y1 = [0 0 150 150];
rec = polyshape(x1,y1);


x2 = [55 55 105 105];
y2 = [112.5 67.5 67.5 112.5];
square = polyshape(x2,y2);


x3 = [120 158 165 188 168 145];
y3 = [55 51 89 51 14 14];
poly = polyshape(x3, y3);


% Circle
xc=180;
yc = 120;
t = 0:0.01:2*pi;
radius = 15;
x4 = radius*cos(t)+ xc;
y4 = radius*sin(t) + yc;


status = false;
disp('')
while ~status

    prompt_x_start = 'Enter X Coordinate of Starting Point between 0 to 250:  ';
    x_s = input(prompt_x_start);
    
    prompt_y_start = 'Enter Y Coordinate of Starting Point between 0 to 150:  ';
    y_s = input(prompt_y_start);
    
    prompt_x_goal = 'Enter X Coordinate of Goal Point between 0 to 250:  ';
    x_g = input(prompt_x_goal);
    
    prompt_y_goal = 'Enter Y Coordinate of Goal Point between 0 to 150:  ';
    y_g = input(prompt_y_goal);
    
    StartNode = [x_s,y_s];
    GoalNode = [x_g,y_g];

    in_start = insidepoly_halfplane(x_s,y_s);
    in_goal = insidepoly_halfplane(x_g,y_g);

    if in_start
        status = false;
        disp('The start point provided is inside the obstacle.');
    elseif in_goal
        status = false;
        disp('The goal point provided is inside the obstacle.');
    elseif (x_s<0 || x_s>250) || (y_s<0 || y_s>150)
        status = false;
        disp('The start point provided is not in the given workspace.')
    elseif  (x_g<0 || x_g>250) || (y_g<0 || y_g>=250)
        status = false;
        disp('The goal point point provided is not in the given workspace.')
    else
        status = true;
    end
end

tic

if status
   
    Nodes = [];
    NodesInfo = [];

    Nodes(:,:,1) = StartNode;
    NodesInfo(:,:,1) = [1,0];

    i = 2;
    j = 1;

    while true
        CurrentNode = Nodes(:,:,j);
        [StatusL, NewNodeL] = ActionMoveLeft(CurrentNode);
        if StatusL == true
            if (any(all(bsxfun(@eq,Nodes,NewNodeL)))) == false
                c = NewNodeL(1);
                d = NewNodeL(2);
                in = insidepoly_halfplane(c,d);
                if in == false 
                    Nodes(:,:,i) = NewNodeL;
                    NodesInfo(:,:,i) = [i,j];
                    i = i+1;
%                     drawnow 
                    plot(c,d,'.','color','yellow')
                    if NewNodeL(1) == GoalNode(1) && NewNodeL(2) == GoalNode(2)
                        break
                    end
                end
            end
        end

        [StatusR, NewNodeR] = ActionMoveRight(CurrentNode);
        if StatusR == true
            if (any(all(bsxfun(@eq,Nodes,NewNodeR)))) == false
                c = NewNodeR(1);
                d = NewNodeR(2);
                in = insidepoly_halfplane(c,d);
                if in == false
                    Nodes(:,:,i) = NewNodeR;
                    NodesInfo(:,:,i) = [i,j];
                    i = i+1;
%                     drawnow
                    plot(c,d,'.','color','yellow')
                    if NewNodeR(1) == GoalNode(1) && NewNodeR(2) == GoalNode(2)
                        break
                    end
                end
            end
        end

        [StatusU, NewNodeU] = ActionMoveUp(CurrentNode);
        if StatusU == true
            if (any(all(bsxfun(@eq,Nodes,NewNodeU)))) == false
                c = NewNodeU(1);
                d = NewNodeU(2);
                in = insidepoly_halfplane(c,d);
                if in == false
                    Nodes(:,:,i) = NewNodeU;
                    NodesInfo(:,:,i) = [i,j];
                    i = i+1;
%                     drawnow
                    plot(c,d,'.','color','yellow')
                    if NewNodeU(1) == GoalNode(1) && NewNodeU(2) == GoalNode(2)
                        break
                    end
                end
            end
        end

        [StatusD, NewNodeD] = ActionMoveDown(CurrentNode);
        if StatusD == true
            if (any(all(bsxfun(@eq,Nodes,NewNodeD)))) == false
                c = NewNodeD(1);
                d = NewNodeD(2);
                in = insidepoly_halfplane(c,d);
                if in == false
                    Nodes(:,:,i) = NewNodeD;
                    NodesInfo(:,:,i) = [i,j];
                    i = i+1;
%                     drawnow
                    plot(c,d,'.','color','yellow') 
                    if NewNodeD(1) == GoalNode(1) && NewNodeD(2) == GoalNode(2)
                        break
                    end
                end
            end
        end

        [StatusDL, NewNodeDL] = ActionMoveDownLeft(CurrentNode);
        if StatusDL == true
            if (any(all(bsxfun(@eq,Nodes,NewNodeDL)))) == false
                c = NewNodeDL(1);
                d = NewNodeDL(2);
                in = insidepoly_halfplane(c,d);
                if in == false 
                    Nodes(:,:,i) = NewNodeDL;
                    NodesInfo(:,:,i) = [i,j];
                    i = i+1;
%                     drawnow
                    plot(c,d,'.','color','yellow') 
                    if NewNodeDL(1) == GoalNode(1) && NewNodeDL(2) == GoalNode(2)
                        break
                    end
                end
            end
        end

        [StatusDR, NewNodeDR] = ActionMoveDownRight(CurrentNode);
        if StatusDR == true
            if (any(all(bsxfun(@eq,Nodes,NewNodeDR)))) == false
                c = NewNodeDR(1);
                d = NewNodeDR(2);
                in = insidepoly_halfplane(c,d);
                if in == false 
                    Nodes(:,:,i) = NewNodeDR;
                    NodesInfo(:,:,i) = [i,j];
                    i = i+1;
%                     drawnow
                    plot(c,d,'.','color','yellow')
                    if NewNodeDR(1) == GoalNode(1) && NewNodeDR(2) == GoalNode(2)
                        break
                    end
                end
            end
        end

        [StatusUL, NewNodeUL] = ActionMoveUpLeft(CurrentNode);
        if StatusUL == true
            if (any(all(bsxfun(@eq,Nodes,NewNodeUL)))) == false
                c = NewNodeUL(1);
                d = NewNodeUL(2);
                in = insidepoly_halfplane(c,d);
                if in == false 
                    Nodes(:,:,i) = NewNodeUL;
                    NodesInfo(:,:,i) = [i,j];
                    i = i+1;
%                     drawnow
                    plot(c,d,'.','color','yellow')
                    if NewNodeUL(1) == GoalNode(1) && NewNodeUL(2) == GoalNode(2)
                        break
                    end
                end
            end
        end

        [StatusUR, NewNodeUR] = ActionMoveUpRight(CurrentNode);
        if StatusUR == true
            if (any(all(bsxfun(@eq,Nodes,NewNodeUR)))) == false
                c = NewNodeUR(1);
                d = NewNodeUR(2);
                in = insidepoly_halfplane(c,d);
                if in == false 
                    Nodes(:,:,i) = NewNodeUR;
                    NodesInfo(:,:,i) = [i,j];
                    i = i+1;
%                     drawnow
                    plot(c,d,'.','color','yellow')
                    if NewNodeUR(1) == GoalNode(1) && NewNodeUR(2) == GoalNode(2)
                        break
                    end
                end
            end
        end
        j = j+1
    end

    k = i-1;
    count = 0;
    
    drawnow 
    plot(rec)
    fill(x1,y1,'w')
    hold on
    
    drawnow
    plot(square)
    fill(x2,y2,'k')
    hold on 
    
    drawnow
    plot(poly)
    fill(x3,y3,'k')
    hold on 
    
    drawnow
    plot(x4,y4)
    xlim([0 250])
    ylim([0 150])
    fill(x4,y4,'k');
    hold on 
    
    
   txt1 = '\leftarrow Start Node';
   txt2 = '\leftarrow Goal Node';
   
    
    plot(StartNode(1),StartNode(2),'s','color','green','markers',20)
    plot(GoalNode(1),GoalNode(2),'s','color','red','markers',20)
    
    text(StartNode(1),StartNode(2),txt1)
    text(GoalNode(1),GoalNode(2),txt2)
    
    while k ~= 1 
        NodesInfo(:,:,k);
        a = Nodes(1,1,k);
        b = Nodes(1,2,k); 
        info = NodesInfo((2*k));
        k = info;
        count = count+1;
        plot(a,b,'*','color','blue')
    end
end

toc
