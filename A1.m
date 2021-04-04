  % function finalList = A1(A, start, goal)
  % 
  % Function: A1
  % 
  % Description: Implamantation of A* algorithm focusing on the huristic value
  % 
  % Parameters: 
  %     A - BitMap of the configuration space
  %     start - the starting node
  %     goal - the goal node
  % Return: 
  %     finalList - the list of nodes in the path that was found by the algorithm
function finalList = A1(A, start, goal)
%load test_A1 %for testing
r = 4;  
A = ~bwmorph(A,'skel',0.25);
A = int8(~imresize(A, 1/r));% scaling the graph for faster run time
pcolor(A); colormap(gray(2));hold on; axis equal % draw map
S = floor(start/r);
G = floor(goal/r);
if(A(S(1, :)) == 0)
    S(1) = mod(S(1), 360/1)+1
    S(2) = mod(S(2), 360/1)+1
end
if(A(G(1, :)) == 0)
    G(1, 1) = mod(G(1, 1), 360/1)+1
    G(1, 2) = mod(G(1, 2), 360/1)+1
end
plot(S(1)+.5,S(2)+.5,'.','MarkerFaceColor','g','markersize',18);
plot(G(1, 1)+.5,G(1, 2)+.5,'.','MarkerFaceColor','r','markersize',18);
if (size(G, 1) > 1)
    plot(G(2, 1)+.5,G(2, 2)+.5,'.','MarkerFaceColor','r','markersize',18);
    if(A(G(2, :)) == 0)
        G(2, 1) = mod(G(2, 1), 360/1)+1
        G(2, 2) = mod(G(2, 2), 360/1)+1
    end
end
drawnow
GoalReached = false; % reached goal flag
Node.State = S; % initialize start node
Node.Predecessor = 0; 
Node.Cost = 0;
OpenList = 1; % initialize the open list with the starting node
finalList = [];
costE = 1;
huristicE = 1;
visitedCount_A1(end+1) = 0;
exploredCount = 0;
u = [1,0;0,1;-1,0;0,-1;1,1;1,-1;-1,1;-1,-1]; % array of possible actions
count = 160;
while ~isempty(OpenList) && ~GoalReached % while there are nodes left to explore and the goal was not reached do...
    cost = [Node(OpenList).Cost];
    state = [Node(OpenList).State];
    dis1 = [sqrt((180/r - abs(180/r - abs(state(1:2:end) - G(1, 1)))).^2 + (180/r - abs(180/r - abs(state(2:2:end) - G(1, 2)))).^2)];
    if (size(G, 1) > 1)
        dis2 = [sqrt((180/r - abs(180/r - abs(state(1:2:end) - G(2, 1)))).^2 + (180/r - abs(180/r - abs(state(2:2:end) - G(2, 2)))).^2)];
        huristic = min([dis1(:)'; dis2(:)']);
    else
        huristic = dis1
    end
    [x, currentNode] = min([cost*costE + huristic*huristicE]);

    exploredCount = exploredCount+1;
    for i = 1:size(u,1) % iterate through possible actions (right up left down)
        newState = Node(OpenList(currentNode)).State + u(i,:);
        newState = mod( newState-1,360/r)+1;
        count = count+1;
        if A(newState(2),newState(1)) == 1
            count = count+1
            nextNode.State = newState; % explore next node
            nextNode.Predecessor = OpenList(currentNode);
            nextNode.Cost = 1;
            A(nextNode.State(2),nextNode.State(1)) = -1; % mark node as visited
            OpenList(end+1) = length(Node) + 1; % add node to open-list
            Node(OpenList(end)) = nextNode;
            Node(OpenList(end)).Cost = Node(Node(OpenList(end)).Predecessor).Cost + nextNode.Cost;
            visitedCount_A1(end) = visitedCount_A1(end)+1;
            plot(floor(nextNode.State(1))+.5,floor(nextNode.State(2))+.5,'sb','markersize',6,'markerfacecolor','b')
            if(any(G(:, 1) == nextNode.State(1) & G(:, 2) == nextNode.State(2)))
                GoalReached = true; % reached Goal flag
                break
            end
            

        end    
    end
    plot(floor(Node(OpenList(currentNode)).State(1))+.5,floor(Node(OpenList(currentNode)).State(2))+.5,'sb','markersize',6,'markerfacecolor','y')%draw
    drawnow limitrate
    OpenList(currentNode) = []; % remove explored node from open-list 

end

% back stepping
drawnow
i = length(Node); % start from goal node and back step to the start node
finalList(end+1, :) = Node(i).State;
while i>0 && GoalReached
    plot(floor(Node(i).State(1))+.5,floor(Node(i).State(2))+.5,'sr','markersize',6,'markerfacecolor','r'); drawnow limitrate;
    finalList(end+1, 1) = Node(i).State(1);
    finalList(end, 2) = Node(i).State(2);
    i = Node(i).Predecessor;
end
finalList = flipud(finalList);
finalList = finalList*r;
finalCount_A1(end+1) = size(finalList, 1)
disp(visitedCount_A1)
%save('test_A1.mat', 'visitedCount_A1', 'finalCount_A1')%saving the results
end