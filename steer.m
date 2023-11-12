function NewNode = steer(rNode, nNode, val, eps)
   newNode = [0 0 0 0 0 0];
   
   if val >= eps
       newNode(1) = nNode(1) + ((rNode(1)-nNode(1))*eps)/dist(rNode,nNode);
       newNode(2) = nNode(2) + ((rNode(2)-nNode(2))*eps)/dist(rNode,nNode);
       newNode(3) = nNode(3) + ((rNode(3)-nNode(3))*eps)/dist(rNode,nNode);
       newNode(4) = nNode(4) + ((rNode(4)-nNode(4))*eps)/dist(rNode,nNode);
       newNode(5) = nNode(5) + ((rNode(5)-nNode(5))*eps)/dist(rNode,nNode);
       newNode(6) = nNode(6) + ((rNode(6)-nNode(6))*eps)/dist(rNode,nNode);
   else
       newNode(1) = rNode(1);
       newNode(2) = rNode(2);
       newNode(3) = rNode(3);
       newNode(4) = rNode(4);
       newNode(5) = rNode(5);
       newNode(6) = rNode(6);
   end   
   NewNode = [newNode(1), newNode(2), newNode(3), newNode(4), newNode(5), newNode(6)];
end
