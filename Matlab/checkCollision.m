function isColliding = checkCollision(robot, vertices, collisionDist)
    %generate transforms for robot links
    q = robot.getpos;
    tr = zeros(4,4,robot.n+1);
    tr(:,:,1) = robot.base;
    L = robot.links;
    for i = 1 : robot.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end

    %check if link is close to vertice
    for i = 1:size(tr,3)
        trP = [tr(1, 4, i), tr(2, 4, i), tr(3, 4, i)];
        for j = 1:size(vertices, 1)
            if norm(vertices(j, :) - trP) < collisionDist
                isColliding = true;
                return;
            end
        end
    end
    isColliding = false;
end