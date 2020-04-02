clearvars
close all
%计算运行时间
tic
x_max = 640;
y_max = 480;
z_max = 400;
frame_range = [x_max,y_max,z_max];

EPS = 20;
numNodes = 2000;        

q_start.coord = [0 0 0];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [640 400 180];
q_goal.cost = 0;

nodes(1) = q_start;
figure(1)

%障碍物设置
n_collision = 10;%障碍物个数
v(:,:,1)=[200 270 200;200 220 200;130 220 200;130 270 200;200 270 0;200 220 0;130 220 0;130 270 0];
f= [1 2 3 4;2 6 7 3;4 3 7 8;1 5 8 4;1 2 6 5;5 6 7 8];
patch('Faces',f,'Vertices',v(:,:,1),'FaceColor','b');
view(30,30)
axis equal
hold on 
v(:,:,2)=v(:,:,1)+[200 100 50;200 100 50;200 100 50;200 100 50;200 100 0;200 100 0;200 100 0;200 100 0];
f= [1 2 3 4;2 6 7 3;4 3 7 8;1 5 8 4;1 2 6 5;5 6 7 8];
patch('Faces',f,'Vertices',v(:,:,2),'FaceColor','b');
view(30,30)
axis equal
hold on 
v(:,:,3)=v(:,:,2)+[70 -130 -30;70 -130 -30;70 -130 -30;70 -130 -30;70 -130 0;70 -130 0;70 -130 0;70 -130 0];
f= [1 2 3 4;2 6 7 3;4 3 7 8;1 5 8 4;1 2 6 5;5 6 7 8];
patch('Faces',f,'Vertices',v(:,:,3),'FaceColor','b');
view(30,30)
axis equal
hold on 
v(:,:,4)=v(:,:,2)+[70 -130 -200;70 -130 -200;70 -130 -200;70 -130 -200;70 -130 0;70 -130 0;70 -130 0;70 -130 0]/2;
f= [1 2 3 4;2 6 7 3;4 3 7 8;1 5 8 4;1 2 6 5;5 6 7 8];
patch('Faces',f,'Vertices',v(:,:,4),'FaceColor','b');
view(30,30)
axis equal
hold on

creat_center = [431.0204  252.9379  100.1604;153.5249  369.0092  250.7006;...
  511.1925  377.2976  228.5198; 510.3887  128.2985  267.8933;...
  145.3908  115.6336  205.9847; 171.4108  369.3130  194.9958;...
  261.1689  208.6961  117.5507; 75.7016  202.8517   37.7578;...
  514.8571  155.2304  159.8630; 114.3705   300.7156  226.4555];
lengthxyz = [43.8046   40.8767   47.9097; 33.8555   36.4555   51.2546;...
   51.3858   48.2733   54.1205;   43.2755   40.0164   33.2863;...
   45.1787   34.4898   38.7004;   58.0563   48.4366   49.3885;...
   57.5073   37.2771   46.2652;   43.4833   43.6257   50.0703;...
   45.5035   42.6180   58.3824;   35.0895   42.3162   35.3476];

v = createObstacle(n_collision,creat_center,lengthxyz);

exp_center.coord=[0 0 0];
v_center(1)=exp_center;
for num=1:1:n_collision
    v_center(num).coord=[((max(v(:,1,num))+min(v(:,1,num)))/2) ((max(v(:,2,num))+min(v(:,2,num)))/2) ((max(v(:,3,num))+min(v(:,3,num)))/2)];
    v_r(num)=dist_3d(v(1,:,num),v_center(num).coord);
end

for i = 1:1:numNodes
    %产生随机点
    r_create = 300;
    m = length(nodes);
    q_rand = creatPoint(frame_range,nodes(m),r_create,v,n_collision);%[rand(1)*x_max rand(1)*y_max rand(1)*z_max];
    plot3(q_rand(1), q_rand(2), q_rand(3), 'x', 'Color',  [0 0.4470 0.7410])
    
    % 判断是否到达目标点
%     for j = 1:1:length(nodes)
%         if dist_3d(nodes(j).coord,q_goal.coord) < 20
%             break
%         end
%     end
    flag = 0;
    for j = 1:1:length(nodes)
        if dist_3d(nodes(j).coord,q_goal.coord) < 5
            flag = 1;
            break
        end
    end
    if flag == 1
        break
    end
    
    % 从节点列表中选取里随机点最近的节点
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist_3d(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    pho = 2;k_att = 1.5;k_rep = 1.5;
    q_new.coord = steer3d(q_rand, q_near.coord, q_goal.coord, pho, k_att,frame_range,k_rep,v_center,v_r,n_collision);
    if collisionChecking(q_new.coord,q_near.coord,v,n_collision)
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
    
        q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost;
    
        % 找到半径r以内所有的节点
        q_nearest = [];
        r = 50;
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if collisionChecking(q_new.coord,nodes(j).coord,v,n_collision)&&(dist_3d(nodes(j).coord, q_new.coord)) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
    
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
    
        % Iterate through all nearest neighbors to find alternate lower
        % cost paths
    
        for k = 1:1:length(q_nearest)
            if (q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord) < C_min)&&collisionChecking(q_new.coord,q_nearest(k).coord,v,n_collision)
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord);
                line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], [q_min.coord(3), q_new.coord(3)], 'Color', 'g');            
                hold on
            end
        end
    
        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
    
        % Append to nodes
        nodes = [nodes q_new];
    end
end
%拓展节点个数
disp(length(nodes));
%路径长度
disp(nodes(length(nodes)).cost);

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist_3d(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], [q_end.coord(3), nodes(start).coord(3)], 'Color', 'r', 'LineWidth', 4);
    hold on
    q_end = nodes(start);
end
disp(['程序运行时间为',num2str(toc),'s']);

% 引入斥力思想
function qnewx = Repulsion(qnew,k_rep,v_center,v_r,n_collision)
    for i=1:1:n_collision
        if dist_3d(qnew,v_center(i).coord)<1.2*v_r(i)
            qnewx = qnew + k_rep*(qnew-v_center(i).coord)/dist_3d(qnew,v_center(i).coord);
        else
            qnewx = qnew;
        end
    end
end
%引入引力思想
function qnew = steer3d(q_rand, q_near, q_goal,pho, k_att,frame_range,k_rep,v_center,v_r,n_collision)
   qnew_1 = q_near(1) + pho*((q_rand(1)-q_near(1))/dist_3d(q_rand,q_near) + k_att*(q_goal(1)-q_near(1))/dist_3d(q_rand,q_near));
   qnew_2 = q_near(2) + pho*((q_rand(2)-q_near(2))/dist_3d(q_rand,q_near) + k_att*(q_goal(2)-q_near(2))/dist_3d(q_rand,q_near));
   qnew_3 = q_near(3) + pho*((q_rand(3)-q_near(3))/dist_3d(q_rand,q_near) + k_att*(q_goal(3)-q_near(3))/dist_3d(q_rand,q_near));
   qnew = [qnew_1,qnew_2,qnew_3];
   qnew = Repulsion(qnew,k_rep,v_center,v_r,n_collision);
   if ~InFrame(frame_range,qnew)
       qnew = q_rand;
   end
end

%计算三维距离
function d = dist_3d(q1,q2)
    d = sqrt((q1(1)-q2(1))^2 + (q1(2)-q2(2))^2 + (q1(3)-q2(3))^2);
end

%判断是否在障碍物中
function feasible=in_obstacle(q,v,n)
    feasible = true;
    for i=1:1:n
        if (q(1)<v(1,1,i))&&(q(1)>v(3,1,i))&&(q(2)<v(1,2,i))&&(q(2)>v(2,2,i))&&(q(3)<v(1,3,i))&&(q(3)>v(5,3,i))
            feasible = false;
        end
    end
end

%碰撞检测
function feasible = collisionChecking(q_new,last_q,v,n)
    feasible = true;
    delta_q = (q_new-last_q)/100;
    q = last_q;
    for i = 1:1:300
        q = q + delta_q;
        if ~in_obstacle(q,v,n)
            feasible = false;
            break
        end
    end
end

%判断是否在坐标系中
function pointIn = InFrame(frame_range,q_rand)
    pointIn = false;
    if (q_rand(1)>0)&&(q_rand(1)<frame_range(1))&&(q_rand(2)>0)&&(q_rand(2)<frame_range(2))&&(q_rand(3)>0)&&(q_rand(3)<frame_range(3))
        pointIn = true;
    end
end

%随机点产生，在一个球中
function q_rand = creatPoint(frame_range,last_q,r,v,n)
    r_rand = rand(1)*r;
%     r_rand = exp(-pi*r_rand.^2)*r;
    theta = rand(1)*pi;
    pho = rand(1)*2*pi;
    q_rand1 = last_q.coord(1) + r_rand*sin(theta)*cos(pho);
    q_rand2 = last_q.coord(2) + r_rand*sin(theta)*sin(pho);
    q_rand3 = last_q.coord(3) + r_rand*cos(theta);
    q_rand = [q_rand1,q_rand2,q_rand3];
    if (~in_obstacle(q_rand,v,n))||(~InFrame(frame_range,q_rand))
        q_rand = [640 400 180];%[rand(1)*frame_range(1) rand(1)*frame_range(2) rand(1)*frame_range(3)];
%         continue
    end
end

%障碍物产生函数
function v = createObstacle(n_collision,creat_center,lengthxyz)
    v(:,:,1)=[200 270 200;200 220 200;130 220 200;130 270 200;200 270 0;200 220 0;130 220 0;130 270 0];
    v(:,:,2)=v(:,:,1)+[200 100 50;200 100 50;200 100 50;200 100 50;200 100 0;200 100 0;200 100 0;200 100 0];
    v(:,:,3)=v(:,:,2)+[70 -130 -30;70 -130 -30;70 -130 -30;70 -130 -30;70 -130 0;70 -130 0;70 -130 0;70 -130 0];
    v(:,:,4)=v(:,:,2)+[70 -180 -200;70 -180 -200;70 -180 -200;70 -180 -200;70 -180 0;70 -180 0;70 -180 0;70 -180 0]/2;
    for i=5:1:n_collision
        point1 = [creat_center(i,1)+lengthxyz(i,1) creat_center(i,2)+lengthxyz(i,2) creat_center(i,3)+lengthxyz(i,3)];
        point2 = [creat_center(i,1) creat_center(i,2)+lengthxyz(i,2) creat_center(i,3)+lengthxyz(i,3)];
        point3 = [creat_center(i,1) creat_center(i,2) creat_center(i,3)+lengthxyz(i,3)];
        point4 = [creat_center(i,1)+lengthxyz(i,1) creat_center(i,2) creat_center(i,3)+lengthxyz(i,3)];
        point5 = [creat_center(i,1)+lengthxyz(i,1) creat_center(i,2)+lengthxyz(i,2) 0];
        point6 = [creat_center(i,1) creat_center(i,2)+lengthxyz(i,2) 0];
        point7 = [creat_center(i,1) creat_center(i,2) 0];
        point8 = [creat_center(i,1)+lengthxyz(i,1) creat_center(i,2) 0];
        v(:,:,i) = [point1;point2;point3;point4;point5;point6;point7;point8];
        f= [1 2 3 4;2 6 7 3;4 3 7 8;1 5 8 4;1 2 6 5;5 6 7 8];
        patch('Faces',f,'Vertices',v(:,:,i),'FaceColor','b');
        view(30,30)
        axis equal
        hold on
    end
end