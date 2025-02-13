function  [event_cluster event_cur] = el_match(K,pose_cur,k,cube,P_3d,pix_threshold,enum)

    p_cur=K*pose_cur*P_3d;
    
    p_cur= [p_cur(1,:)./p_cur(3,:); (p_cur(2,:)./p_cur(3,:));ones(1,size(p_cur,2))];
    
    [~, index] = min(abs(k - cube(:,1)));
    
    start_index = index(1) - enum/2;
    
    end_index = index(1) + enum/2;
    
    start_index=max(1,start_index);
    
    event_cur=cube(start_index:end_index,:);
    
    [event_cluster] = point_cluster(p_cur,event_cur,pix_threshold);
end

function [event_cluster] = point_cluster(p_cur,event,pix_threshold)

    event_cluster=[];
    
    lines_cur=lineparasolve(p_cur);
    
    part_num=size(lines_cur,2);
    
    for i=1:size(event,1)
    
    
        for j=1:size(lines_cur,2)
    
            point_distance(j)=abs(event(i,2)*lines_cur(1,j) + event(i,3)*lines_cur(2,j) + lines_cur(3,j))/ sqrt(lines_cur(1,j)^2+lines_cur(2,j)^2);
    
        end
    
        [min_dis,min_row] = min(point_distance);
    
        point_distance(min_row) = Inf;
    
        [secondmin_dis, secondmin_row] = min(point_distance);
    
        if min_dis < pix_threshold  && secondmin_dis > 1.5*(pix_threshold)
    
            point_mid=(p_cur(1:2,min_row)+p_cur(1:2,(part_num+min_row)))./2;
    
            distance_end=norm(p_cur(1:2,min_row)-p_cur(1:2,(part_num+min_row)));
    
            point_cur=[event(i,2);event(i,3)];
    
            distance_p_mid=norm(point_cur-point_mid);
    
            if distance_p_mid<0.5*distance_end
    
                event_new     = [event(i,2);event(i,3);1;min_row];
    
                event_cluster = [event_cluster,event_new];
    
            else
                continue
            end
        else
            continue
        end
    end
end
