function [repro_Image_left, repro_Image_right]=reproje_show(K_left, K_right, pose_cur_left, pose_cur_right, event_cur_left, event_cur_right, P_3d)

    % Create two figure windows
    figure(1);
    set(gcf,'Position',[100 100 640 480]); % Left window position
    
    % Left camera projection
    p_pro_left = K_left*[pose_cur_left]*P_3d;
    p_pro_left = [p_pro_left(1,:)./p_pro_left(3,:); p_pro_left(2,:)./p_pro_left(3,:); ones(1,size(p_pro_left,2))];
    pp1_left = p_pro_left(:,1:size(p_pro_left,2)/2);
    pp2_left = p_pro_left(:,size(p_pro_left,2)/2+1:end);
    
    % Display left event
    image_cur_left = zeros(480,640);
    for i=1:size(event_cur_left,1)
        if event_cur_left(i,3)==0
            event_cur_left(i,3)=1;
        end
        if event_cur_left(i,2)==0
            event_cur_left(i,2)=1;
        end
        image_cur_left(event_cur_left(i,3), event_cur_left(i,2)) = 255;
    end
    imshow(image_cur_left);
    title('Left Camera View', 'Color', 'white'); % Move title here and set color
    hold on
    line([pp1_left(1,:);pp2_left(1,:)],[pp1_left(2,:);pp2_left(2,:)],'color','r','LineWidth',1);
    hold off
    repro_Image_left = getframe;
    repro_Image_left = repro_Image_left.cdata;
    
    % Right camera window
    figure(2);
    set(gcf,'Position',[750 100 640 480]); % Right window position
    
    % Right camera projection
    p_pro_right = K_right*[pose_cur_right]*P_3d;
    p_pro_right = [p_pro_right(1,:)./p_pro_right(3,:); p_pro_right(2,:)./p_pro_right(3,:); ones(1,size(p_pro_right,2))];
    pp1_right = p_pro_right(:,1:size(p_pro_right,2)/2);
    pp2_right = p_pro_right(:,size(p_pro_right,2)/2+1:end);
    
    % Display right event
    image_cur_right = zeros(480,640);
    for i=1:size(event_cur_right,1)
        if event_cur_right(i,3)==0
            event_cur_right(i,3)=1;
        end
        if event_cur_right(i,2)==0
            event_cur_right(i,2)=1;
        end
        image_cur_right(event_cur_right(i,3), event_cur_right(i,2)) = 255;
    end
    imshow(image_cur_right);
    title('Right Camera View', 'Color', 'white'); % Move title here and set color
    hold on
    line([pp1_right(1,:);pp2_right(1,:)],[pp1_right(2,:);pp2_right(2,:)],'color','r','LineWidth',1);
    hold off
    repro_Image_right = getframe;
    repro_Image_right = repro_Image_right.cdata;

end