set = 9;

img_nr = 152;

width = 128;
% height = 94;

for img_nr = 65

I = imread(['Track' num2str(set) '/' num2str(img_nr) '.bmp']);

I = cat(3, I, I, I);

% IR = I(:,1:width,1,:);
% IL = I(:,width+1:end,:);

IR = I(1:end,1:width,:);
IL = I(1:end,width+1:end,:);

if set < 7

    div = 50;

    IR = I([1:div-1 div:end-2  ],1:width,:);
    IL = I([2:div   div+2:end  ],width+1:end,:);
end

if set == 9

    

    IR = I([2:end  ],1:width,:);
    IL = I([1:end-1  ],width+1:end,:);
end

height = size(IR,1);

% figure(10)
% imshow(IL)
% figure(11)
% imshow(IR)

%% 

DISP = zeros(size(IL(:,:,1)));

DIFF_L = imabsdiff(IL(:,1:end-1,1),IL(:,2:end,1));
DIFF_R = imabsdiff(IR(:,1:end-1,1),IR(:,2:end,1));

window_size = 1;
% SUM_DIFF_L = DIFF_L(:,1:end-window_size+1,:);
% SUM_DIFF_R = DIFF_R(:,1:end-window_size+1,:);
% for i = 2:window_size
%     SUM_DIFF_L = SUM_DIFF_L+DIFF_L(:,i:end-window_size+i,:);
%     SUM_DIFF_R = SUM_DIFF_R+DIFF_R(:,i:end-window_size+i,:);
% end
% DIFF_L = SUM_DIFF_L;
% DIFF_R = SUM_DIFF_R;
% imshow([DIFF_L DIFF_R])

IF_L = IL;
DISP_L = IL;

DIFF_THRESHOLD = 5;
feature_window_size = 5;
feature_window_2 = (feature_window_size-1)/2;
feature_window_vertical = 2;
feature_factor = 1.5;


feature_count = 0;

max_disparity = 15;
disparity_range = max_disparity+1;

cost = zeros(1,disparity_range);
cost_VV = zeros(1,disparity_range);

for line = 1+feature_window_vertical:height-feature_window_vertical
    
    
    max_feature = max(DIFF_L(line,:));
    features = zeros(size(DIFF_L(line,:)));
    for i = max_disparity+window_size+5: width-window_size-max_disparity
        if (DIFF_L(line,i) >= DIFF_L(line,i-1)) && (DIFF_L(line,i) >= DIFF_L(line,i+1) && DIFF_L(line,i) > DIFF_THRESHOLD )
            features(i) = DIFF_L(line,i);
            IF_L(line,i+((window_size-1)/2),1) = 255;
            feature_count = feature_count+1;
            
            for D = 0:max_disparity
                cost(D+1) = sum(sum(imabsdiff(IL(line-feature_window_vertical:line+feature_window_vertical,i-feature_window_2:i+feature_window_2,1),IR(line-feature_window_vertical:line+feature_window_vertical,i-feature_window_2-D:i+feature_window_2-D,1))));
            end
            
            [first, i_first] = min(cost);
            cost(i_first) = 1000000;
            second = min(cost);
            cost(i_first) = first;
            
%             if (second > first*feature_factor)
            if (second / first > feature_factor)
                
%                 for D = 0:max_disparity
%                     cost_VV(D+1) = sum(sum(imabsdiff(IR(line-feature_window_vertical:line+feature_window_vertical,i-i_first+1-feature_window_2:i-i_first+1+feature_window_2,1),IL(line-feature_window_vertical:line+feature_window_vertical,i-i_first+1-feature_window_2+D:i-i_first+1+feature_window_2+D,1))));
%                 end
%                 
%                 [check, i_check] = min(cost_VV);
                
%                 if ( i_first == i_check )
%                 
%                     DISP(line,i) = i_first;
%                     DISP_L(line,i,1) = 255;
%                 end
                
                
                    DISP(line,i) = i_first;
                    DISP_L(line,i,1) = 255;
            end
            
            
            
        end
    end
    
end

% sum(features>0)
% feature_ratio = sum(features>0)/length(features)

feature_count
feature_ratio = feature_count/(width*height)

% figure(2)
% hold on
% plot(DIFF_L(line,:))
% plot(features,'dr')
% % plot(DIFF_R(line,:),'r')
% hold off

level = 0.1;

BW_L = im2bw(DIFF_L, level);
BW_R = im2bw(DIFF_R, level);

sum(sum(BW_L==1))/(sum(sum(BW_L==1))+sum(sum(BW_L==0)))

figure(1)
subplot(2,2,1:2)
imshow([IF_L IR DISP_L])

subplot(2,2,3)
image(DISP,'CDataMapping','scaled')
caxis([0,16])

end