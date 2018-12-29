% 动车检测和跟踪，在119-128帧间进行测试,多目标跟踪，每一帧都执行检测和跟踪
clear
close all
BeginFrame =119;       %起始帧数
EndFrame = 128;         %终止帧数
N = 3;
figure(1)    
for k = 1:N
    grname=strcat('../Data/PointCloud/',num2str(k*3+BeginFrame),'.mat');
    load(grname)
    subplot(1,3,k)
    plot3(-Ground(:,1),-Ground(:,2),Ground(:,3),'k.');
    axis equal
    axis([-80 80 -30 30 -10 10])% 用来标注输出的图线的最大值最小值。MATLAB中坐标系的设置函数
    view(90,90);%设置视点
    grid on%显示网格
end

figure(2)    
for k = 1:N
    grname=strcat('../Data/PointCloud/',num2str(k*3+BeginFrame),'.mat');
    load(grname)
    subplot(3,1,k)
    plot3(-Ground(:,1),-Ground(:,2),Ground(:,3),'k.');
    axis equal
    axis([-80 80 -30 30 -5 5])% 用来标注输出的图线的最大值最小值。MATLAB中坐标系的设置函数
    view(90,0);%设置视点
    grid on%显示网格
end

figure(3) 
grname=strcat('../Data/PointCloud/',num2str(+BeginFrame),'.mat');
load(grname)
subplot(3,1,k)
plot3(-Ground(:,1),-Ground(:,2),Ground(:,3),'k.');
axis equal
axis([-80 80 -30 30 -5 5])% 用来标注输出的图线的最大值最小值。MATLAB中坐标系的设置函数
grid on%显示网格

    
    