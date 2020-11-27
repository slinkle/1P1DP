%% accuracy test with increasing 2d noise of mono-camera algorithms
clear;
clc;
close all

% add your own mexopencv path below to run p3p epnp ap3p
addpath('/home/slinkle/mexopencv');

% set the camera intrinsic 
K = [800, 0, 640;
     0, 800, 480;
     0, 0, 1];
img_width=640*2;
img_height=480*2;

% number of points in the scene
totalNum=50;
% sample iteration in RANSAC
iterNum=100;

% load the synthetic world
datapath = './';
worldpath = strcat(datapath,'world/Allworlds_',num2str(totalNum),'_2dnoise.mat');
load(worldpath);%Allworlds

%% with mexopencv
% if have installed mexopencv, you can run following algorithms
% % The algorithms we want to test
% algorithms = {'p3p'; 'epnp'; 'ap3p'; '1p1dp' };
% % This defines the number of points used for every algorithm
% indices = { 4; 4; 4; 2 };
% % The name of the algorithms on the plots
% names = { 'P3P'; 'EPnP'; 'AP3P'; '1P1DP'};

%% no mexopencv
% if have installed mexopencv, you can run following algorithms
% The algorithms we want to test
algorithms = {'1p1dp' };
% This defines the number of points used for every algorithm
indices = { 2 };
% The name of the algorithms on the plots
names = { '1P1DP'};


Noise_results=[];
NoiseLevel=0:0.5:5;%2dnoise: the varying standard deviation
noise_levels=NoiseLevel;

number_noise_levels=length(NoiseLevel);
num_algorithms = size(algorithms,1);
mean_position_errors = zeros(num_algorithms,number_noise_levels);
mean_rotation_errors = zeros(num_algorithms,number_noise_levels);
median_position_errors = zeros(num_algorithms,number_noise_levels);
median_rotation_errors = zeros(num_algorithms,number_noise_levels);


% inlier re-projection error
pointTh=16;
for id=1:length(NoiseLevel)
    fprintf('Level=%f\n',NoiseLevel(id));
    for a=1:num_algorithms
        % run each algorithm with 100 cases to get mean error
        AlltranErrors=zeros(1,100);
        AllrotaErrors=zeros(1,100);
        for posenum=1:100
            fprintf('Level=%f,algorithms=%s,posenum=%d\n',NoiseLevel(id),algorithms{a},posenum);
            % 3d points in view j (in camera coordinate)
            Pjs=Allworlds{id,posenum}.Pjs;
            % bearing vector of 2d feature points in view j
            xjs=Allworlds{id,posenum}.xjs;
            % bearing vector of 2d feature points in view i (xi=inv(k)*pi)
            xis=Allworlds{id,posenum}.xis;
            % 2d feature points in view i
            pis=Allworlds{id,posenum}.pis;
            
            % ground truth of transformation from j to i
            Rij=Allworlds{id,posenum}.R;
            tij=Allworlds{id,posenum}.t;
        
            % perfrom ransac
            inliernum=[];
            Allinliers={};
            Allpose={};
            for iter=1:iterNum
                if strcmp(algorithms{a},'p3p')
                    res={};
                    pointId=randperm(size(Pjs,2));
                    pointId=pointId(1:indices{a});
                    [rvec,t]=cv.solvePnP(Pjs(:,pointId).',pis(1:2,pointId).',K,'Method','P3P');
                    R=cv.Rodrigues(rvec);% world to camera Tij
                    res=[res,[R,t; 0 0 0 1]];
                end
                if strcmp(algorithms{a},'epnp')
                    res={};
                    pointId=randperm(size(Pjs,2));
                    pointId=pointId(1:indices{a});
                    [rvec,t]=cv.solvePnP(Pjs(:,pointId).',pis(1:2,pointId).',K,'Method','EPnP');
                    R=cv.Rodrigues(rvec);% world to camera Tij
                    res=[res,[R,t; 0 0 0 1]];
                end
                if strcmp(algorithms{a},'ap3p')
                    res={};
                    pointId=randperm(size(Pjs,2));
                    pointId=pointId(1:indices{a});
                    [rvec,t]=cv.solvePnP(Pjs(:,pointId).',pis(1:2,pointId).',K,'Method','AP3P');
                    R=cv.Rodrigues(rvec);% world to camera Tij
                    res=[res,[R,t; 0 0 0 1]];
                end
                if strcmp(algorithms{a},'1p1dp')
                    % in this condition, all points have corresponding depth, so we randomly select two point ids
                    pointId=randperm(size(Pjs,2));
                    pointId=pointId(1:indices{a});
                    % the id of depth point
                    onedpid=pointId(1);
                    % the id of point without point 
                    onepid=pointId(2);
                    % xi1->Pj1: the 2d-3d correspondences from query i to
                    % reference j1
                    Pj1=Pjs(:,onedpid);
                    xi1=xis(:,onedpid);
                    % xi2->xj2: the 2d-2d correspondences from query i to
                    % reference j2
                    xj2=xjs(:,onepid);
                    xi2=xis(:,onepid);
                    % compute the Rt with 1p1dp
                    % input1: 2d bearing point of the depth point in view i
                    % input2: 3d point of the depth point in view j (in camera coordinate system)
                    % input3: 2d bearing point of the point in view i
                    % input4: 2d bearing point of the point in view j2 (can be the same as j)
                    % input5: rotation matrix from view j2 to j
                    % input6: translation matrix from view j2 to j
                    % output: estimated tranformation matrix from view j to i
                    res=compute_Rt_1p1dp(xi1,Pj1,xi2,xj2,eye(3),zeros(3,1));
                end
                
                
                for i=1:size(res,2)
                    esR=res{i}(1:3,1:3);
                    est=res{i}(1:3,4);
                    % count inliers according to the estimated RT
                    imagePoints=pis;
                    worldPoints=Pjs;
                    % re-project the points using estimated pose
                    pts = esR*worldPoints+repmat(est,1,size(worldPoints,2));
                    testimagePoints = K*pts;
                    testimagePointsd=testimagePoints(3,:).';
                    testimagePoints = [testimagePoints(1,:)./testimagePoints(3,:);testimagePoints(2,:)./testimagePoints(3,:);ones(1,size(testimagePoints,2))];
                    % compute the re-projection error of each point
                    residue = imagePoints.'- testimagePoints.';
                    pcost = sum(residue.*residue,2);
                    % find inliers
                    pinlier=pcost<pointTh & testimagePointsd>0;
                    pinliers=find(pinlier);
                    pinliers=pinliers.';
                    inliersNum=length(pinliers);
                    if inliersNum>0
                        inliernum=[inliernum,inliersNum];
                        Allinliers=[Allinliers;pinliers];
                        Allpose=[Allpose,[esR,est;0 0 0 1]];
                    end
                end
                
                
            end

            % evaluation find Rt with the max inliers
            [val,pos]=max(inliernum);
            T=Allpose{pos};
            finalR=T(1:3,1:3);
            finalt=T(1:3,4);
            tranError=norm(finalt-tij);
            quatError=rotm2axang(Rij/finalR);
            rotaError=quatError(4);
            rotaError=rotaError/pi*180;%degree
            AlltranErrors(posenum)=tranError;
            AllrotaErrors(posenum)=rotaError;
        end
        mean_position_errors(a,id) = mean(AlltranErrors);
        median_position_errors(a,id) = median(AlltranErrors);
        mean_rotation_errors(a,id) = mean(AllrotaErrors);
        median_rotation_errors(a,id) = median(AllrotaErrors);
    end
end

%%

figure(1)
plot(noise_levels',mean_rotation_errors','LineWidth',2)
legend(names,'Location','NorthWest')
xlabel('noise level [pix]')
ylabel('mean rot. error [deg]')
grid on

figure(2)
plot(noise_levels',median_rotation_errors','LineWidth',2)
legend(names,'Location','NorthWest')
xlabel('noise level [pix]')
ylabel('median rot. error [deg]')
grid on

figure(3)
plot(noise_levels',mean_position_errors','LineWidth',2)
legend(names,'Location','NorthWest')
xlabel('noise level [pix]')
ylabel('mean pos. error [m]')
grid on

figure(4)
plot(noise_levels',median_position_errors','LineWidth',2)
legend(names,'Location','NorthWest')
xlabel('noise level [pix]')
ylabel('median pos. error [m]')
grid on


