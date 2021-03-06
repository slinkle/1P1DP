%% accuracy test with increasing 2d noise of multi-camera algorithms
clear;
clc;
close all

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
worldpath = strcat(datapath,'world/AllMCworlds_',num2str(totalNum),'_2dnoise.mat');
load(worldpath);%All multi camera worlds

% The algorithms we want to test
algorithms = {'mc1p1dp' };
% This defines the number of points used for every algorithm
indices = { 2 };
% The name of the algorithms on the plots
names = { 'MC1P1DP'};


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
            % 3d points in world (base reference camera coordinate system)
            Pjs=Allworlds{id,posenum}.Pjs;
            % bearing vector of all 2d feature points in all view j
            xjs=Allworlds{id,posenum}.xjs;
            % bearing vector of all 2d feature points in all view i
            xis=Allworlds{id,posenum}.xis;
            % the normalized bearing vector of 2d feature points in all
            % view i
            norm_xis=Allworlds{id,posenum}.norm_xis;
            % input data for gp3p
            v=Allworlds{id,posenum}.v;
            % 2d feature points in all view i
            pis=Allworlds{id,posenum}.pis;
            
            % ground truth of transformation from world to base query
            % camera
            Rij=Allworlds{id,posenum}.R;
            tij=Allworlds{id,posenum}.t;
            % the transformation of each reference camera to world of each point
            Tw_cjs=Allworlds{id,posenum}.Tw_cjs;
            % the transformation of each query camera to base query camera
            Tcb_cis=Allworlds{id,posenum}.Tcb_cis;
        
            % perfrom ransac
            inliernum=[];
            Allinliers={};
            Allpose={};
            for iter=1:iterNum
                if strcmp(algorithms{a},'mc1p1dp')
                    pointId=randperm(size(Pjs,2));
                    pointId=pointId(1:indices{a});
                    onedpid=pointId(1);
                    onepid=pointId(2);
                    % current reference camera to world
                    T_wj1=Tw_cjs{onedpid};
                    % world to reference camera 1
                    T_j1w=inv(T_wj1);
                    % xi1->Pj1: the 2d-3d correspondences from query i to
                    % reference j
                    Pj1=T_j1w(1:3,1:3)*Pjs(:,onedpid)+T_j1w(1:3,4);
                    xi1=xis(:,onedpid);
                    % xi2->xj2: the 2d-2d correspondences from query i2 to
                    % reference j2
                    xj2=xjs(:,onepid);
                    xi2=xis(:,onepid);
                    T_wj2=Tw_cjs{onepid};
                    % transformation matrix from view j2 to j
                    Tjj2=T_wj1\T_wj2;
                    % transformation matrix from view i to i2
                    Ti2i=Tcb_cis{onepid}\Tcb_cis{onedpid};
                    % compute the Rt with mc1p1dp
                    % input1: 2d bearing vector of the depth point in view
                    % i
                    % input2: 3d point of the depth point in view j (in camera coordinate system)
                    % input3: 2d bearing vector of the point in view i2
                    % input4: 2d bearing vector of the point in view j2 
                    % input5: transformation matrix from view i to i2
                    % input6: transformation matrix from view j2 to j
                    % output: estimated tranformation matrix from view j
                    % to i
                    res=compute_Rt_mc1p1dp(xi1,Pj1,xi2,xj2,Ti2i,Tjj2);
                    Tcb_ci=Tcb_cis{onedpid};
                    for ri=1:length(res)
                        res{ri}=Tcb_ci*res{ri}*T_j1w;%world to base query camera
                    end
                end
                
                
                for i=1:size(res,2)
                    % count inliers according to the estimated RT
                    pinliers=[];
                    for ji=1:size(pis,2)
                        imagePoints=pis(:,ji);
                        worldPoints=Pjs(:,ji);
                        esT=Tcb_cis{ji}\res{i};
                        esR=esT(1:3,1:3);
                        est=esT(1:3,4);
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
                        if pinlier
                            pinliers=[pinliers,ji];
                        end
                    end
                    inliersNum=length(pinliers);
                    if inliersNum>0
                        inliernum=[inliernum,inliersNum];
                        Allinliers=[Allinliers;pinliers];
                        Allpose=[Allpose,res{i}];
                    end
                end
                
                
            end

            % evaluation find Rt with the max inliers
            if isempty(Allpose)
                tranError=999;
                rotaError=999;
            else
                [val,pos]=max(inliernum);
                T=Allpose{pos};
                finalR=T(1:3,1:3);
                finalt=T(1:3,4);
                tranError=norm(finalt-tij);
                quatError=rotm2axang(Rij/finalR);
                rotaError=quatError(4);
                rotaError=rotaError/pi*180;%degree
            end
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