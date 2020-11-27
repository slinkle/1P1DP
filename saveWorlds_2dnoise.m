clear;
clc;

% set an camera intrinsic
K = [800, 0, 640;
     0, 800, 480;
     0, 0, 1];

img_width=640*2;
img_height=480*2;
%% 

datapath = './world/';

totalNum=50;
outlierRate = 0;
% outlierRate = 0:0.1:0.8;
% depthRate = 0.5;
% featureNoises=0:0.003:0.03;
featureNoise_2d = 0:0.5:5;
% featureNoises=0:0.1:1;
featureNoise_3d=0.05;
savepathw = strcat(datapath,'Allworlds_',num2str(totalNum),'_2dnoise.mat');

sample_pnum=totalNum;
Allworlds=[];
for Id=1:size(featureNoise_2d,2)
% for Id=1:size(outlierRate,2)
    fprintf('featureNoise_2d=%f\n',featureNoise_2d(Id));
%     outrate=outlierRate(Id);
%     fprintf('featureNoise=%f\n',featureNoises(Id));
    featureNoise=featureNoise_2d(Id);
    worlds={};
    matchNum=round(totalNum*(1-outlierRate));
    posenum=1;
    while posenum<=100
        %generate different pose % random generate the pose TW0_C0
        fprintf('posenum=%d\n',posenum);
        Pjs=zeros(3,sample_pnum);
        xjs=zeros(3,sample_pnum);
        xis=zeros(3,sample_pnum);%bearing vector
        pis=zeros(3,sample_pnum);%keypoint
        norm_xis=zeros(3,sample_pnum);%normalized bearing vector
%         while true
        angles = (0 + (pi/2-(0)).*rand(2,1));
        theta=angles(1);
        phi=angles(2);
        rho=(0.01 + (1-(0.01)).*rand(1));
        Rij=[cos(theta), 0, sin(theta);...
             0           1,      0    ;...
             -sin(theta),0, cos(theta)]; %Ry(theta) from j to i (from camera to world)
        tij=rho*[sin(phi);0;cos(phi)];
        R=Rij;
        t=tij;
        sample_success=true;
        for i=1:sample_pnum
            % generate 3d points in view j
            test_num=0;
            while true
                test_num=test_num+1;
                if test_num>10000
                    sample_success=false;
                    break;
                end
                or_points3d_xy = (-3 + (3-(-3)).*rand(2,1));
                or_points3d_z = (2 + (8-(2)).*rand(1));
                Pj=[or_points3d_xy;or_points3d_z];
                if i>matchNum
                    angles2 = (0 + (pi/2-(0)).*rand(2,1));
                    theta2=angles2(1);
                    phi2=angles2(2);
                    rho2=(0.01 + (1-(0.01)).*rand(1));
                    Rij2=[cos(theta2), 0, sin(theta2);...
                         0           1,      0    ;...
                         -sin(theta2),0, cos(theta2)]; %Ry(theta) from j to i (from camera to world)
                    tij2=rho2*[sin(phi2);0;cos(phi2)];
                    R=Rij2;
                    t=tij2;
                end
                pCi=R*Pj+t;
                if pCi(3)<=0
    %                 disp('o');
                    continue;
                end
                noise_p = randn(2,2)*featureNoise;
                pCj=Pj/Pj(3);
                pj=K*pCj+[noise_p(:,1);0];
                if pj(1)<=0||pj(1)>img_width||pj(2)<=0||pj(2)>img_height
                    continue;
                end
                xj=K\pj;%bearing2d

                pCi=pCi/pCi(3);
                ppi=K*pCi+[noise_p(:,2);0];
                if ppi(1)<=0||ppi(1)>img_width||ppi(2)<=0||ppi(2)>img_height
                    continue;
                end
                pis(:,i)=ppi;
                xi=K\ppi;
                depth_noise=randn(1,1)*featureNoise_3d;
                Pjs(:,i)=Pj+[0;0;depth_noise];
                xjs(:,i)=xj;
                xis(:,i)=xi;
                norm_xis(:,i)=xi./norm(xi);
                break;
            end
            if ~sample_success
                break;
            end
        end
        if ~sample_success
            continue;
        end
        world.R=Rij;
        world.t=tij;
        world.theta=theta;
        world.phi=phi;
        world.rho=rho;
        world.Pjs=Pjs;
        world.xjs=xjs;
        world.xis=xis;
        world.norm_xis=norm_xis;
        world.pis=pis;
        worlds=[worlds,world];
        posenum=posenum+1;
        
    end % end for each pose
    Allworlds = [Allworlds;worlds];
    
end% end for each dismatch level


save(savepathw,'Allworlds');

disp('done');   









