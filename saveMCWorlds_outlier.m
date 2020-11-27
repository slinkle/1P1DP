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
% outlierRate = 0;
% outlierRate = 0:0.05:0.8;
outlierRate = 0.5:0.1:0.8;
% depthRate = 0.5;
% featureNoises=0:0.003:0.03;
featureNoise_2d = 2;
% featureNoises=0:0.1:1;
featureNoise_3d=0.1;
savepathw = strcat(datapath,'AllMCworlds_',num2str(totalNum),'_depth.mat');

theta=60/180*pi;
phi=-15/180*pi;
rho=0.25;
Rij=[cos(theta), 0, sin(theta);...
     0           1,      0    ;...
     -sin(theta),0, cos(theta)]; %Ry(theta) from j to i (from reference to query)
tij=rho*[sin(phi);0;cos(phi)];
Tcb_c1=[Rij,tij;0 0 0 1];

theta=-60/180*pi;
phi=-165/180*pi;
rho=0.25;
Rij=[cos(theta), 0, sin(theta);...
     0           1,      0    ;...
     -sin(theta),0, cos(theta)]; %Ry(theta) from j to i (from reference to query)
tij=rho*[sin(phi);0;cos(phi)];
Tcb_c2=[Rij,tij;0 0 0 1];

Tcb_cs={eye(4),Tcb_c1,Tcb_c2};

sample_pnum=totalNum;
Allworlds=[];
% for Id=1:size(featureNoises,2)
for Id=1:size(outlierRate,2)
    fprintf('outlierRate=%f\n',outlierRate(Id));
    outrate=outlierRate(Id);
%     fprintf('featureNoise=%f\n',featureNoises(Id));
%     featureNoise=featureNoises(Id);
    worlds={};
    matchNum=round(totalNum*(1-outrate));
    posenum=1;
    while posenum<=100
        %generate different pose % random generate the pose TW0_C0
        fprintf('posenum=%d\n',posenum);
        Pjs=zeros(3,sample_pnum*length(Tcb_cs));
        xjs=zeros(3,sample_pnum*length(Tcb_cs));
        xis=zeros(3,sample_pnum*length(Tcb_cs));%bearing vector
        pis=zeros(3,sample_pnum*length(Tcb_cs));%keypoint
        norm_xis=zeros(3,sample_pnum*length(Tcb_cs));%normalized bearing vector
        v=zeros(15,sample_pnum*length(Tcb_cs));%for gp3p
        Tw_cjs=cell(1,sample_pnum*length(Tcb_cs));%reference to world
        Tcb_cis=cell(1,sample_pnum*length(Tcb_cs));%current query to base query
        camids=ones(1,sample_pnum*length(Tcb_cs));%current query to base query
%         while true
        Tijs=cell(1,length(Tcb_cs));
        for cid=1:length(Tcb_cs)
            angles = (0 + (pi/2-(0)).*rand(2,1));
            theta=angles(1);
            phi=angles(2);
            rho=(0.01 + (1-(0.01)).*rand(1));
            Rij=[cos(theta), 0, sin(theta);...
                 0           1,      0    ;...
                 -sin(theta),0, cos(theta)]; %Ry(theta) from j to i (from reference to query)
            tij=rho*[sin(phi);0;cos(phi)];
            Tijs{cid}=[Rij,tij;0 0 0 1];
        end
        Rij=Tijs{1}(1:3,1:3);
        tij=Tijs{1}(1:3,4);
        Tcb_w=Tijs{1};
        Tw_cb=inv(Tcb_w);
        
        sample_success=true;
        for cid=1:length(Tcb_cs)
%             fprintf('posenum=%d,camid=%d\n',posenum,cid);
            Tcb_ci=Tcb_cs{cid};
            Tci_cj=Tijs{cid};
            Tw_cj=Tw_cb*Tcb_ci*Tci_cj;
            R=Tci_cj(1:3,1:3);
            t=Tci_cj(1:3,4);
        
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
                    noise_p = randn(2,2)*featureNoise_2d;
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
                    pis(:,sample_pnum*(cid-1)+i)=ppi;
                    xi=K\ppi;
                    depth_noise=randn(1,1)*featureNoise_3d;
                    Pjs(:,sample_pnum*(cid-1)+i)=Tw_cj(1:3,1:3)*(Pj+[0;0;depth_noise])+Tw_cj(1:3,4);
                    xjs(:,sample_pnum*(cid-1)+i)=xj;
                    xis(:,sample_pnum*(cid-1)+i)=xi;
                    norm_xis(:,sample_pnum*(cid-1)+i)=xi./norm(xi);
                    v(:,sample_pnum*(cid-1)+i)=[xi./norm(xi);Tcb_ci(1:3,4);Tcb_ci(1,1);Tcb_ci(1,2);Tcb_ci(1,3);...
                        Tcb_ci(2,1);Tcb_ci(2,2);Tcb_ci(2,3);Tcb_ci(3,1);Tcb_ci(3,2);Tcb_ci(3,3)];
                    Tw_cjs{sample_pnum*(cid-1)+i}=Tw_cj;
                    Tcb_cis{sample_pnum*(cid-1)+i}=Tcb_ci;
                    camids(sample_pnum*(cid-1)+i)=cid;
                    break;
                end
                if ~sample_success
                    break;
                end
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
        world.Pjs=Pjs;%world points
        world.xjs=xjs;
        world.xis=xis;
        world.norm_xis=norm_xis;
        world.v=v;
        world.pis=pis;
        world.Tw_cjs=Tw_cjs;
        world.Tcb_cis=Tcb_cis;
        world.camids=camids;
        worlds=[worlds,world];
        posenum=posenum+1;
        
    end % end for each pose
    Allworlds = [Allworlds;worlds];
    
end% end for each dismatch level


save(savepathw,'Allworlds');

disp('done');   









