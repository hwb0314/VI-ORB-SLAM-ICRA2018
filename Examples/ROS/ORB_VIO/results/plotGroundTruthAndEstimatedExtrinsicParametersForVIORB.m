% plot GroundTruth And Estimated Extrinsic Parameters
clear
close all

% dataname='V1_01_easy'
dataname='V1_02_medium'
% dataname='V1_03_difficult'
% dataname='V2_01_easy'
% dataname='V2_02_medium'
% dataname='V2_03_difficult'
% dataname='MH_01_easy'
% dataname='MH_02_easy'
% dataname='MH_03_medium'
% dataname='MH_04_difficult'
% dataname='MH_05_difficult'

% ite = '1'
ite = '2'
% ite = '3'
% ite = '4'
% ite = '5'

R_bc_estimate_path = [dataname, '/', ite, '/', 'R_bc_estimate.txt'];
R_bc_groundtruth_path = 'R_bc_groundtruth.txt';
StandardDeviationOfRbcEstimation_path = [dataname, '/', ite, '/', 'StandardDeviationOfRbcEstimation.txt'];

p_bc_estimate_path = [dataname, '/', ite, '/', 'p_bc_estimate.txt'];
p_bc_groundtruth_path = 'p_bc_groundtruth.txt';

p_bc_refined_path = [dataname, '/', ite, '/', 'p_bc_refined.txt'];
StandardDeviationOfPbcEstimation_path = [dataname, '/', ite, '/', 'StandardDeviationOfPbcEstimation.txt'];

biasg_path = [dataname, '/', ite, '/', 'biasg.txt'];
biasa_path = [dataname, '/', ite, '/', 'biasa.txt'];
Processing_Time_path = [dataname, '/', ite, '/',  'Processing_Time.txt'];
scale_path = [dataname, '/', ite, '/', 'scale.txt'];

%% for Rbc
figure;
Rbc_estimate = importdata(R_bc_estimate_path);
timestamp = Rbc_estimate.data(:,1);
startTime = timestamp(1);
timestamp = timestamp-timestamp(1);

yaw = Rbc_estimate.data(:,2);
pitch = Rbc_estimate.data(:,3);
roll = Rbc_estimate.data(:,4);

N = size(timestamp);
Rbc_gt = importdata(R_bc_groundtruth_path);
yaw_gt(1:N(1)) = Rbc_gt.data(1);
pitch_gt(1:N(1))  = Rbc_gt.data(2);
roll_gt(1:N(1))  = Rbc_gt.data(3);

% plot yaw
% figure;
subplot(311); title('Rotation from camera to IMU (Rbc)'); 
hold on; plot(timestamp, yaw_gt, 'k-');
hold on; plot(timestamp, yaw, 'r-');
% title('yaw')
axis([0 timestamp(end)+5 yaw_gt(1)-3 yaw_gt(1)+3]); 
xlabel('Time (s)'); ylabel('Yaw (degree)');

% plot pitch
% figure;
subplot(312)
hold on; plot(timestamp, pitch_gt, 'k-');
hold on; plot(timestamp, pitch, 'r-');
% title('pitch')
axis([0 timestamp(end)+5 pitch_gt(1)-3 pitch_gt(1)+3]); 
xlabel('Time (s)'); ylabel('Pitch (degree)');

% plot roll
% figure;
subplot(313)
hold on; plot(timestamp, roll_gt, 'k-');
hold on; plot(timestamp, roll, 'r-');
% title('roll')
axis([0 timestamp(end)+5 roll_gt(1)-3 roll_gt(1)+3]); 
xlabel('Time (s)'); ylabel('Roll (degree)');

%% for Rbc -- StandardDeviationOfRbcEstimation.txt
figure; grid on;
staDevOfRbc = importdata(StandardDeviationOfRbcEstimation_path);
timestamp_Rbc = Rbc_estimate.data(:,1);
timestamp = staDevOfRbc.data(:,1);
% timestamp = timestamp-timestamp_Rbc(1);
timestamp = timestamp-startTime;

staDevOfYaw = staDevOfRbc.data(:,2);
staDevOfRoll = staDevOfRbc.data(:,3);
staDevOfPitch = staDevOfRbc.data(:,4);

title('Standard Deviation Of Rbc'); 
hold on; plot(timestamp, staDevOfYaw, 'b-');
hold on; plot(timestamp, staDevOfRoll, 'r-');
hold on; plot(timestamp, staDevOfPitch, 'g-');

% axis([0 timestamp(end)+5 -0.1 0.5]); 
xlim([0 timestamp(end)+5]);
ylim([-0.1 1]);
xlabel('Time (s)'); ylabel('Standard Deviation');
legend('staDevOfYaw','staDevOfRoll', 'staDevOfPitch')

%% for pbc first estimated
figure; 
pbc_estimate = importdata(p_bc_estimate_path);
timestamp = pbc_estimate.data(:,1);
% timestamp = timestamp-timestamp(1);
timestamp = timestamp-startTime;

x = pbc_estimate.data(:,2);
y = pbc_estimate.data(:,3);
z = pbc_estimate.data(:,4);

N = size(timestamp);
Rbc_gt = importdata(p_bc_groundtruth_path);
x_gt(1:N(1)) = Rbc_gt.data(1);
y_gt(1:N(1))  = Rbc_gt.data(2);
z_gt(1:N(1))  = Rbc_gt.data(3);

% plot x
% figure;
subplot(311); title('Translation from camera to IMU (Pbc)');
hold on; plot(timestamp, x_gt, 'k-');
hold on; plot(timestamp, x, 'r-');
% title('x')
axis([0 timestamp(end)+5 x_gt(1)-0.5 x_gt(1)+0.5]); 
xlabel('Time (s)'); ylabel('X (m)');

% plot y
% figure;
subplot(312)
hold on; plot(timestamp, y_gt, 'k-');
hold on; plot(timestamp, y, 'r-');
% title('y')
axis([0 timestamp(end)+5 y_gt(1)-0.5 y_gt(1)+0.5]);
xlabel('Time (s)'); ylabel('Y (m)');

% plot z
% figure;
subplot(313)
hold on; plot(timestamp, z_gt, 'k-');
hold on; plot(timestamp, z, 'r-');
% title('z')
axis([0 timestamp(end)+5 z_gt(1)-0.5 z_gt(1)+0.5]); 
xlabel('Time (s)'); ylabel('Z (m)');

%% for pbc_refined
figure; 
pbc_refined = importdata(p_bc_refined_path);
timestamp = pbc_refined.data(:,1);
% timestamp = timestamp-timestamp(1);
timestamp = timestamp-startTime;

x_refined = pbc_refined.data(:,2);
y_refined = pbc_refined.data(:,3);
z_refined = pbc_refined.data(:,4);

N = size(timestamp);
Rbc_gt = importdata(p_bc_groundtruth_path);
x_gt(1:N(1)) = Rbc_gt.data(1);
y_gt(1:N(1))  = Rbc_gt.data(2);
z_gt(1:N(1))  = Rbc_gt.data(3);

% plot x
% figure;
subplot(311); title('Translation from camera to IMU (Pbc) - after refined');
hold on; plot(timestamp, x_gt, 'k-');
hold on; plot(timestamp, x_refined, 'r-');
% title('x')
axis([0 timestamp(end)+5 x_gt(1)-0.5 x_gt(1)+0.5]); 
xlabel('Time (s)'); ylabel('X (m)');

% plot y
% figure;
subplot(312)
hold on; plot(timestamp, y_gt, 'k-');
hold on; plot(timestamp, y_refined, 'r-');
% title('y')
axis([0 timestamp(end)+5 y_gt(1)-0.5 y_gt(1)+0.5]);
xlabel('Time (s)'); ylabel('Y (m)');

% plot z
% figure;
subplot(313)
hold on; plot(timestamp, z_gt, 'k-');
hold on; plot(timestamp, z_refined, 'r-');
% title('z')
axis([0 timestamp(end)+5 z_gt(1)-0.5 z_gt(1)+0.5]); 
xlabel('Time (s)'); ylabel('Z (m)');

%% for Pbc -- StandardDeviationOfPbcEstimation.txt
figure; grid on;
staDevOfPbc = importdata(StandardDeviationOfPbcEstimation_path);
timestamp_Pbc = Rbc_estimate.data(:,1);
timestamp = staDevOfPbc.data(:,1);
% timestamp = timestamp-timestamp_Pbc(1);
timestamp = timestamp-startTime;

staDevOfX = staDevOfPbc.data(:,2);
staDevOfY = staDevOfPbc.data(:,3);
staDevOfZ = staDevOfPbc.data(:,4);

% title('Standard Deviation of Pbc'); 
hold on; plot(timestamp, staDevOfX, 'b-');
hold on; plot(timestamp, staDevOfY, 'r-');
hold on; plot(timestamp, staDevOfZ, 'g-');

% axis([0 timestamp(end)+5 0 0.1]); 
ylim([-0.1 1])
xlabel('Time (s)'); ylabel('Standard Deviation of Pbc');
legend('staDevOfX','staDevOfY', 'staDevOfZ','Location','NorthEast','orientation','ver')

%% for biasg
figure; grid on;
biasg = importdata(biasg_path);
timestamp = pbc_refined.data(:,1);
timestamp = timestamp-startTime;
biasg_x = biasg.data(:,2);
biasg_y = biasg.data(:,3);
biasg_z = biasg.data(:,4);

hold on; plot(timestamp, biasg_x, 'b-');
hold on; plot(timestamp, biasg_y, 'r-');
hold on; plot(timestamp, biasg_z, 'g-');

axis([0 timestamp(end)+5 -0.1 0.1]); 
xlabel('Time (s)'); ylabel('gyroscope bias (rad/s)');
legend('x','y', 'z','Location','SouthEast','orientation','hor')

%% for biasa
figure; grid on;
biasa = importdata(biasa_path);
timestamp = pbc_refined.data(:,1);
timestamp = timestamp-startTime;
biasa_x = biasa.data(:,2);
biasa_y = biasa.data(:,3);
biasa_z = biasa.data(:,4);

hold on; plot(timestamp, biasa_x, 'b-');
hold on; plot(timestamp, biasa_y, 'r-');
hold on; plot(timestamp, biasa_z, 'g-');

axis([0 timestamp(end)+5 -0.4 0.4]); 
xlabel('Time (s)'); ylabel('accelerometer bias (m/s^2)');
legend('x','y', 'z','Location','SouthEast','orientation','hor')

%% for Processing Time
figure; grid on;
PT = importdata(Processing_Time_path);
timestampTmp = PT.data(:,1);
ptTmp = PT.data(:,2);
timestamp=0;
pt=0;
for i=1:size(timestampTmp)
    if timestampTmp(i)>=startTime
        timestamp(i,:) = timestampTmp(i)-startTime;
        pt(i,:) = ptTmp(i);
    end
end
hold on; plot(timestamp, pt/1000, 'k-');
xlabel('Time (s)'); ylabel('processing time (s)');
axis([0 timestamp(end)+5 0 0.08]); 

%% for scale (s_refined)
figure; grid on;
scale = importdata(scale_path);
timestamp = scale.data(:,1);
timestamp = timestamp-startTime;
s_refined = scale.data(:,2);
s_star = scale.data(:,3);

hold on; plot(timestamp, s_refined, 'r-');
% hold on; plot(timestamp, s_star, 'b-');

% axis([0 timestamp(end)+5 0 4]); 
xlim([0 timestamp(end)+5])
xlabel('Time (s)'); ylabel('scale factor');
% legend('s\_refined','s\_star','Location','SouthEast','orientation','hor')
