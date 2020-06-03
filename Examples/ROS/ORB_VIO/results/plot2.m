% plot GroundTruth And Estimated Extrinsic Parameters
clear
close all

% dataname='V1_01_easy'
% dataname='V1_02_medium'
% dataname='V1_03_difficult'
dataname='V2_01_easy'
% dataname='V2_02_medium'
% dataname='V2_03_difficult'
% dataname='MH_01_easy'
% dataname='MH_02_easy'
% dataname='MH_03_medium'
% dataname='MH_04_difficult'
% dataname='MH_05_difficult'

ite = '1'
% ite = '2'
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
subplot(431); grid on; title('Rotation from camera to IMU (Rbc)'); 
hold on; plot(timestamp, yaw_gt, 'r-','LineWidth',2);
hold on; plot(timestamp, yaw, 'b-','LineWidth',2);
% hold on; plot(15, 80:0.01:90,'g--','LineWidth',2)
% title('yaw')
axis([0 timestamp(end)+5 yaw_gt(1)-3 yaw_gt(1)+3]); 
xlabel('Time (s)'); ylabel('Yaw (degree)');
% legend('pre-calib','estimate','Location','SouthEast','orientation','hor')

% plot pitch
% figure;
subplot(432); grid on;
hold on; plot(timestamp, pitch_gt, 'r-','LineWidth',2);
hold on; plot(timestamp, pitch, 'b-','LineWidth',2);
% title('pitch')
axis([0 timestamp(end)+5 pitch_gt(1)-3 pitch_gt(1)+3]); 
xlabel('Time (s)'); ylabel('Pitch (degree)');
% legend('pre-calib','estimate','Location','SouthEast','orientation','hor')

% plot roll
% figure;
subplot(433); grid on;
hold on; plot(timestamp, roll_gt, 'r-','LineWidth',2);
hold on; plot(timestamp, roll, 'b-','LineWidth',2);
% title('roll')
axis([0 timestamp(end)+5 roll_gt(1)-3 roll_gt(1)+3]); 
xlabel('Time (s)'); ylabel('Roll (degree)');
% legend('pre-calib','estimate','Location','SouthEast','orientation','hor')

legend('Ground Truth','Ours','Location','SouthEast','orientation','hor')

%% 
%% for Rbc -- StandardDeviationOfRbcEstimation.txt

staDevOfRbc = importdata(StandardDeviationOfRbcEstimation_path);
timestamp_Rbc = Rbc_estimate.data(:,1);
timestamp = staDevOfRbc.data(:,1);
% timestamp = timestamp-timestamp_Rbc(1);
timestamp = timestamp-startTime;

staDevOfYaw = staDevOfRbc.data(:,2);
staDevOfPitch = staDevOfRbc.data(:,3);
staDevOfRoll = staDevOfRbc.data(:,4);

% title('Standard Deviation Of Rbc'); 
subplot(434); grid on;
hold on; plot(timestamp, staDevOfYaw, 'b-');
xlim([0 timestamp(end)+5]); ylim([-0.1 1]); xlabel('Time (s)'); ylabel('staDevOfYaw');

subplot(435); grid on;
hold on; plot(timestamp, staDevOfPitch, 'b-');
xlim([0 timestamp(end)+5]); ylim([-0.1 1]); xlabel('Time (s)'); ylabel('staDevOfPitch');

subplot(436); grid on;
hold on; plot(timestamp, staDevOfRoll, 'b-');
xlim([0 timestamp(end)+5]); ylim([-0.1 1]); xlabel('Time (s)'); ylabel('staDevOfRoll');

%% for pbc_refined
% figure; 
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
subplot(437); grid on; title('Translation from camera to IMU (Pbc) - after refined');
hold on; plot(timestamp, x_gt, 'r-','LineWidth',2);
hold on; plot(timestamp, x_refined, 'b-','LineWidth',2);
% title('x')
axis([0 timestamp(end)+5 x_gt(1)-0.5 x_gt(1)+0.5]); 
xlabel('Time (s)'); ylabel('X (m)');
% legend('pre-calib','estimate','Location','SouthEast','orientation','hor')

% plot y
% figure;
subplot(438); grid on;
hold on; plot(timestamp, y_gt, 'r-','LineWidth',2);
hold on; plot(timestamp, y_refined, 'b-','LineWidth',2);
% title('y')
axis([0 timestamp(end)+5 y_gt(1)-0.5 y_gt(1)+0.5]);
xlabel('Time (s)'); ylabel('Y (m)');
% legend('pre-calib','estimate','Location','SouthEast','orientation','hor')

% plot z
% figure;
subplot(439); grid on;
hold on; plot(timestamp, z_gt, 'r-','LineWidth',2);
hold on; plot(timestamp, z_refined, 'b-','LineWidth',2);
% title('z')
axis([0 timestamp(end)+5 z_gt(1)-0.5 z_gt(1)+0.5]); 
xlabel('Time (s)'); ylabel('Z (m)');

%% for Pbc -- StandardDeviationOfPbcEstimation.txt
staDevOfPbc = importdata(StandardDeviationOfPbcEstimation_path);
timestamp_Pbc = Rbc_estimate.data(:,1);
timestamp = staDevOfPbc.data(:,1);
% timestamp = timestamp-timestamp_Pbc(1);
timestamp = timestamp-startTime;

staDevOfX = staDevOfPbc.data(:,2);
staDevOfY = staDevOfPbc.data(:,3);
staDevOfZ = staDevOfPbc.data(:,4);

% title('Standard Deviation of Pbc'); 
subplot(4,3,10); grid on;
hold on; plot(timestamp, staDevOfX, 'b-');
ylim([-0.1 0.5]); xlabel('Time (s)'); ylabel('staDevOfX');

subplot(4,3,11); grid on;
hold on; plot(timestamp, staDevOfY, 'b-');
ylim([-0.1 0.5]); xlabel('Time (s)'); ylabel('staDevOfY');

subplot(4,3,12); grid on;
hold on; plot(timestamp, staDevOfZ, 'b-');
ylim([-0.1 0.5]); xlabel('Time (s)'); ylabel('staDevOfZ');
% 
% % axis([0 timestamp(end)+5 0 0.1]); 
% ylim([-0.1 1])
% xlabel('Time (s)'); ylabel('Standard Deviation of Pbc');
% legend('staDevOfX','staDevOfY', 'staDevOfZ','Location','NorthEast','orientation','ver')
% 

% %% for Rbc -- StandardDeviationOfRbcEstimation.txt
% figure; grid on;
% staDevOfRbc = importdata(StandardDeviationOfRbcEstimation_path);
% timestamp_Rbc = Rbc_estimate.data(:,1);
% timestamp = staDevOfRbc.data(:,1);
% % timestamp = timestamp-timestamp_Rbc(1);
% timestamp = timestamp-startTime;
% 
% staDevOfYaw = staDevOfRbc.data(:,2);
% staDevOfPitch = staDevOfRbc.data(:,3);
% staDevOfRoll = staDevOfRbc.data(:,4);
% 
% title('Standard Deviation Of Rbc'); 
% hold on; plot(timestamp, staDevOfYaw, 'b-');
% hold on; plot(timestamp, staDevOfPitch, 'g-');
% hold on; plot(timestamp, staDevOfRoll, 'r-');
% 
% 
% % axis([0 timestamp(end)+5 -0.1 0.5]); 
% xlim([0 timestamp(end)+5]);
% ylim([-0.1 1]);
% xlabel('Time (s)'); ylabel('Standard Deviation');
% legend('staDevOfYaw','staDevOfPitch', 'staDevOfRoll')


% %% for Pbc -- StandardDeviationOfPbcEstimation.txt
% figure; grid on;
% staDevOfPbc = importdata(StandardDeviationOfPbcEstimation_path);
% timestamp_Pbc = Rbc_estimate.data(:,1);
% timestamp = staDevOfPbc.data(:,1);
% % timestamp = timestamp-timestamp_Pbc(1);
% timestamp = timestamp-startTime;
% 
% staDevOfX = staDevOfPbc.data(:,2);
% staDevOfY = staDevOfPbc.data(:,3);
% staDevOfZ = staDevOfPbc.data(:,4);
% 
% % title('Standard Deviation of Pbc'); 
% hold on; plot(timestamp, staDevOfX, 'b-');
% hold on; plot(timestamp, staDevOfY, 'r-');
% hold on; plot(timestamp, staDevOfZ, 'g-');
% 
% % axis([0 timestamp(end)+5 0 0.1]); 
% ylim([-0.1 1])
% xlabel('Time (s)'); ylabel('Standard Deviation of Pbc');
% legend('staDevOfX','staDevOfY', 'staDevOfZ','Location','NorthEast','orientation','ver')
