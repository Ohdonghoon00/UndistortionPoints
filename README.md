# UndistortionPoints

## Inputs
- Lidar Raw Points ( distortion points )
- VIO Poses results all camera frames
- IMU Gyro

## Build
```
 git clone https://github.com/Ohdonghoon00/UndistortionPoints.git
 mkdir build
 cd build
 cmake ..
 make
```

## Run
```
./UndirtortionPoints DATA_DIR/
Ex) ./UndirtortionPoints /home/multipleye/Dataset/201014_skt_lobby_day_lidar/
```

## Data Structure Ex
```
 DATA_DIR
    ├─ lidar_timestamp.csv // lidar timestamp and fidx			
    ├─ lidar	// Raw lidar points
    	├─ 00000.xyz
    	├─ 00001.xyz
    	└─ ...						
    ├─ imu_data.csv // IMU data
    └─ rovins2_all_frames.txt // VIO pose
```

# Output File
```
Data_DIR
    ├─ lidar2	// Undistortion lidar points
    	├─ 00000.bin
    	├─ 00001.bin
    	└─ ...
    └─ VIOPoses_lidarframes.txt // Lidar Pose by Lidar Frames ( Result of VIO )
```
