#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <glog/logging.h>

#include <Eigen/Core>

#include "common.h"



// Extrinsic parameter rig - imu / rig - lidar
const Eigen::Matrix4d IMUToRig = To44RT(imu2rig_pose);
const Eigen::Matrix4d LidarToRig = To44RT(lidar2rig_pose);

LidarData lidar_data;
DataBase DB;

Eigen::Matrix4d gyroToRotation(Eigen::Vector3f gyro, float timediff)
{
    // float t = 0.005; // 200Hz
    float angle_x(gyro[0] * timediff), angle_y(gyro[1] * timediff), angle_z(gyro[2] * timediff);  
    
    Eigen::Matrix3f data_x;
    data_x <<   1.0, 0.0, 0.0,
                0.0, cos(angle_x), sin(angle_x),
                0.0, -sin(angle_x), cos(angle_x);

    Eigen::Matrix3f data_y;
    data_y <<   cos(angle_y), 0.0, -sin(angle_y),
                0.0, 1.0, 0.0,
                sin(angle_y), 0.0, cos(angle_y);

    Eigen::Matrix3f data_z;
    data_z <<   cos(angle_z), sin(angle_z), 0.0,
                -sin(angle_z), cos(angle_z), 0.0,
                0.0, 0.0, 1.0;

    Eigen::Matrix3f rot = data_x * data_y * data_z;
    Eigen::Matrix4d RT;
    RT <<   (double)rot(0, 0), (double)rot(0, 1), (double)rot(0, 2), 0,
            (double)rot(1, 0), (double)rot(1, 1), (double)rot(1, 2), 0,
            (double)rot(2, 0), (double)rot(2, 1), (double)rot(2, 2), 0,
            0,         0,         0,         1;

    return RT;
}

void MoveDistortionPoints(std::vector<Eigen::Vector3d> &points, const Eigen::Matrix4d LidarRotation, int ScanStepNum, int num_seqs)
{
    int PointNum = points.size();
    Eigen::MatrixXd MatrixPoints(4, PointNum);
    for(int i = 0; i < PointNum; i++){
        MatrixPoints(0, i) = points[i].x();
        MatrixPoints(1, i) = points[i].y();
        MatrixPoints(2, i) = points[i].z();
        MatrixPoints(3, i) = 1.0;
    }
    
    double angle = ToAngle(LidarRotation);
    Eigen::Vector3d Axis = ToAxis(LidarRotation);
    
    double AngleRatio = (((double)(num_seqs - ScanStepNum) / (double)num_seqs) * angle);
    Axis = (Axis * AngleRatio);
    
    Eigen::Matrix3d R = ToMat33(Axis);

    Eigen::Matrix4d RT;
    RT <<   R(0, 0), R(0, 1), R(0, 2), LidarRotation(0, 3),
            R(1, 0), R(1, 1), R(1, 2), LidarRotation(1, 3),
            R(2, 0), R(2, 1), R(2, 2), LidarRotation(2, 3),
            0,       0,       0,       1;

    // move points
    Eigen::Matrix4Xd MatrixPoints_ = RT * MatrixPoints;
    points.clear();
    for(int i = 0; i < PointNum; i++){
        Eigen::Vector3d Point;
        Point.x() = MatrixPoints_(0, i);
        Point.y() = MatrixPoints_(1, i);
        Point.z() = MatrixPoints_(2, i);
        points.push_back(Point);
    }

}

int ReadIMUdata(const std::string Path, DataBase* db)
{
    std::ifstream IMUcsvFile(Path, std::ifstream::in);

    if(!IMUcsvFile.is_open()){
        std::cout << " IMU csv file failed to open " << std::endl;
        return EXIT_FAILURE;
    }

    std::string IMUcsvline;
    int IMUline_num = 0;
    

    
    // Read IMU Data csv
    while(std::getline(IMUcsvFile, IMUcsvline))
    {
        if(IMUline_num == 0){
            IMUline_num++;
            continue;
        }
            
        std::string IMUvalue;
        std::vector<std::string> IMUvalues;
            
        // IMUvalues[0] -> Timestamp (ns)
        // IMUvalues[1] -> Gyro_x
        // IMUvalues[2] -> Gyro_y
        // IMUvalues[3] -> Gyro_z
        // IMUvalues[4] -> Acc_x
        // IMUvalues[5] -> Acc_y
        // IMUvalues[6] -> Acc_z

        std::stringstream ss(IMUcsvline);
        while(std::getline(ss, IMUvalue, ','))
            IMUvalues.push_back(IMUvalue);
            
        db->IMUtimestamps.push_back(std::stod(IMUvalues[0]) * 10e-10); // sec

        Eigen::Vector3f Gyro;
        Gyro.resize(3);
        Gyro[0] = std::stof(IMUvalues[1]);
        Gyro[1] = std::stof(IMUvalues[2]);
        Gyro[2] = std::stof(IMUvalues[3]);
        db->IMUGyros.push_back(Gyro);
            
        IMUline_num++;
    } 

    IMUcsvFile.close();
    return EXIT_SUCCESS;
}

int ReadVIOdata(const std::string Path, DataBase *db)
{
    std::ifstream Rovins2PoseFile(Path, std::ifstream::in);

    if(!Rovins2PoseFile.is_open()){
        std::cout << " Rovins2Pose file failed to open " << std::endl;
        return EXIT_FAILURE;
    }

    std::string line;
    int line_num = 0;

    while(std::getline(Rovins2PoseFile, line)){

        std::string value;
        std::vector<std::string> values;        

        // values[0]        -> camera fidx
        // values[1] ~ [6]  -> rig pose
        // values[7]        -> timestamps 
        
        std::stringstream ss(line);
        while(std::getline(ss, value, ' '))
            values.push_back(value);

        Vector6d pos;
        pos << std::stod(values[1]), std::stod(values[2]), std::stod(values[3]), std::stod(values[4]), std::stod(values[5]), std::stod(values[6]);
        Eigen::Matrix4d Rigpos = To44RT(pos);
        Eigen::Matrix4d Lidarpos = LidarToRig.inverse() * Rigpos * LidarToRig;

        // Save Lidar Pose by VIO all Frames ( Result of VIO )
        db->VIOtimestamps.push_back(std::stod(values[7]) * 10e-10 ); // sec
        db->VIOLidarPoses.push_back(To6DOF(Lidarpos));

        line_num++;
    }

    Rovins2PoseFile.close();
    return EXIT_SUCCESS;
}






int ReadLidardata(const std::string Path, const std::string LidarBinaryPath, DataBase* db, bool ToUndistortionPoints)
{
    std::ifstream LidarcsvFile(Path, std::ifstream::in);

    if(!LidarcsvFile.is_open()){
        std::cout << " Lidar csv file failed to open " << std::endl;
        return EXIT_FAILURE;
    }
    
    // Read Lidar timestamp.csv
    Eigen::Matrix4d LidarRotation;
    std::string Lidarcsvline;
    int Lidarline_num(1);
    size_t IMUcount(0);

    while(std::getline(LidarcsvFile, Lidarcsvline))
    {
        if(Lidarline_num == 1){
            Lidarline_num++;
            continue;
        }
        std::string value;
        std::vector<std::string> values;
        
        // values[0] -> First Seq Timestamp (ns)
        // values[1] -> Last Seq Timestamp (ns)
        // values[2] -> Fidx
        // values[3] -> Num pts
        // values[4] -> Date
        
        std::stringstream ss(Lidarcsvline);
        while(std::getline(ss, value, ','))
            values.push_back(value);
        int fidx = std::stoi(values[2]);
        double LidarScantimestamp = std::stod(values[1]) * 10e-10;
        
        // Binary Data Path
        std::stringstream Lidar_binary_path;
        Lidar_binary_path <<    LidarBinaryPath << std::setfill('0') << 
                                std::setw(5) << fidx << ".xyz";
        
        std::ifstream ifs(Lidar_binary_path.str(), std::ifstream::in);
        
        if (!ifs.is_open()){
            std::cout << "xyz file failed to open: " << std::endl;
            return EXIT_FAILURE;
        }        
                
        // Read Binary data file
        int num_seqs = 0;
        ifs.read((char*)&num_seqs, sizeof(int));

        // Integral IMU rotation
        Eigen::Matrix4d IMURotation_integral = Eigen::Matrix4d::Identity();
        while(LidarScantimestamp > db->IMUtimestamps[IMUcount]){
            
            float timediff = 0;
            if(IMUcount == 0) timediff = 0.005;
            else{
                timediff = db->IMUtimestamps[IMUcount] - db->IMUtimestamps[IMUcount - 1];
            }

            Eigen::Matrix4d IMURotation = gyroToRotation(db->IMUGyros[IMUcount], timediff);
            IMURotation_integral = IMURotation * IMURotation_integral;
            IMUcount++;
            if(IMUcount == db->IMUtimestamps.size()) break;
        }
        
        // IMU Rotation -> Lidar Motion
        Eigen::Matrix4d RT_ = IMUToRig * IMURotation_integral * IMUToRig.inverse();
        Eigen::Matrix4d RT = LidarToRig.inverse() * RT_ * LidarToRig;
        LidarRotation = RT;

        
        std::vector<Eigen::Vector3d> TotalPoints;
        for (int j = 0; j < num_seqs; j++){

            
            std::vector<Eigen::Vector3d> Points;
            int& num_pts = lidar_data.num_points;
            ifs.read((char*)&num_pts, sizeof(int));
            
            lidar_data.binary_data.resize((4 * num_pts) * sizeof(float) + 2 * num_pts);
            

            ifs.read((char*) lidar_data.points_ptr(), num_pts * sizeof(float) * 3);
            ifs.read((char*) lidar_data.intensities_ptr(), num_pts );
            
            ifs.read((char*) lidar_data.azimuth_idxs_ptr(), num_pts);    
            ifs.read((char*) &lidar_data.num_blocks, sizeof(int));
            CHECK_LE(lidar_data.num_blocks, num_pts);  
            ifs.read((char*) lidar_data.azimuth_degs_ptr(),
                    lidar_data.num_blocks * sizeof(float));
            ifs.read((char*) &lidar_data.num_channels, sizeof(uint8_t));
            ifs.read((char*) &lidar_data.timestamp_ns, sizeof(int64_t));


            // save 3D points and intensity 
            for(int k = 0; k < num_pts; k++){

                Eigen::Vector3d point;
                point.x() = (double)*(lidar_data.points_ptr() + 3 * k);
                point.y() = (double)*(lidar_data.points_ptr() + 3 * k + 1);
                point.z() = (double)*(lidar_data.points_ptr() + 3 * k + 2);
                Points.push_back(point);
            }
            
            // UndistortionPoints
            if(ToUndistortionPoints) MoveDistortionPoints(Points, LidarRotation, j, num_seqs);

            for(int k = 0; k < Points.size(); k++){

                Eigen::Vector3d point;
                point.x() = Points[k].x();
                point.y() = Points[k].y();
                point.z() = Points[k].z();
                TotalPoints.push_back(point);
            }
        }

        Eigen::Matrix3Xd UndistortionPoints(3, TotalPoints.size());
        for(size_t i = 0; i < TotalPoints.size(); i++){
            UndistortionPoints(0, i) = TotalPoints[i].x();
            UndistortionPoints(1, i) = TotalPoints[i].y();
            UndistortionPoints(2, i) = TotalPoints[i].z();
        }
        
        db->LidarPoints.push_back(UndistortionPoints);
        
        double timestamp_ = lidar_data.timestamp_ns * 10e-10; //sec
        db->LidarLastseqtimestamps.push_back(timestamp_);

        ifs.close();
        Lidarline_num++;
    }    
    
    LidarcsvFile.close();
    return EXIT_SUCCESS;
}
        


int main(int argc, char** argv)
{

    std::string data_dir = argv[1];

    ////////// Read IMU data ///////////
    std::cout << " Load imu Data ... " << std::endl;
    std::string IMUcsvPath = data_dir + "imu_data.csv";
    ReadIMUdata(IMUcsvPath, &DB);
    
    ////////// VIO data /////////////
    std::cout << " Load VIO Data ... " << std::endl;
    std::string Rovins2PosePath = data_dir + "rovins2_all_frames.txt";
    

    // for camera idx (no use)
    Vector6d p;
    p << 0, 0, 0, 0, 0, 0;
    DB.VIOLidarPoses.push_back(p);
    DB.VIOtimestamps.push_back(0.0);

    ReadVIOdata(Rovins2PosePath, &DB);

    //////////// Lidar timestamp.csv path ////////////////
    std::cout << " Load Lidar Data ... " << std::endl;
    std::string LidarcsvPath = data_dir + "lidar_timestamp.csv";
    std::string Lidar_binary_path = data_dir + "lidar/";
    ReadLidardata(LidarcsvPath, Lidar_binary_path, &DB, true);
    
    ////////////// Find Lidar idx - Camera idx /////////////////
    for(size_t i = 0; i < DB.LidarLastseqtimestamps.size(); i++){
        
        double MinVal = 1000;
        int Minidx = -1;
        
        for(size_t j = 1; j < DB.VIOtimestamps.size(); j++){
            
            double DiffTime = std::fabs(DB.LidarLastseqtimestamps[i] - DB.VIOtimestamps[j]);
            if(DiffTime < MinVal){
                MinVal = DiffTime;
                Minidx = j;
            }
        }
        DB.Lidaridx2VIOidx[i] = Minidx;
    }
    



    // Save VIO pose and UndistortionPoints
    std::cout << "SaveFile" << std::endl;
    std::ofstream VIOPose2LidarFrames;
    std::string VIOposefile = data_dir + "VIOPoses_lidarframes.txt";
    VIOPose2LidarFrames.open(VIOposefile);

    std::string LidarBinaryPath = data_dir + "lidar2/";
    for(size_t i = 0; i < DB.LidarLastseqtimestamps.size(); i++){
        
        std::stringstream Lidar_binary_path_;
        Lidar_binary_path_ <<    LidarBinaryPath << std::setfill('0') << 
                                std::setw(5) << i << ".bin";
        std::ofstream OutPoints(Lidar_binary_path_.str(), std::ios::out | std::ios::binary);
        
        int PointNum = DB.LidarPoints[i].cols();
        float point[DB.LidarPoints[i].cols() * 3];
        for(int j = 0; j < DB.LidarPoints[i].cols(); j++){
            point[3 * j] = static_cast<float>(DB.LidarPoints[i](0, j));
            point[3 * j + 1] = static_cast<float>(DB.LidarPoints[i](1, j));
            point[3 * j + 2] = static_cast<float>(DB.LidarPoints[i](2, j));
        }

        OutPoints.write(reinterpret_cast<char*>(&PointNum) , sizeof(int));
        OutPoints.write(reinterpret_cast<char*>(&point) , sizeof(point));        
        
        // Save Lidar Pose by Lidar Frames ( Result of VIO )
        int VIOidx = DB.Lidaridx2VIOidx[i];
        VIOPose2LidarFrames << VIOidx << " " << std::setprecision(7) << DB.VIOLidarPoses[VIOidx][0] << " " << DB.VIOLidarPoses[VIOidx][1] << " " << DB.VIOLidarPoses[VIOidx][2] << " "
                            << DB.VIOLidarPoses[VIOidx][3] << " " << DB.VIOLidarPoses[VIOidx][4] << " " << DB.VIOLidarPoses[VIOidx][5] << std::endl;
    
        OutPoints.close();
    }

    
    VIOPose2LidarFrames.close();
    return 0;
}



