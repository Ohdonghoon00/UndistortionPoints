#pragma once

#include <cmath>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>



typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

const std::vector<double> imu2rig_pose{-0.011773881878296, -2.212344247385963, 2.229193892963689, -0.016975989407230, 0.016444757006134, 0.128779023189435};
const std::vector<double> lidar2rig_pose{1.5620435019860173, -0.005377623186353324, 0.003014408980859652, -8.458334129298635E-4, -0.19542397891778734, -0.0012719333618026098};



Eigen::Matrix4d To44RT(Vector6d rot);

struct LidarData 
{
    std::vector<char> binary_data;
    int64_t timestamp_ns;
    int num_points, num_blocks;
    uint8_t num_channels;

    
    LidarData()
        : num_points(0), num_blocks(0), num_channels(0) {}
    virtual ~LidarData() {}
    
    
    float* points_ptr() const { return (float*) binary_data.data(); }
    uint8_t* intensities_ptr() const { return (uint8_t*) &binary_data[3 * num_points * sizeof(float)]; } // reflectivity
    uint8_t* azimuth_idxs_ptr() const { return intensities_ptr() + num_points; }
    float* azimuth_degs_ptr() const { return (float*) (azimuth_idxs_ptr() + num_points); }
};

struct DataBase
{
    std::vector<double> IMUtimestamps;
    std::vector<Eigen::Vector3f> IMUGyros;

    std::vector<double> VIOtimestamps;
    std::vector<Vector6d> VIOLidarPoses;

    std::vector<double> LidarLastseqtimestamps;
    // std::vector<double> Lidartimestamps;
    std::vector<Eigen::Matrix3Xd> LidarPoints;

    // std::map<int, int> VIOidx2Lidaridx;
    std::map<int, int> Lidaridx2VIOidx;

};

Eigen::Vector3d ToVec3(Eigen::Matrix3d rot)
{
    Eigen::AngleAxisd rod(rot);
    Eigen::Vector3d axis(rod.axis());
    double angle = rod.angle();
    axis *= angle;

    Eigen::Vector3d vec3;
    vec3 << axis.x(), axis.y(), axis.z();

    return vec3;
}

Eigen::Vector3f ToVec3(Eigen::Matrix3f rot)
{
    Eigen::AngleAxisf rod(rot);
    Eigen::Vector3f axis(rod.axis());
    float angle = rod.angle();
    axis *= angle;

    Eigen::Vector3f vec3;
    vec3 << axis.x(), axis.y(), axis.z();

    return vec3;
}

Eigen::Matrix3d ToMat33(Eigen::Vector3d rod)
{
    Eigen::AngleAxisd r(rod.norm(), rod.normalized());
    Eigen::Matrix3d rot = r.toRotationMatrix();

    return rot;
}

Eigen::Matrix3f ToMat33(Eigen::Vector3f rod)
{
    Eigen::AngleAxisf r(rod.norm(), rod.normalized());
    Eigen::Matrix3f rot = r.toRotationMatrix();

    return rot;
}

Vector6d To6DOF(Eigen::Quaterniond q, Eigen::Vector3d t)
{
    Eigen::AngleAxisd rod(q);
    Eigen::Vector3d r(rod.axis());
    double angle = rod.angle();
    r *= angle;

    Vector6d Pose;
    Pose << r.x(), r.y(), r.z(), t.x(), t.y(), t.z();

    return Pose;
}

Vector6d To6DOF(Eigen::Matrix4d RT)
{
    Eigen::Matrix3d R = RT.block<3, 3>(0, 0);
    Eigen::Vector3d rod = ToVec3(R);
    
    Vector6d Pose;
    Pose << rod.x(), rod.y(), rod.z(), RT(0, 3), RT(1, 3), RT(2, 3);
    return Pose;
}

Eigen::Quaterniond ToQuaternion(const Vector6d Pose)
{
    Eigen::Matrix4d Pos = To44RT(Pose);
    Eigen::Matrix3d rot = Pos.block<3, 3>(0, 0); 

    Eigen::Quaterniond q(rot);

    return q; 
}



Eigen::Matrix4f To44RT(Vector6f pose)
{
    Eigen::Vector3f rod;
    rod << pose[0], pose[1], pose[2];
    Eigen::Matrix3f rot = ToMat33(rod);

    Eigen::Matrix4f RT;
    RT <<   rot(0, 0), rot(0, 1), rot(0, 2), pose[3],
            rot(1, 0), rot(1, 1), rot(1, 2), pose[4],
            rot(2, 0), rot(2, 1), rot(2, 2), pose[5],
            0,         0,         0,         1;

    return RT;
}

Eigen::Matrix4d To44RT(Vector6d pose)
{
    Eigen::Vector3d rod;
    rod << pose[0], pose[1], pose[2];
    Eigen::Matrix3d rot = ToMat33(rod);

    Eigen::Matrix4d RT;
    RT <<   rot(0, 0), rot(0, 1), rot(0, 2), pose[3],
            rot(1, 0), rot(1, 1), rot(1, 2), pose[4],
            rot(2, 0), rot(2, 1), rot(2, 2), pose[5],
            0,         0,         0,         1;

    return RT;
}

Eigen::Matrix4d To44RT(std::vector<double> pose)
{
    Eigen::Vector3d rod;
    rod << pose[0], pose[1], pose[2];

    Eigen::Matrix3d R = ToMat33(rod);

    Eigen::Matrix4d RT;
    RT <<   R(0, 0), R(0, 1), R(0, 2), pose[3],
            R(1, 0), R(1, 1), R(1, 2), pose[4],
            R(2, 0), R(2, 1), R(2, 2), pose[5],
            0,       0,       0,       1;

    return RT;
}

double ToAngle(Eigen::Matrix4d LidarRotation)
{
    Eigen::Matrix3d rot = LidarRotation.block<3, 3>(0, 0);
    Eigen::AngleAxisd rod(rot);
    double angle = rod.angle();

    return angle;
}

Eigen::Vector3d ToAxis(Eigen::Matrix4d LidarRotation)
{
    Eigen::Matrix3d rot = LidarRotation.block<3, 3>(0, 0);
    Eigen::AngleAxisd rod(rot);
    Eigen::Vector3d Axis = rod.axis();    

    return Axis;
}