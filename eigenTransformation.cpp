#include <iostream>
#include <math.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>



#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching.hpp>

#include "PCLUtilities.h"
#include "Display1.h"

const Eigen::Matrix4d outputAsMatrix(const Eigen::Quaterniond& q, const Eigen::Vector3d& translational_vec)
{
    const Eigen::Matrix3d R = q.normalized().toRotationMatrix();

    // std::cout << "q: " << q(0) << ", " << q(1) << ", "  <<  q(2) << ", " <<  q(3) << ", " << '\n';

    Eigen::Matrix4d transformation_mat;
    transformation_mat.setIdentity();   
    transformation_mat.block<3,3>(0,0) = R;
    transformation_mat.block<3,1>(0,3) = translational_vec;

    return transformation_mat;
}

const Eigen::Matrix4d outputAsMatrixTranspose(const Eigen::Quaterniond& q, const Eigen::Vector3d& translational_vec)
{
    const Eigen::Matrix3d R = q.normalized().toRotationMatrix();

    Eigen::Matrix4d transformation_mat;
    // Eigen::Matrix4d temp_mat;
    transformation_mat.setIdentity();
    // temp_mat.setIdentity();   
    transformation_mat.block<3,3>(0,0) = R;
    transformation_mat.block<3,1>(0,3) = translational_vec;

    // transformation_mat = transformation_mat*temp_mat;

    return transformation_mat.inverse();
}


int main(int argc, const char** argv) 
{
    Eigen::Quaterniond q_lidar_to_egoFrame1;
    q_lidar_to_egoFrame1.w() = 0.7077955119163518;
    q_lidar_to_egoFrame1.x() = -0.006492242056004365;
    q_lidar_to_egoFrame1.y() = 0.010646214713995808;
    q_lidar_to_egoFrame1.z() = -0.7063073142877817;

    const Eigen::Vector3d translational_vec_lidar_to_egoFrame1(0.943713,
    0.0,
    1.84023);
    Eigen::Matrix4d 
    transformation_mat_lidar_to_egoFrame1 = outputAsMatrix(q_lidar_to_egoFrame1, translational_vec_lidar_to_egoFrame1);

    Eigen::Quaterniond q_egoFrame1_to_global;
    q_egoFrame1_to_global.w() = 0.5720320396729045;
    q_egoFrame1_to_global.x() = -0.0016977771610471074;
    q_egoFrame1_to_global.y() = 0.011798001930183783;
    q_egoFrame1_to_global.z() = -0.8201446642457809;

    const Eigen::Vector3d translational_vec_egoFrame1_to_global(411.3039349319818,
    1180.8903791765097,
    0.0);
    Eigen::Matrix4d 
    transformation_mat_egoFrame1_to_global = outputAsMatrix(q_egoFrame1_to_global, translational_vec_egoFrame1_to_global);

    // std::cout << "transformation_mat_egoFrame1_to_global size = \n" << transformation_mat_egoFrame1_to_global.rows() << "x" << transformation_mat_egoFrame1_to_global.cols()  << '\n';
    std::cout << "transformation_mat_egoFrame1_to_global size = \n" << transformation_mat_egoFrame1_to_global  << '\n';

    //Camera front  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    Eigen::Quaterniond q_egoFrameCamFront_to_global;
    q_egoFrameCamFront_to_global.w() = 0.5720063498929273;
    q_egoFrameCamFront_to_global.x() = -0.0021434844534272707;
    q_egoFrameCamFront_to_global.y() = 0.011564094980151613;
    q_egoFrameCamFront_to_global.z() = -0.8201648693182716;

    const Eigen::Vector3d translational_vec_egoFrameCamFront_to_global(411.4199861830012,
    1181.197175631848,
    0.0);
    Eigen::Matrix4d 
    transformation_mat_global_to__egoFrameCamFront = outputAsMatrixTranspose(q_egoFrameCamFront_to_global, translational_vec_egoFrameCamFront_to_global);

    Eigen::Quaterniond q_camera_to_egoFrameCamFront;
    q_camera_to_egoFrameCamFront.w() = 0.4998015430569128;
    q_camera_to_egoFrameCamFront.x() = -0.5030316162024876;
    q_camera_to_egoFrameCamFront.y() = 0.4997798114386805;
    q_camera_to_egoFrameCamFront.z() = -0.49737083824542755;

    const Eigen::Vector3d translational_vec_camera_to_egoFrameCamFront(1.70079118954,
    0.0159456324149,
    1.51095763913);
    Eigen::Matrix4d 
    transformation_mat_egoFrameFront_to_camera = outputAsMatrixTranspose(q_camera_to_egoFrameCamFront, translational_vec_camera_to_egoFrameCamFront);

    //Camera front right xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    Eigen::Quaterniond q_egoFrameCamFrontRight_to_global;
    q_egoFrameCamFrontRight_to_global.w() = 0.5719976683022334;
    q_egoFrameCamFrontRight_to_global.x() = -0.0020681756810697035;
    q_egoFrameCamFrontRight_to_global.y() = 0.011639346518095882;
    q_egoFrameCamFrontRight_to_global.z() = -0.820170052927313;

    const Eigen::Vector3d translational_vec_egoFrameCamFrontRight_to_global(411.3940085709597,
    1181.1288333610287,
    0.0);
    Eigen::Matrix4d 
    transformation_mat_global_to__egoFrameCamFrontRight = outputAsMatrixTranspose(q_egoFrameCamFrontRight_to_global, translational_vec_egoFrameCamFrontRight_to_global);

    Eigen::Quaterniond q_camera_to_egoFrameCamFrontRight;
    q_camera_to_egoFrameCamFrontRight.w() = 0.2060347966337182;
    q_camera_to_egoFrameCamFrontRight.x() = -0.2026940577919598;
    q_camera_to_egoFrameCamFrontRight.y() = 0.6824507824531167;
    q_camera_to_egoFrameCamFrontRight.z() = -0.6713610884174485 ;

    const Eigen::Vector3d translational_vec_camera_to_egoFrameCamFrontRight(1.5508477543,
    -0.493404796419,
    1.49574800619);
    Eigen::Matrix4d 
    transformation_mat_egoFrameCamFrontRight_to_camera = outputAsMatrixTranspose(q_camera_to_egoFrameCamFrontRight, translational_vec_camera_to_egoFrameCamFrontRight);

    //Camera front left xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    Eigen::Quaterniond q_egoFrameCamFrontLeft_to_global;
    q_egoFrameCamFrontLeft_to_global.w() = 0.5720093912295269;
    q_egoFrameCamFrontLeft_to_global.x() = -0.002216202910524616;
    q_egoFrameCamFrontLeft_to_global.y() = 0.011491368029605502;
    q_egoFrameCamFrontLeft_to_global.z() = -0.8201635771300098;

    const Eigen::Vector3d translational_vec_egoFrameCamFrontLeft_to_global(411.4449780367985,
    1181.2631893914647,
    0.0);
    Eigen::Matrix4d 
    transformation_mat_global_to__egoFrameCamFrontLeft = outputAsMatrixTranspose(q_egoFrameCamFrontLeft_to_global, translational_vec_egoFrameCamFrontLeft_to_global);

    Eigen::Quaterniond q_camera_to_egoFrameCamFrontLeft;
    q_camera_to_egoFrameCamFrontLeft.w() = 0.6757265034669446;
    q_camera_to_egoFrameCamFrontLeft.x() = -0.6736266522251881;
    q_camera_to_egoFrameCamFrontLeft.y() = 0.21214015046209478;
    q_camera_to_egoFrameCamFrontLeft.z() = -0.21122827103904068;

    const Eigen::Vector3d translational_vec_camera_to_egoFrameCamFrontLeft(1.52387798135,
    0.494631336551,
    1.50932822144);
    Eigen::Matrix4d 
    transformation_mat_egoFrameCamFrontLeft_to_camera = outputAsMatrixTranspose(q_camera_to_egoFrameCamFrontLeft, translational_vec_camera_to_egoFrameCamFrontLeft);

    //Camera back xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    Eigen::Quaterniond q_egoFrameCamBack_to_global;
    q_egoFrameCamBack_to_global.w() = 0.5720139793595926;
    q_egoFrameCamBack_to_global.x() = -0.0018477237033291815;
    q_egoFrameCamBack_to_global.y() = 0.0117641593359254;
    q_egoFrameCamBack_to_global.z() = -0.820157422626558;

    const Eigen::Vector3d translational_vec_egoFrameCamBack_to_global(411.33787291696916,
    1180.980332035968,
    0.0);
    Eigen::Matrix4d 
    transformation_mat_global_to__egoFrameCamBack = outputAsMatrixTranspose(q_egoFrameCamBack_to_global, translational_vec_egoFrameCamBack_to_global);

    Eigen::Quaterniond q_camera_to_egoFrameCamBack;
    q_camera_to_egoFrameCamBack.w() = 0.5037872666382278;
    q_camera_to_egoFrameCamBack.x() = -0.49740249788611096;
    q_camera_to_egoFrameCamBack.y() = -0.4941850223835201;
    q_camera_to_egoFrameCamBack.z() = 0.5045496097725578;

    const Eigen::Vector3d translational_vec_camera_to_egoFrameCamBack(0.0283260309358,
    0.00345136761476,
    1.57910346144);
    Eigen::Matrix4d 
    transformation_mat_egoFrameCamBack_to_camera = outputAsMatrixTranspose(q_camera_to_egoFrameCamBack, translational_vec_camera_to_egoFrameCamBack);

    //Camera back left xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    Eigen::Quaterniond q_egoFrameCamBackLeft_to_global;
    q_egoFrameCamBackLeft_to_global.w() = 0.572035626175666;
    q_egoFrameCamBackLeft_to_global.x() = -0.0017054511514414208;
    q_egoFrameCamBackLeft_to_global.y() = 0.011796400634764472;
    q_egoFrameCamBackLeft_to_global.z() = -0.8201421698426733;

    const Eigen::Vector3d translational_vec_egoFrameCamBackLeft_to_global(411.3057690287572,
    1180.8950018232533,
    0.0);
    Eigen::Matrix4d 
    transformation_mat_global_to__egoFrameCamBackLeft = outputAsMatrixTranspose(q_egoFrameCamBackLeft_to_global, translational_vec_egoFrameCamBackLeft_to_global);

    Eigen::Quaterniond q_camera_to_egoFrameCamBackLeft;
    q_camera_to_egoFrameCamBackLeft.w() = 0.6924185592174665;
    q_camera_to_egoFrameCamBackLeft.x() = -0.7031619420114925;
    q_camera_to_egoFrameCamBackLeft.y() = -0.11648342771943819;
    q_camera_to_egoFrameCamBackLeft.z() = 0.11203317912370753;

    const Eigen::Vector3d translational_vec_camera_to_egoFrameCamBackLeft(1.03569100218,
    0.484795032713,
    1.59097014818);
    Eigen::Matrix4d 
    transformation_mat_egoFrameCamBackLeft_to_camera = outputAsMatrixTranspose(q_camera_to_egoFrameCamBackLeft, translational_vec_camera_to_egoFrameCamBackLeft);

    //Camera back right xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    Eigen::Quaterniond q_egoFrameCamBackRight_to_global;
    q_egoFrameCamBackRight_to_global.w() = 0.571997253729231;
    q_egoFrameCamBackRight_to_global.x() = -0.0019833418894953424;
    q_egoFrameCamBackRight_to_global.y() = 0.011713250499854123;
    q_egoFrameCamBackRight_to_global.z() = -0.820169499459651;

    const Eigen::Vector3d translational_vec_egoFrameCamBackRight_to_global(411.36921032161575,
    1181.0634380795361,
    0.0);
    Eigen::Matrix4d 
    transformation_mat_global_to__egoFrameCamBackRight = outputAsMatrixTranspose(q_egoFrameCamBackRight_to_global, translational_vec_egoFrameCamBackRight_to_global);

    Eigen::Quaterniond q_camera_to_egoFrameCamBackRight;
    q_camera_to_egoFrameCamBackRight.w() = 0.12280980120078765;
    q_camera_to_egoFrameCamBackRight.x() = -0.132400842670559;
    q_camera_to_egoFrameCamBackRight.y() = -0.7004305821388234;
    q_camera_to_egoFrameCamBackRight.z() = 0.690496031265798;

    const Eigen::Vector3d translational_vec_camera_to_egoFrameCamBackRight(1.0148780988,
    -0.480568219723,
    1.56239545128);
    Eigen::Matrix4d 
    transformation_mat_egoFrameCamBackRight_to_camera = outputAsMatrixTranspose(q_camera_to_egoFrameCamBackRight, translational_vec_camera_to_egoFrameCamBackRight);

    //Camera Front Instrinsic xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    Eigen::MatrixXd camera_instrinsic_front(3, 4);
    camera_instrinsic_front << 1266.417203046554, 0.0, 816.2670197447984, 0.0,
                        0.0, 1266.417203046554, 491.50706579294757, 0.0,
                        0.0, 0.0, 1.0, 0.0;

    
    //Camera Front Right Instrinsic xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    Eigen::MatrixXd camera_instrinsic_frontRight(3, 4);
    camera_instrinsic_frontRight << 1260.8474446004698, 0.0, 807.968244525554, 0.0,
                        0.0, 1260.8474446004698, 495.3344268742088, 0.0,
                        0.0, 0.0, 1.0, 0.0;

    //Camera Front Left Instrinsic xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    Eigen::MatrixXd camera_instrinsic_frontLeft(3, 4);
    camera_instrinsic_frontLeft << 1272.5979470598488, 0.0, 826.6154927353808, 0.0,
                        0.0, 1272.5979470598488, 479.75165386361925, 0.0,
                        0.0, 0.0, 1.0, 0.0;

    //Camera Back Instrinsic xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    Eigen::MatrixXd camera_instrinsic_back(3, 4);
    camera_instrinsic_back << 809.2209905677063, 0.0, 829.2196003259838, 0.0,
                        0.0, 809.2209905677063, 481.77842384512485, 0.0,
                        0.0, 0.0, 1.0, 0.0;

    //Camera Back Left Instrinsic xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    Eigen::MatrixXd camera_instrinsic_backLeft(3, 4);
    camera_instrinsic_backLeft << 1256.7414812095406, 0.0, 792.1125740759628, 0.0,
                        0.0, 1256.7414812095406, 492.7757465151356, 0.0,
                        0.0, 0.0, 1.0, 0.0;


    //Camera Back Right Instrinsic xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    Eigen::MatrixXd camera_instrinsic_backRight(3, 4);
    camera_instrinsic_backRight << 1259.5137405846733, 0.0, 807.2529053838625, 0.0,
                        0.0, 1259.5137405846733, 501.19579884916527, 0.0,
                        0.0, 0.0, 1.0, 0.0;


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRgb (new pcl::PointCloud<pcl::PointXYZRGB>());
    float* f = new float [5];
    std::size_t count {0};
    const std::string& fileName = "../../LIDAR_TOP/n015-2018-07-24-11-22-45+0800__LIDAR_TOP__1532402927647951.pcd.bin";
    loadPcd<pcl::PointXYZ>(cloud, fileName, f, count);


    cv::Mat transformation_mat_lidar_to_egoFrame1_cv2(4,4,cv::DataType<double>::type, transformation_mat_lidar_to_egoFrame1.data());
    cv::Mat transformation_mat_egoFrame1_to_global_cv2(4,4,cv::DataType<double>::type, transformation_mat_egoFrame1_to_global.data());

    cv::Mat transformation_mat_global_to__egoFrameCamFront_cv2(4,4,cv::DataType<double>::type, transformation_mat_global_to__egoFrameCamFront.data());
    cv::Mat transformation_mat_egoFrameFront_to_camera_cv2(4,4,cv::DataType<double>::type, transformation_mat_egoFrameFront_to_camera.data());

    cv::Mat transformation_mat_global_to__egoFrameCamFrontRight_cv2(4,4,cv::DataType<double>::type, transformation_mat_global_to__egoFrameCamFrontRight.data());
    cv::Mat transformation_mat_egoFrameCamFrontRight_to_camera_cv2(4,4,cv::DataType<double>::type, transformation_mat_egoFrameCamFrontRight_to_camera.data());

    cv::Mat transformation_mat_global_to__egoFrameCamFrontLeft_cv2(4,4,cv::DataType<double>::type, transformation_mat_global_to__egoFrameCamFrontLeft.data());
    cv::Mat transformation_mat_egoFrameCamFrontLeft_to_camera_cv2(4,4,cv::DataType<double>::type, transformation_mat_egoFrameCamFrontLeft_to_camera.data());

    cv::Mat transformation_mat_global_to__egoFrameCamBack_cv2(4,4,cv::DataType<double>::type, transformation_mat_global_to__egoFrameCamBack.data());
    cv::Mat transformation_mat_egoFrameCamBack_to_camera_cv2(4,4,cv::DataType<double>::type, transformation_mat_egoFrameCamBack_to_camera.data());

    cv::Mat transformation_mat_global_to__egoFrameCamBackLeft_cv2(4,4,cv::DataType<double>::type, transformation_mat_global_to__egoFrameCamBackLeft.data());
    cv::Mat transformation_mat_egoFrameCamBackLeft_to_camera_cv2(4,4,cv::DataType<double>::type, transformation_mat_egoFrameCamBackLeft_to_camera.data());

    cv::Mat transformation_mat_global_to__egoFrameCamBackRight_cv2(4,4,cv::DataType<double>::type, transformation_mat_global_to__egoFrameCamBackRight.data());
    cv::Mat transformation_mat_egoFrameCamBackRight_to_camera_cv2(4,4,cv::DataType<double>::type, transformation_mat_egoFrameCamBackRight_to_camera.data());
    
    cv::Mat camera_instrinsic_front_cv2(4,3,cv::DataType<double>::type, camera_instrinsic_front.data());
    cv::Mat camera_instrinsic_frontRight_cv2(4,3,cv::DataType<double>::type, camera_instrinsic_frontRight.data());
    cv::Mat camera_instrinsic_frontLeft_cv2(4,3,cv::DataType<double>::type, camera_instrinsic_frontLeft.data());
    cv::Mat camera_instrinsic_back_cv2(4,3,cv::DataType<double>::type, camera_instrinsic_back.data());
    cv::Mat camera_instrinsic_backLeft_cv2(4,3,cv::DataType<double>::type, camera_instrinsic_backLeft.data());
    cv::Mat camera_instrinsic_backRight_cv2(4,3,cv::DataType<double>::type, camera_instrinsic_backRight.data());

    transformation_mat_lidar_to_egoFrame1_cv2 = transformation_mat_lidar_to_egoFrame1_cv2.t();
    transformation_mat_egoFrame1_to_global_cv2 = transformation_mat_egoFrame1_to_global_cv2.t();

    transformation_mat_global_to__egoFrameCamFront_cv2 = transformation_mat_global_to__egoFrameCamFront_cv2.t();
    transformation_mat_egoFrameFront_to_camera_cv2 = transformation_mat_egoFrameFront_to_camera_cv2.t();

    transformation_mat_global_to__egoFrameCamFrontRight_cv2 = transformation_mat_global_to__egoFrameCamFrontRight_cv2.t();
    transformation_mat_egoFrameCamFrontRight_to_camera_cv2 = transformation_mat_egoFrameCamFrontRight_to_camera_cv2.t();

    transformation_mat_global_to__egoFrameCamFrontLeft_cv2 = transformation_mat_global_to__egoFrameCamFrontLeft_cv2.t();
    transformation_mat_egoFrameCamFrontLeft_to_camera_cv2 = transformation_mat_egoFrameCamFrontLeft_to_camera_cv2.t();

    transformation_mat_global_to__egoFrameCamBack_cv2 = transformation_mat_global_to__egoFrameCamBack_cv2.t();
    transformation_mat_egoFrameCamBack_to_camera_cv2 = transformation_mat_egoFrameCamBack_to_camera_cv2.t();

    transformation_mat_global_to__egoFrameCamBackLeft_cv2 = transformation_mat_global_to__egoFrameCamBackLeft_cv2.t();
    transformation_mat_egoFrameCamBackLeft_to_camera_cv2 = transformation_mat_egoFrameCamBackLeft_to_camera_cv2.t();

    transformation_mat_global_to__egoFrameCamBackRight_cv2 = transformation_mat_global_to__egoFrameCamBackRight_cv2.t();
    transformation_mat_egoFrameCamBackRight_to_camera_cv2 = transformation_mat_egoFrameCamBackRight_to_camera_cv2.t();

    camera_instrinsic_front_cv2 = camera_instrinsic_front_cv2.t();
    camera_instrinsic_frontRight_cv2 = camera_instrinsic_frontRight_cv2.t();
    camera_instrinsic_frontLeft_cv2 = camera_instrinsic_frontRight_cv2.t();
    camera_instrinsic_back_cv2 = camera_instrinsic_back_cv2.t();
    camera_instrinsic_backLeft_cv2 = camera_instrinsic_backLeft_cv2.t();
    camera_instrinsic_backRight_cv2 = camera_instrinsic_backRight_cv2.t();


    cv::Mat X(4,1,cv::DataType<double>::type);
    cv::Mat Y_front(3,1,cv::DataType<double>::type);
    cv::Mat Y_frontRight(3,1,cv::DataType<double>::type);
    cv::Mat Y_frontLeft(3,1,cv::DataType<double>::type);
    cv::Mat Y_back(3,1,cv::DataType<double>::type);
    cv::Mat Y_backLeft(3,1,cv::DataType<double>::type);
    cv::Mat Y_backRight(3,1,cv::DataType<double>::type);

    cv::Mat imgFront = cv::imread("../../CAM_FRONT/n015-2018-07-24-11-22-45+0800__CAM_FRONT__1532402927612460.jpg");
    cv::Mat imgFrontRight = cv::imread("../../CAM_FRONT_RIGHT/n015-2018-07-24-11-22-45+0800__CAM_FRONT_RIGHT__1532402927620339.jpg");
    cv::Mat imgFrontLeft = cv::imread("../../CAM_FRONT_LEFT/n015-2018-07-24-11-22-45+0800__CAM_FRONT_LEFT__1532402927604844.jpg");
    cv::Mat imgBack = cv::imread("../../CAM_BACK/n015-2018-07-24-11-22-45+0800__CAM_BACK__1532402927637525.jpg");
    cv::Mat imgBackLeft = cv::imread("../../CAM_BACK_LEFT/n015-2018-07-24-11-22-45+0800__CAM_BACK_LEFT__1532402927647423.jpg");
    cv::Mat imgBackRight = cv::imread("../../CAM_BACK_RIGHT/n015-2018-07-24-11-22-45+0800__CAM_BACK_RIGHT__1532402927627893.jpg");


    cv::Mat visimgFront = imgFront.clone(); 
    cv::Mat visimgFrontRight = imgFrontRight.clone();
    cv::Mat visimgFrontLeft = imgFrontLeft.clone();
    cv::Mat visimgBack = imgBack.clone();
    cv::Mat visimgBackLeft = imgBackLeft.clone();
    cv::Mat visimgBackRight = imgBackRight.clone();

    cv::Mat overlayFront = visimgFront.clone();
    cv::Mat overlayFrontRight = visimgFrontRight.clone();
    cv::Mat overlayFrontLeft = visimgFrontLeft.clone();
    cv::Mat overlayBack = visimgBack.clone();
    cv::Mat overlayBackLeft = visimgBackLeft.clone();
    cv::Mat overlayBackRight = visimgBackRight.clone();

    const uint32_t imgFrontHeight = imgFront.size().height;
    const uint32_t imgFrontWidth = imgFront.size().width;

    const uint32_t imgFrontRightHeight = imgFrontRight.size().height;
    const uint32_t imgFrontRightWidth = imgFrontRight.size().width;

    const uint32_t imgFrontLeftHeight = imgFrontLeft.size().height;
    const uint32_t imgFrontLeftWidth = imgFrontLeft.size().width;

    const uint32_t imgBackHeight = imgBack.size().height;
    const uint32_t imgBackWidth = imgBack.size().width;

    const uint32_t imgBackLeftHeight = imgBackLeft.size().height;
    const uint32_t imgBackLeftWidth = imgBackLeft.size().width;

    const uint32_t imgBackRightHeight = imgBackRight.size().height;
    const uint32_t imgBackRightWidth = imgBackRight.size().width;

    std::cout << "imgBackRightHeight: " << imgBackRightHeight << '\n';
    std::cout << "imgBackRightWidth: " << imgBackRightWidth << '\n';

    for(std::size_t i=0; i < cloud->points.size() ; ++i) 
    {
            X.at<double>(0, 0) = cloud->at(i).x;
            X.at<double>(1, 0) = cloud->at(i).y;
            X.at<double>(2, 0) = cloud->at(i).z;
            X.at<double>(3, 0) = 1;

            Y_front = camera_instrinsic_front_cv2*transformation_mat_egoFrameFront_to_camera_cv2*transformation_mat_global_to__egoFrameCamFront_cv2*transformation_mat_egoFrame1_to_global_cv2*transformation_mat_lidar_to_egoFrame1_cv2*X;
            Y_frontRight = camera_instrinsic_frontRight_cv2*transformation_mat_egoFrameCamFrontRight_to_camera_cv2*transformation_mat_global_to__egoFrameCamFrontRight_cv2*transformation_mat_egoFrame1_to_global_cv2*transformation_mat_lidar_to_egoFrame1_cv2*X;
            Y_frontLeft = camera_instrinsic_frontRight_cv2*transformation_mat_egoFrameCamFrontLeft_to_camera_cv2*transformation_mat_global_to__egoFrameCamFrontLeft_cv2*transformation_mat_egoFrame1_to_global_cv2*transformation_mat_lidar_to_egoFrame1_cv2*X;
            Y_back = camera_instrinsic_back_cv2*transformation_mat_egoFrameCamBack_to_camera_cv2*transformation_mat_global_to__egoFrameCamBack_cv2*transformation_mat_egoFrame1_to_global_cv2*transformation_mat_lidar_to_egoFrame1_cv2*X;
            Y_backLeft = camera_instrinsic_backLeft_cv2*transformation_mat_egoFrameCamBackLeft_to_camera_cv2*transformation_mat_global_to__egoFrameCamBackLeft_cv2*transformation_mat_egoFrame1_to_global_cv2*transformation_mat_lidar_to_egoFrame1_cv2*X;
            Y_backRight = camera_instrinsic_backRight_cv2*transformation_mat_egoFrameCamBackRight_to_camera_cv2*transformation_mat_global_to__egoFrameCamBackRight_cv2*transformation_mat_egoFrame1_to_global_cv2*transformation_mat_lidar_to_egoFrame1_cv2*X;

            cv::Point pt_front;
            pt_front.x = Y_front.at<double>(0, 0) / Y_front.at<double>(0, 2);
            pt_front.y = Y_front.at<double>(1, 0) / Y_front.at<double>(0, 2);


            cv::Point pt_frontRight;
            pt_frontRight.x = Y_frontRight.at<double>(0, 0) / Y_frontRight.at<double>(0, 2);
            pt_frontRight.y = Y_frontRight.at<double>(1, 0) / Y_frontRight.at<double>(0, 2);


            cv::Point pt_frontLeft;
            pt_frontLeft.x = Y_frontLeft.at<double>(0, 0) / Y_frontLeft.at<double>(0, 2);
            pt_frontLeft.y = Y_frontLeft.at<double>(1, 0) / Y_frontLeft.at<double>(0, 2);


            cv::Point pt_back;
            pt_back.x = Y_back.at<double>(0, 0) / Y_back.at<double>(0, 2);
            pt_back.y = Y_back.at<double>(1, 0) / Y_back.at<double>(0, 2);


            cv::Point pt_backLeft;
            pt_backLeft.x = Y_backLeft.at<double>(0, 0) / Y_backLeft.at<double>(0, 2);
            pt_backLeft.y = Y_backLeft.at<double>(1, 0) / Y_backLeft.at<double>(0, 2);

            cv::Point pt_backRight;
            pt_backRight.x = Y_backRight.at<double>(0, 0) / Y_backRight.at<double>(0, 2);
            pt_backRight.y = Y_backRight.at<double>(1, 0) / Y_backRight.at<double>(0, 2);


            pcl::PointXYZRGB p;
            p.x = cloud->at(i).x;
            p.y = cloud->at(i).y;
            p.z = cloud->at(i).z;
            p.r = 0;
            p.g = 0;
            p.b = 0;

            if(Y_front.at<double>(0, 2) > 0) //remove points behind camera front
            {

                if(pt_front.x >=0 && pt_front.y >=0 && pt_front.x<imgFrontWidth && pt_front.y<imgFrontHeight) //keep only points within camera image plane
                {
                    cv::Vec3b color = imgFront.at<cv::Vec3b>(pt_front.y, pt_front.x);

                    p.b = color.val[0];
                    p.g  = color.val[1];
                    p.r  = color.val[2];
                }

            }

            if(Y_frontRight.at<double>(0, 2) > 0) //remove points behind camera front right
            {

                if(pt_frontRight.x >=0 && pt_frontRight.y >=0 && pt_frontRight.x< imgFrontRightWidth && pt_frontRight.y< imgFrontRightHeight) //keep only points within camera image plane
                {
                    cv::Vec3b color = imgFrontRight.at<cv::Vec3b>(pt_frontRight.y, pt_frontRight.x);
                    p.b = color.val[0];
                    p.g  = color.val[1];
                    p.r  = color.val[2];
                }

            }

            if(Y_frontLeft.at<double>(0, 2) > 0) //remove points behind camera front left
            {

                if(pt_frontLeft.x >=0 && pt_frontLeft.y >=0 && pt_frontLeft.x< imgFrontLeftWidth && pt_frontLeft.y< imgFrontLeftHeight) //keep only points within camera image plane
                {
                    cv::Vec3b color = imgFrontLeft.at<cv::Vec3b>(pt_frontLeft.y, pt_frontLeft.x);
                    p.b = color.val[0];
                    p.g  = color.val[1];
                    p.r  = color.val[2];
                }

            }


            if(Y_back.at<double>(0, 2) > 0) //remove points behind camera back
            {

                if(pt_back.x >=0 && pt_back.y >=0 && pt_back.x< imgBackWidth && pt_back.y< imgBackHeight) //keep only points within camera image plane
                {
                    cv::Vec3b color = imgBack.at<cv::Vec3b>(pt_back.y, pt_back.x);
                    p.b = color.val[0];
                    p.g  = color.val[1];
                    p.r  = color.val[2];
                }

            }

            if(Y_backLeft.at<double>(0, 2) > 0) //remove points behind camera back left
            {

                if(pt_backLeft.x >=0 && pt_backLeft.y >=0 && pt_backLeft.x< imgBackLeftWidth && pt_backLeft.y< imgBackLeftHeight) //keep only points within camera image plane
                {
                    cv::Vec3b color = imgBackLeft.at<cv::Vec3b>(pt_backLeft.y, pt_backLeft.x);
                    p.b = color.val[0];
                    p.g  = color.val[1];
                    p.r  = color.val[2];
                }

            }


            if(Y_backRight.at<double>(0, 2) > 0) //remove points behind camera back right
            {

                if(pt_backRight.x >=0 && pt_backRight.y >=0 && pt_backRight.x< imgBackRightWidth && pt_backRight.y< imgBackRightHeight) //keep only points within camera image plane
                {
                    cv::Vec3b color = imgBackRight.at<cv::Vec3b>(pt_backRight.y, pt_backRight.x);
                    p.b = color.val[0];
                    p.g  = color.val[1];
                    p.r  = color.val[2];
                }

            }

            cloudRgb->points.push_back(p);
            
            if(Y_front.at<double>(0, 2) > 0) //remove points behind camera
            {

                if(pt_front.x >=0 && pt_front.y >=0 && pt_front.x<imgFrontWidth && pt_front.y<imgFrontHeight)  //keep only points within camera image plane
                {
                    cv::Vec3b color = imgFront.at<cv::Vec3b>(pt_front.y, pt_front.x);
                    pcl::PointXYZ minPt, maxPt;
                    pcl::getMinMax3D (*cloud, minPt, maxPt);
                    float val = cloudRgb->at(i).y;
                    float maxVal = maxPt.y;
                    int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
                    int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
                    cv::circle(overlayFront, pt_front, 4, cv::Scalar(0, green, red), -1);
                }

            }


            if(Y_frontRight.at<double>(0, 2) > 0) //remove points behind camera
            {

                if(pt_frontRight.x >=0 && pt_frontRight.y >=0 && pt_frontRight.x<imgFrontRightWidth && pt_frontRight.y<imgFrontRightHeight)  //keep only points within camera image plane
                {
                    cv::Vec3b color = imgFrontRight.at<cv::Vec3b>(pt_frontRight.y, pt_frontRight.x);
                    pcl::PointXYZ minPt, maxPt;
                    pcl::getMinMax3D (*cloud, minPt, maxPt);
                    float val = cloudRgb->at(i).y;
                    float maxVal = maxPt.y;
                    int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
                    int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
                    cv::circle(overlayFrontRight, pt_frontRight, 4, cv::Scalar(0, green, red), -1);
                }

            }


            if(Y_frontLeft.at<double>(0, 2) > 0) //remove points behind camera
            {

                if(pt_frontLeft.x >=0 && pt_frontLeft.y >=0 && pt_frontLeft.x<imgFrontLeftWidth && pt_frontLeft.y<imgFrontLeftHeight)  //keep only points within camera image plane
                {
                    cv::Vec3b color = imgFrontLeft.at<cv::Vec3b>(pt_frontLeft.y, pt_frontLeft.x);
                    pcl::PointXYZ minPt, maxPt;
                    pcl::getMinMax3D (*cloud, minPt, maxPt);
                    float val = cloudRgb->at(i).y;
                    float maxVal = maxPt.y;
                    int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
                    int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
                    cv::circle(overlayFrontLeft, pt_frontLeft, 4, cv::Scalar(0, green, red), -1);
                }

            }


            if(Y_back.at<double>(0, 2) > 0) //remove points behind camera
            {

                if(pt_back.x >=0 && pt_back.y >=0 && pt_back.x<imgBackWidth && pt_back.y<imgBackHeight)  //keep only points within camera image plane
                {
                    cv::Vec3b color = imgBack.at<cv::Vec3b>(pt_back.y, pt_back.x);
                    pcl::PointXYZ minPt, maxPt;
                    pcl::getMinMax3D (*cloud, minPt, maxPt);
                    float val = cloudRgb->at(i).y;
                    float maxVal = maxPt.y;
                    int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
                    int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
                    cv::circle(overlayBack, pt_back, 4, cv::Scalar(0, green, red), -1);
                }

            }

            if(Y_backLeft.at<double>(0, 2) > 0) //remove points behind camera
            {

                if(pt_backLeft.x >=0 && pt_backLeft.y >=0 && pt_backLeft.x<imgBackLeftWidth && pt_backLeft.y<imgBackLeftHeight)  //keep only points within camera image plane
                {
                    cv::Vec3b color = imgBackLeft.at<cv::Vec3b>(pt_backLeft.y, pt_backLeft.x);
                    pcl::PointXYZ minPt, maxPt;
                    pcl::getMinMax3D (*cloud, minPt, maxPt);
                    float val = cloudRgb->at(i).y;
                    float maxVal = maxPt.y;
                    int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
                    int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
                    cv::circle(overlayBackLeft, pt_backLeft , 4, cv::Scalar(0, green, red), -1);
                }

            }

            if(Y_backRight.at<double>(0, 2) > 0) //remove points behind camera
            {

                if(pt_backRight.x >=0 && pt_backRight.y >=0 && pt_backRight.x<imgBackRightWidth && pt_backRight.y<imgBackRightHeight)  //keep only points within camera image plane
                {
                    cv::Vec3b color = imgBackRight.at<cv::Vec3b>(pt_backRight.y, pt_backRight.x);
                    pcl::PointXYZ minPt, maxPt;
                    pcl::getMinMax3D (*cloud, minPt, maxPt);
                    float val = cloudRgb->at(i).y;
                    float maxVal = maxPt.y;
                    int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
                    int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
                    cv::circle(overlayBackRight, pt_backRight, 4, cv::Scalar(0, green, red), -1);
                }

            }

    }

    float opacity = 0.7;
    cv::addWeighted(overlayFront, opacity, visimgFront, 1 - opacity, 0, visimgFront);
    cv::addWeighted(overlayFrontRight, opacity, visimgFrontRight, 1 - opacity, 0, visimgFrontRight);
    cv::addWeighted(overlayFrontLeft, opacity, visimgFrontLeft, 1 - opacity, 0, visimgFrontLeft);
    cv::addWeighted(overlayBack, opacity, visimgBack, 1 - opacity, 0, visimgBack);
    cv::addWeighted(overlayBackLeft, opacity, visimgBackLeft, 1 - opacity, 0, visimgBackLeft);
    cv::addWeighted(overlayBackRight, opacity, visimgBackRight, 1 - opacity, 0, visimgBackRight);

    std::vector<cv::Mat> imagesForward{visimgFrontLeft, visimgFront, visimgFrontRight};
    std::vector<cv::Mat> imagesBack{visimgBackLeft, visimgBack, visimgBackRight};

    cv::Mat horizontalStitchedFront;
    cv::Mat horizontalStitchedBack;
   
    cv::hconcat(imagesForward, horizontalStitchedFront);
    cv::hconcat(imagesBack, horizontalStitchedBack);

    std::string windowName1 = "LiDAR data on image overlayFront Sitched";
    std::string windowName2 = "LiDAR data on image overlayBack Stitched";
    // cv::namedWindow(windowName1, cv::WINDOW_NORMAL);
    // cv::namedWindow(windowName2, cv::WINDOW_NORMAL);
    // cv::resizeWindow(windowName1, horizontalStitchedFront.size().width,  horizontalStitchedFront.size().height);
    // cv::resizeWindow(windowName2, horizontalStitchedBack.size().width,  horizontalStitchedBack.size().height);
    // cv::imshow(windowName1, horizontalStitchedFront);
    // cv::imshow(windowName2, horizontalStitchedBack);
    // cv::waitKey(0); // wait for key to be pressed

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	const std::string& cloudID = "Cloud View";

    int v1(0);
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	const std::string& cloudID1 = cloudID + std::to_string(1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, cloudID1, v1);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudID1, v1);
	viewer->setLookUpTableID(cloudID1);
	viewer->addText("Original Cloud", 0, 0, 25.0, 1.0, 1.0, 1.0, "v1 text", v1);
	viewer->createViewPortCamera(v1);
    viewer->addCoordinateSystem(1.0);


	int v2(0);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v2);
	const std::string& cloudID2 = cloudID + std::to_string(2);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloudRgb, cloudID2, v2);
	viewer->setLookUpTableID(cloudID2);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudID2, v2);
	viewer->addText("RGB Cloud", 0, 0, 25.0, 0.0, 1.0, 0.0, "v2 text", v2);
	viewer->createViewPortCamera(v2);
    viewer->addCoordinateSystem(1.0);
    

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(1000);
    }

    delete [] f;
    return 0;
}