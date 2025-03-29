#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <opencv2/core/eigen.hpp> // for cv::cv2eigen
//EIgen SVD libnary, may help you solve SVD
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

using namespace cv;
using namespace aruco;
using namespace Eigen;

//global varialbles for aruco detector
aruco::CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;
float MarkerSize = 0.20 / 1.5 * 1.524;
float MarkerWithMargin = MarkerSize * 1.2;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;
ros::Publisher pub_odom_yourwork;
ros::Publisher pub_odom_ref;
cv::Mat K, D;

// RMS error track
int frame_count = 0;
double R_rms_error_sum = 0.0;
double T_rms_error_sum = 0.0;

// test function, can be used to verify your estimation
void calculateReprojectionError(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const cv::Mat R, const cv::Mat t)
{
    puts("calculateReprojectionError begins");
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    for (unsigned int i = 0; i < pts_3.size(); i++)
    {
        cv::Mat p_mat(3, 1, CV_64FC1);
        p_mat.at<double>(0, 0) = pts_3[i].x;
        p_mat.at<double>(1, 0) = pts_3[i].y;
        p_mat.at<double>(2, 0) = pts_3[i].z;
        cv::Mat p = (R * p_mat + t);
        printf("(%f, %f, %f) -> (%f, %f) and (%f, %f)\n",
               pts_3[i].x, pts_3[i].y, pts_3[i].z,
               un_pts_2[i].x, un_pts_2[i].y,
               p.at<double>(0) / p.at<double>(2), p.at<double>(1) / p.at<double>(2));
    }
    puts("calculateReprojectionError ends");
}

// the main function you need to work with
// pts_id: id of each point
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(const vector<int> &pts_id, const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    //version 1, as reference
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);
    cv::Rodrigues(rvec, r);
    Matrix3d R_ref;
    Eigen::Vector3d T_ref; // store the reference rotation and translation for RMS error calculation
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            R_ref(i,j) = r.at<double>(i, j);
        }
        T_ref(i) = t.at<double>(i, 0);
    }
    Quaterniond Q_ref;
    Q_ref = R_ref;

    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp = frame_time;
    odom_ref.header.frame_id = "world";
    odom_ref.pose.pose.position.x = t.at<double>(0, 0);
    odom_ref.pose.pose.position.y = t.at<double>(1, 0);
    odom_ref.pose.pose.position.z = t.at<double>(2, 0);
    odom_ref.pose.pose.orientation.w = Q_ref.w();
    odom_ref.pose.pose.orientation.x = Q_ref.x();
    odom_ref.pose.pose.orientation.y = Q_ref.y();
    odom_ref.pose.pose.orientation.z = Q_ref.z();
    pub_odom_ref.publish(odom_ref);

    // version 2, your work
    Matrix3d R;
    Vector3d T;
    R.setIdentity();
    T.setZero();
    vector<cv::Point2f> un_pts_2; // store undistorted points in normalized cam coordinates
    // cv::undistortPoints(pts_2, un_pts_2, K, D); // undistort the points and store in un_pts_2 in normalized cam coordinates
    cv::undistortPoints(pts_2, un_pts_2, K, D,noArray(),K); // undistort the points and store in un_pts_2, and project back to the pixel coordinates, same as the pts_2's frame
    
    // ROS_INFO("write your code here!");

    /* we assume world frame z=0, so we can use 2D PnP */
    // check the size for correspondence
    int num_pts = pts_3.size();
    if (num_pts < 4)
    {
        ROS_WARN("Not enough points for PnP");
        return;
    }

    // build A
    Eigen::MatrixXd A(2 * num_pts, 9);
    for (int i = 0; i < num_pts; i++)
    {
        cv::Point3f p_3d = pts_3[i];
        cv::Point2f p_2d = un_pts_2[i];

        double x = p_3d.x, y = p_3d.y;
        double u = p_2d.x, v = p_2d.y;

        int row = 2 * i;
        
        A(row, 0) = x; A(row, 1) = y; A(row, 2) = 1.0;
        A(row, 3) = 0.0; A(row, 4) = 0.0; A(row, 5) = 0.0;
        A(row, 6) = -x * u; A(row, 7) = -y * u; A(row, 8) = -u;

        row++;

        A(row, 0) = 0.0; A(row, 1) = 0.0; A(row, 2) = 0.0; 
        A(row, 3) = x; A(row, 4) = y; A(row, 5) = 1.0;
        A(row, 6) = -x * v; A(row, 7) = -y * v; A(row, 8) = -v;
    }

    // SVD for Ax=0
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::VectorXd h = V.col(V.cols() - 1); // get the last column of V -> solution -> h = [r1, r2, t]

    // reshape into H
    Eigen::Matrix<double, 3, 3> H;
    for (int i = 0; i < 9; i++)
    {
        H(i / 3, i % 3) = h(i);
    }

    if (H(2, 2) < 0) // check consistently oriented (check whether camera is upside down)
    {
        ROS_INFO("Inconsistent orientation, flip the H");
        H *= -1;
    }

    // extract R and t from H
    Eigen::Matrix3d K_eigen_form;
    cv::cv2eigen(K, K_eigen_form);
    Eigen::MatrixXd homo_T_est = K_eigen_form.inverse() * H;
    // Eigen::MatrixXd homo_T_est = H;
    Eigen::Vector3d h1_est = homo_T_est.col(0);
    Eigen::Vector3d h2_est = homo_T_est.col(1);
    Eigen::Vector3d h3_est = h1_est.cross(h2_est);
    Eigen::Matrix3d R_est;
    R_est << h1_est, h2_est, h3_est;

    // SVD to find the best orthogonal matrix R
    JacobiSVD<Matrix3d> svd_R(R_est, ComputeFullU | ComputeFullV);
    Matrix3d U_R = svd_R.matrixU();
    Matrix3d V_R = svd_R.matrixV();
    R = U_R * V_R.transpose();

    // enforce det(R) = 1 (valid rotation)
    if (R.determinant() < 0)
    {
        ROS_INFO("Inconsistent rotation, flip the R");
        V_R.col(2) *= -1;
        R = U_R * V_R.transpose();
    }

    // translation
    T = homo_T_est.col(2) / homo_T_est.col(0).norm();
    if (T(2) < 0) // check consistency of translation (check whether camera is upside down)
    {
        ROS_INFO("Inconsistent translation, flip the T");
        T *= -1;
    }

    ////////////////////////////////
    // calculate the RMS error from the reference track
    frame_count++;
    Eigen::Matrix3d R_diff = R - R_ref;
    Eigen::Vector3d T_diff = T - T_ref;
    double R_rms_error = R_diff.norm(); // frobenius norm
    double T_rms_error = T_diff.norm();
    R_rms_error_sum += R_rms_error;
    T_rms_error_sum += T_rms_error;
    double R_rms_error_avg = R_rms_error_sum / frame_count;
    double T_rms_error_avg = T_rms_error_sum / frame_count;
    ROS_INFO_STREAM("\n" << "Frame: " << frame_count << ", R_rms_error_avg: " << R_rms_error_avg << ", T_rms_error_avg: " << T_rms_error_avg << "\n");
    ////////////////////////////////

    // convert to quaternion    
    Quaterniond Q_yourwork;
    Q_yourwork = R;
    // publish the result
    nav_msgs::Odometry odom_yourwork;
    odom_yourwork.header.stamp = frame_time;
    odom_yourwork.header.frame_id = "world";
    odom_yourwork.pose.pose.position.x = T(0);
    odom_yourwork.pose.pose.position.y = T(1);
    odom_yourwork.pose.pose.position.z = T(2);
    odom_yourwork.pose.pose.orientation.w = Q_yourwork.w();
    odom_yourwork.pose.pose.orientation.x = Q_yourwork.x();
    odom_yourwork.pose.pose.orientation.y = Q_yourwork.y();
    odom_yourwork.pose.pose.orientation.z = Q_yourwork.z();
    pub_odom_yourwork.publish(odom_yourwork);
}

cv::Point3f getPositionFromIndex(int idx, int nth)
{
    int idx_x = idx % 6, idx_y = idx / 6;
    double p_x = idx_x * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
    double p_y = idx_y * MarkerWithMargin - (12 + 11.5 * 0.2) * MarkerSize;
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 2 || nth == 3) * MarkerSize, 0.0);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double t = clock();
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    MDetector.detect(bridge_ptr->image, Markers);
    float probDetect = TheBoardDetector.detect(Markers, TheBoardConfig, TheBoardDetected, CamParam, MarkerSize);
    ROS_DEBUG("p: %f, time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC);

    vector<int> pts_id;
    vector<cv::Point3f> pts_3;
    vector<cv::Point2f> pts_2;
    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        int idx = TheBoardConfig.getIndexOfMarkerId(Markers[i].id);

        char str[100];
        sprintf(str, "%d", idx);
        cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        for (unsigned int j = 0; j < 4; j++)
        {
            sprintf(str, "%d", j);
            cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        }

        for (unsigned int j = 0; j < 4; j++)
        {
            pts_id.push_back(Markers[i].id * 4 + j);
            pts_3.push_back(getPositionFromIndex(idx, j));
            pts_2.push_back(Markers[i][j]);
        }
    }

    //begin your function
    if (pts_id.size() > 5)
        process(pts_id, pts_3, pts_2, img_msg->header.stamp);

    cv::imshow("in", bridge_ptr->image);
    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");

    ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);
    pub_odom_yourwork = n.advertise<nav_msgs::Odometry>("odom_yourwork",10);
    pub_odom_ref = n.advertise<nav_msgs::Odometry>("odom_ref",10);
    //init aruco detector
    string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    CamParam.readFromXMLFile(cam_cal);
    TheBoardConfig.readFromFile(board_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;

    //init window for visualization
    cv::namedWindow("in", 1);

    ros::spin();
}
