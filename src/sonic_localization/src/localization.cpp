
// 从testodom改
// 初始化：
// 1. 读到get_map发来的地图，surfmap和edgemap
    // 1.1 读取候选帧
    // 1.2 提取候选帧特征
    // 1.3 进入循环
// 2. 等一个手动输出的信号 
// 3. 将地图切块
// 4. 提取surf和edge
// 5. 使用icp进行一个迭代找最优解，surf和edge
// 6. 输出相对于map的位姿

// 初始化结束以后：
// 2. 初始使用积累的轨迹
// 别的都一样

//test for feature extract
#include <vector> 
#include <string> 
#include <fstream> 
#include <iostream> 
#include <queue>
#include <thread>
#include <mutex>

#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/pca.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <ros/ros.h> 
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h> 
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include "common.h"
#include "lidarOptimization.h"
#include  "tictoc.h"
#include <tf/transform_broadcaster.h>

typedef pcl::PointXYZI PointType;

std::queue<sensor_msgs::PointCloud2ConstPtr> currcloudBuf;
ros::Publisher pubtfcloud;
ros::Publisher puborigincloud;
ros::Publisher pubsurfmap;
ros::Publisher pubsegsurfmap;
ros::Publisher pubsegedgemap;
ros::Publisher publocal_odom;

pcl::PointCloud<PointType>::Ptr  surface_map (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr  edge_map (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr  seg_surface_map (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr  seg_edge_map (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr  curr_cloud (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr  curr_surf_cloud (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr  curr_edge_cloud (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr  tf_cloud (new pcl::PointCloud<PointType>);
std::vector<pcl::PointCloud<PointType>::Ptr> mapfeaturepackage;
std::vector<pcl::PointCloud<PointType>::Ptr> new_mapfeaturepackage;
std::vector<float> linear_list;
std::vector<float> planar_list;
std::vector<float> curvature_list;
std::mutex mBuf;
int frame_num = -1;
const int DISTORTION = 0;
constexpr double SCAN_PERIOD = 0.1;
double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;
int N_SCAN = 0;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};
double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
double update_t[3] = {0, 0, 0};
//  double init_q[4] = {0.045, -0.01, 0.55, 0.82};
// double init_t[3] = {29.9, 34.1, -0.4};
double init_q[4] = {0, 0, 0, 1};
double init_t[3] = {0, 0, 0};

Eigen::Isometry3d odom;
Eigen::Isometry3d last_odom;
// nav_msgs::Odometry


int optimization_count = 10;
int USE_INIT_ODOM = 1;

Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters+4);


bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }
void para_reset(float init_x, float init_y, float init_z, float init_rw, float init_rx, float init_ry, float init_rz){
    parameters[0] = init_rx;
    parameters[1] = init_ry;
    parameters[2] = init_rz;
    parameters[3] = init_rw;
    parameters[4] = init_x;
    parameters[5] = init_y;
    parameters[6] = init_z;
}
void update_init(float init_x, float init_y, float init_z, float init_rw, float init_rx, float init_ry, float init_rz){
    init_q[0] = init_rx;
    init_q[1] = init_ry;
    init_q[2] = init_rz;
    init_q[3] = init_rw;
    init_t[0] = init_x;
    init_t[1] = init_y;
    init_t[2] = init_z;
}
pcl::PointCloud<PointType>::Ptr pointcloudplus(pcl::PointCloud<PointType>::Ptr src1, pcl::PointCloud<PointType>::Ptr src2){
	pcl::PointCloud<PointType>::Ptr outcloud(new pcl::PointCloud<PointType>);
	for(int i = 0;i < src1->points.size();i++)
		outcloud->push_back(src1->points[i]);
	for(int i = 0;i < src2->points.size();i++)
		outcloud->push_back(src2->points[i]);
	return outcloud;
}
void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloudIn)
{
    mBuf.lock();
    currcloudBuf.push(cloudIn);
    mBuf.unlock();
}
pcl::PointCloud<PointType>::Ptr  TransformCloud(pcl::PointCloud<PointType>::Ptr cloudin,    Eigen::Quaterniond q,  Eigen::Vector3d  t)
{
    pcl::PointCloud<PointType>::Ptr cloudout(new pcl::PointCloud<PointType> );
    PointType tmppoint;
    for(int i = 0;i < cloudin->points.size(); i++){
        Eigen::Vector3d point(cloudin->points[i].x, cloudin->points[i].y, cloudin->points[i].z);
        Eigen::Vector3d un_point =q * point +t;
        
        tmppoint.x = un_point.x();
        tmppoint.y = un_point.y();
        tmppoint.z = un_point.z();
        tmppoint.intensity = cloudin->points[i].intensity;
        cloudout->points.push_back(tmppoint);
    }
    return cloudout;
}

class Double2d{
public:
	int id;
	double value;
    Double2d(int id_in, double value_in){
        id = id_in;
        value =value_in;
    };
};
//points info class
class PointsInfo{
public:
	int layer;
	double time;
    PointsInfo(int layer_in, double time_in){
        layer = layer_in;
        time = time_in;
    };
};

void downSamplingToMap(const pcl::PointCloud<PointType>::Ptr& edge_pc_in, pcl::PointCloud<PointType>::Ptr& edge_pc_out, const pcl::PointCloud<PointType>::Ptr& surf_pc_in, pcl::PointCloud<PointType>::Ptr& surf_pc_out){
    pcl::VoxelGrid<PointType> downSizeFilterEdge;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    downSizeFilterEdge.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);    
}
void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}


void featureExtractionFromSector(const pcl::PointCloud<PointType>::Ptr& pc_in, std::vector<Double2d>& cloudCurvature, pcl::PointCloud<PointType>::Ptr& pc_out_edge, pcl::PointCloud<PointType>::Ptr& pc_out_surf){

    std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const Double2d & a, const Double2d & b)
    { 
        return a.value < b.value; 
    });


    int largestPickedNum = 0;
    std::vector<int> picked_points;
    int point_info_count =0;
    for (int i = cloudCurvature.size()-1; i >= 0; i--)
    {
        int ind = cloudCurvature[i].id; 
        if(std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
            if(cloudCurvature[i].value <= 0.1){
                break;
            }
            
            largestPickedNum++;
            picked_points.push_back(ind);
            
            if (largestPickedNum <= 20){
                pc_out_edge->push_back(pc_in->points[ind]);
                point_info_count++;
            }else{
                break;
            }

            for(int k=1;k<=5;k++){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k);
            }
            for(int k=-1;k>=-5;k--){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k);
            }

        }
    }

    for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
    {
        int ind = cloudCurvature[i].id; 
        if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end())
        {
            pc_out_surf->push_back(pc_in->points[ind]);
        }
    }
    


}

void featureExtraction(const pcl::PointCloud<PointType>::Ptr& pc_in, pcl::PointCloud<PointType>::Ptr& pc_out_edge, pcl::PointCloud<PointType>::Ptr& pc_out_surf){

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_in, indices);


    int N_SCANS = 32;
    std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudScans;
    for(int i=0;i<N_SCANS;i++){
        laserCloudScans.push_back(pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>()));
    }

    for (int i = 0; i < (int) pc_in->points.size(); i++)
    {
        int scanID=0;
        double distance = sqrt(pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y);
        if(distance < 0.5 || distance > 60)
            continue;
        double angle = atan(pc_in->points[i].z / distance) * 180 / M_PI;
        
        if (N_SCANS == 16){
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 32){
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 64){   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            if (angle > 2 || angle < -24.33 || scanID > 63 || scanID < 0)
            {
                continue;
            }
        }
        else{
            printf("wrong scan number\n");
        }
        laserCloudScans[scanID]->push_back(pc_in->points[i]); 

    }

    for(int i = 0; i < N_SCANS; i++){
        if(laserCloudScans[i]->points.size()<131){
            continue;
        }
        
        std::vector<Double2d> cloudCurvature; 
        int total_points = laserCloudScans[i]->points.size()-10;
        for(int j = 5; j < (int)laserCloudScans[i]->points.size() - 5; j++){
            double diffX = laserCloudScans[i]->points[j - 5].x + laserCloudScans[i]->points[j - 4].x + laserCloudScans[i]->points[j - 3].x + laserCloudScans[i]->points[j - 2].x + laserCloudScans[i]->points[j - 1].x - 10 * laserCloudScans[i]->points[j].x + laserCloudScans[i]->points[j + 1].x + laserCloudScans[i]->points[j + 2].x + laserCloudScans[i]->points[j + 3].x + laserCloudScans[i]->points[j + 4].x + laserCloudScans[i]->points[j + 5].x;
            double diffY = laserCloudScans[i]->points[j - 5].y + laserCloudScans[i]->points[j - 4].y + laserCloudScans[i]->points[j - 3].y + laserCloudScans[i]->points[j - 2].y + laserCloudScans[i]->points[j - 1].y - 10 * laserCloudScans[i]->points[j].y + laserCloudScans[i]->points[j + 1].y + laserCloudScans[i]->points[j + 2].y + laserCloudScans[i]->points[j + 3].y + laserCloudScans[i]->points[j + 4].y + laserCloudScans[i]->points[j + 5].y;
            double diffZ = laserCloudScans[i]->points[j - 5].z + laserCloudScans[i]->points[j - 4].z + laserCloudScans[i]->points[j - 3].z + laserCloudScans[i]->points[j - 2].z + laserCloudScans[i]->points[j - 1].z - 10 * laserCloudScans[i]->points[j].z + laserCloudScans[i]->points[j + 1].z + laserCloudScans[i]->points[j + 2].z + laserCloudScans[i]->points[j + 3].z + laserCloudScans[i]->points[j + 4].z + laserCloudScans[i]->points[j + 5].z;
            Double2d distance(j,diffX * diffX + diffY * diffY + diffZ * diffZ);
            cloudCurvature.push_back(distance);

        }
        for(int j=0;j<6;j++){
            int sector_length = (int)(total_points/6);
            int sector_start = sector_length *j;
            int sector_end = sector_length *(j+1)-1;
            if (j==5){
                sector_end = total_points - 1; 
            }
            std::vector<Double2d> subCloudCurvature(cloudCurvature.begin()+sector_start,cloudCurvature.begin()+sector_end); 
            
            featureExtractionFromSector(laserCloudScans[i],subCloudCurvature, pc_out_edge, pc_out_surf);
            
        }

    }

}

void odometry(const pcl::PointCloud<PointType>::Ptr& edge_map, const pcl::PointCloud<PointType>::Ptr& surf_map, 
              const pcl::PointCloud<PointType>::Ptr& curr_edge_cloud, const pcl::PointCloud<PointType>::Ptr& curr_surf_cloud){

    int opti_counter = 2;
    getParameter("opti_counter", opti_counter);
    if(optimization_count>opti_counter)
        optimization_count--;

    if(USE_INIT_ODOM){
        Eigen::Isometry3d init_odom = Eigen::Isometry3d::Identity();
        Eigen::Quaterniond tmp_q(init_q[3], init_q[0], init_q[1], init_q[2]);
        Eigen::Matrix3d rotation_matrix =  tmp_q.matrix();
        Eigen::Vector3d tmp_t(init_t[0], init_t[1], init_t[2]);
        init_odom.rotate(rotation_matrix);
        init_odom.pretranslate(tmp_t);
        odom = init_odom;
        last_odom = odom;
        USE_INIT_ODOM = 0;
    }
    else{
        Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
        last_odom = odom;
        odom = odom_prediction;
    }

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();

    pcl::PointCloud<PointType>::Ptr downsampledEdgeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr downsampledSurfCloud(new pcl::PointCloud<PointType>());
    downSamplingToMap(curr_edge_cloud, downsampledEdgeCloud, curr_surf_cloud, downsampledSurfCloud);
    //ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
    if(edge_map->points.size()>10 && surf_map->points.size()>50){
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeEdgeMap;
        kdtreeEdgeMap = pcl::KdTreeFLANN<PointType>::Ptr(new pcl::KdTreeFLANN<PointType>());;
		pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfMap;
        kdtreeSurfMap = pcl::KdTreeFLANN<PointType>::Ptr(new pcl::KdTreeFLANN<PointType>());;
        kdtreeEdgeMap->setInputCloud(edge_map);
        kdtreeSurfMap->setInputCloud(surf_map);

        for (int iterCount = 0; iterCount < optimization_count; iterCount++){
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
            
            // addEdgeCostFactor;
            /////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////
            int corner_num=0;
            for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
            {
                pcl::PointXYZI point_temp;
                pointAssociateToMap(&(downsampledEdgeCloud->points[i]), &point_temp);

                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;
                kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis); 
                if (pointSearchSqDis[4] < 1.0)
                {
                    std::vector<Eigen::Vector3d> nearCorners;
                    Eigen::Vector3d center(0, 0, 0);
                    for (int j = 0; j < 5; j++)
                    {
                        Eigen::Vector3d tmp(edge_map->points[pointSearchInd[j]].x,
                                            edge_map->points[pointSearchInd[j]].y,
                                            edge_map->points[pointSearchInd[j]].z);
                        center = center + tmp;
                        nearCorners.push_back(tmp);
                    }
                    center = center / 5.0;

                    Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                    for (int j = 0; j < 5; j++)
                    {
                        Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                        covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                    }

                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                    Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                    Eigen::Vector3d curr_point(downsampledEdgeCloud->points[i].x, downsampledEdgeCloud->points[i].y, downsampledEdgeCloud->points[i].z);
                    if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                    { 
                        Eigen::Vector3d point_on_line = center;
                        Eigen::Vector3d point_a, point_b;
                        point_a = 0.1 * unit_direction + point_on_line;
                        point_b = -0.1 * unit_direction + point_on_line;

                        ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);  
                        problem.AddResidualBlock(cost_function, loss_function, parameters);
                        corner_num++;   
                    }                           
                }
            }
            if(corner_num<20){
                printf("not enough correct points\n");
            }
            //////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////
            // addSurfCostFactor;
            //////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////
            
            int surf_num=0;
            for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
            {
                pcl::PointXYZI point_temp;
                pointAssociateToMap(&(downsampledSurfCloud->points[i]), &point_temp);
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;
                kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

                Eigen::Matrix<double, 5, 3> matA0;
                Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                if (pointSearchSqDis[4] < 1.0)
                {
                    
                    for (int j = 0; j < 5; j++)
                    {
                        matA0(j, 0) = surf_map->points[pointSearchInd[j]].x;
                        matA0(j, 1) = surf_map->points[pointSearchInd[j]].y;
                        matA0(j, 2) = surf_map->points[pointSearchInd[j]].z;
                    }
                    // find the norm of plane
                    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                    double negative_OA_dot_norm = 1 / norm.norm();
                    norm.normalize();

                    bool planeValid = true;
                    for (int j = 0; j < 5; j++)
                    {
                        // if OX * n > 0.2, then plane is not fit well
                        if (fabs(norm(0) * surf_map->points[pointSearchInd[j]].x +
                                norm(1) * surf_map->points[pointSearchInd[j]].y +
                                norm(2) * surf_map->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                        {
                            planeValid = false;
                            break;
                        }
                    }
                    Eigen::Vector3d curr_point(downsampledSurfCloud->points[i].x, downsampledSurfCloud->points[i].y, downsampledSurfCloud->points[i].z);
                    if (planeValid)
                    {
                        ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);    
                        problem.AddResidualBlock(cost_function, loss_function, parameters);

                        surf_num++;
                    }
                }

            }
            if(surf_num<20){
                printf("not enough correct points");
            }
            
            
            //////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////

            // feature_distribution_file<<"edge:\n";
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);

        }
    }else{
        printf("not enough points in map to associate, map error");
    }
    odom = Eigen::Isometry3d::Identity();
    odom.linear() = q_w_curr.toRotationMatrix();
    odom.translation() = t_w_curr;

}




int main(int argc, char **argv)
{
    ros::init (argc, argv, "localization_gaoxin"); 
    ros::NodeHandle nh; 
    std::string ws_path;


    ros::Subscriber subfirstcloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 100, cloudHandler);
   // ros::Subscriber subinitialpose = nh.subscribe("/initialpose", 1000, initHandler);//rviz
    pubtfcloud = nh.advertise<sensor_msgs::PointCloud2> ("tf_cloud", 1);
    puborigincloud = nh.advertise<sensor_msgs::PointCloud2> ("origin_cloud", 1);
    pubsegsurfmap = nh.advertise<sensor_msgs::PointCloud2> ("seg_surface_map", 1);
    pubsegedgemap = nh.advertise<sensor_msgs::PointCloud2> ("seg_edge_map", 1);
    pubsurfmap = nh.advertise<sensor_msgs::PointCloud2> ("surface_map", 1);
    publocal_odom = nh.advertise<nav_msgs::Odometry>("/local_odom", 100);
    nh.getParam("/ws_path", ws_path);
    

    sensor_msgs::PointCloud2 cloudtempmsg;
    
    ros::Duration(1).sleep();//延时

    pcl::io::loadPCDFile<PointType>(ws_path+"/data/map/surfaceMap_pre.pcd", *edge_map);
    pcl::io::loadPCDFile<PointType>(ws_path+"/data/map/edgeMap_pre.pcd", *surface_map);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*edge_map, *edge_map, indices);
    std::vector<int> indices1;
    pcl::removeNaNFromPointCloud(*surface_map, *surface_map, indices1);
    
   

    
    float init_x, init_y, init_z, init_rw, init_rx, init_ry, init_rz;
    getParameter("/translation/x", init_x);
    getParameter("/translation/y", init_y);
    getParameter("/translation/z", init_z);
    getParameter("/rotation/x", init_rx);
    getParameter("/rotation/y", init_ry);
    getParameter("/rotation/z", init_rz);
    getParameter("/rotation/w", init_rw);
    update_init(init_x, init_y, init_z, init_rw, init_rx, init_ry, init_rz);

    seg_surface_map->clear();
    seg_edge_map->clear();
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud (surface_map);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (init_t[0] - 200, init_t[0] + 200);
    pass.setNegative (false);
    pass.filter (*seg_surface_map);
    pass.setInputCloud (seg_surface_map);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (init_t[1] - 200, init_t[1] + 200);
    pass.setNegative (false);
    pass.filter (*seg_surface_map);

    pass.setInputCloud (edge_map);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (init_t[0] - 200, init_t[0] + 200);
    pass.setNegative (false);
    pass.filter (*seg_edge_map);
    pass.setInputCloud (seg_edge_map);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (init_t[1] - 200, init_t[1] + 200);
    pass.setNegative (false);
    pass.filter (*seg_edge_map);

    


    
    

    // std::thread offer_map_process{offer_map};
    std::cout<<"------------start localization-----------------"<<std::endl;
    while (ros::ok())
    {
        ros::spinOnce();
        // ros::Duration(0.1).sleep();//延时
        
        if (!currcloudBuf.empty()){
            //std::cout<<"start>>>>>>>"<<std::endl;
            frame_num += 1;
            mBuf.lock();
            ros::Time timestamp = currcloudBuf.front()->header.stamp;
            pcl::fromROSMsg(*currcloudBuf.front(), *curr_cloud);
            currcloudBuf.pop();
            mBuf.unlock();



            TicToc  t_whole;
            TicToc t_feature;
            std::cout<<"frame cloud size:"<< curr_cloud->points.size()<<std::endl;
            curr_edge_cloud->clear();
            curr_surf_cloud->clear();
            featureExtraction(curr_cloud, curr_edge_cloud, curr_surf_cloud);
            std::cout<<"feature extraction time: "<<t_feature.toc()<<std::endl;
            
            double distance_for_update_map = (parameters[4] - update_t[0])*(parameters[4] - update_t[0]) + (parameters[5] - update_t[1])*(parameters[5] - update_t[1]) 
                                            + (parameters[6] - update_t[2]) * (parameters[6] - update_t[2]);
            // std::cout<<"#########"<<parameters[4]<<" "<<update_t[0]<<" "<<parameters[5]<<" "<<update_t[1]<<" "<<parameters[6]<<" "<<update_t[2]<<std::endl;
            if (distance_for_update_map > 25){
                pcl::PassThrough<PointType> pass;
                pass.setInputCloud (surface_map);
                pass.setFilterFieldName ("x");
                pass.setFilterLimits (parameters[4] - 70, parameters[4] + 70);
                pass.setNegative (false);
                pass.filter (*seg_surface_map);
                pass.setInputCloud (seg_surface_map);
                pass.setFilterFieldName ("y");
                pass.setFilterLimits (parameters[5] - 70, parameters[5] + 70);
                pass.setNegative (false);
                pass.filter (*seg_surface_map);

                pass.setInputCloud (edge_map);
                pass.setFilterFieldName ("x");
                pass.setFilterLimits (parameters[4] - 70, parameters[4] + 70);
                pass.setNegative (false);
                pass.filter (*seg_edge_map);
                pass.setInputCloud (seg_edge_map);
                pass.setFilterFieldName ("y");
                pass.setFilterLimits (parameters[5] - 70, parameters[5] + 70);
                pass.setNegative (false);
                pass.filter (*seg_edge_map);
                update_t[0] = parameters[4];
                update_t[1] = parameters[5];
                update_t[2] = parameters[6];
            }

            TicToc t_odom;
            std::cout<<"map size: "<<seg_edge_map->points.size()<<", "<<seg_surface_map->points.size()
            <<", curr feature size: "<<curr_edge_cloud->points.size()<<", "<<curr_surf_cloud->points.size()<<std::endl;
            odometry(seg_edge_map, seg_surface_map, curr_edge_cloud, curr_surf_cloud);
            std::cout<<"t_odom:"<<t_odom.toc()<<std::endl;

            update_init(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5], parameters[6]);
            std::cout<<"t: "<<parameters[4]<<" "<<parameters[5]<<" "<<parameters[6]<<std::endl;
            std::cout<<"q: "<<parameters[0]<<" "<< parameters[1]<<" "<< parameters[2]<<" "<<parameters[3]<<std::endl;

            Eigen::Quaterniond q(parameters[3], parameters[0], parameters[1], parameters[2]);
            Eigen::Vector3d  t(parameters[4], parameters[5], parameters[6]);
            TicToc t_tfcloud;
            tf_cloud = TransformCloud(curr_cloud , q,t);
            std::cout<<"t_tfcloud:"<<t_tfcloud.toc()<<std::endl;

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(t.x(), t.y(), t.z()) );
            tf::Quaternion q_tf(q.x(),q.y(),q.z(),q.w());
            transform.setRotation(q_tf);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
            

            std::cout<<"all time:"<<t_whole.toc()<<std::endl<<std::endl;

            pcl::toROSMsg(*curr_cloud, cloudtempmsg);
            cloudtempmsg.header.frame_id = "map";
            puborigincloud.publish(cloudtempmsg);

             pcl::toROSMsg(*surface_map, cloudtempmsg);
            cloudtempmsg.header.frame_id = "map";
            pubsurfmap.publish(cloudtempmsg);

            pcl::toROSMsg(*tf_cloud, cloudtempmsg);
            cloudtempmsg.header.frame_id = "map";
            pubtfcloud.publish(cloudtempmsg);

            pcl::toROSMsg(*seg_surface_map, cloudtempmsg);
            cloudtempmsg.header.frame_id = "map";
            pubsegsurfmap.publish(cloudtempmsg);

            pcl::toROSMsg(*seg_edge_map, cloudtempmsg);
            cloudtempmsg.header.frame_id = "map";
            pubsegedgemap.publish(cloudtempmsg);

            nav_msgs::Odometry local_odom;
            local_odom.header.stamp = timestamp;
            local_odom.pose.pose.position.x = parameters[4];
            local_odom.pose.pose.position.y = parameters[5];
            local_odom.pose.pose.position.z = parameters[6];
            local_odom.pose.pose.orientation.x = parameters[0];
            local_odom.pose.pose.orientation.y = parameters[1];
            local_odom.pose.pose.orientation.z = parameters[2];
            local_odom.pose.pose.orientation.w = parameters[3];
            local_odom.header.frame_id = "/map";
            publocal_odom.publish(local_odom);
        }  
    }
    return 0; 
}

