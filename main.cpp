#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/auto_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include "Model_PreProsessing.h"
#include "Database.h"
#include "example.hpp"
#include <AndreiUtils/classes/RandomNumberGenerator.hpp>
#include "Aux.cpp"


#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>


// Standard library
#include <iostream>
#include <vector>
#include <thread>
#include <ctime>
#include <string>
#include <chrono>
#include <cmath>



// Camera
#include <librealsense2/rs.hpp>


using namespace std::chrono_literals;


const int M = 25000;           // I don't know how to select this number
const float K = 0.1;           // default
const float C = 0.25;          // default
float OffLineRadius {0.005};  // I need to tune this later (0.0025, 0.005)

float tolerance {OffLineRadius*10/100}; //20% of radius

// Online
const float Ps = 0.98;                    // probability of success
const float Octree_resolution = 0.001;         // I chose this number to keep 50% of the original scene

// camera
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;


pcl_ptr points_to_pcl(const rs2::points& points);


int main() {

    OuterMap generatedMap = DB::create_hashMap(OffLineRadius,tolerance, Octree_resolution ,"../YCB_ply/TWO");
    //DB::to_serialize_hashMap(generatedMap, "Adel.bin");
    //std::string filename = "Adel.bin";
   // OuterMap deserialized_map = DB::to_deserialize_hashMap(filename);



    pcl::PointCloud<pcl::PointXYZ>::Ptr Model_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr BOX_Model_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr BOX_Model_cloud_noise_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr Norm_of_Scene_cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);


    Model_cloud->clear();

    window app(1280, 720, "RealSense Pointcloud Example");
    glfw_state app_state;
    register_glfw_callbacks(app, app_state);

    rs2::pointcloud pc;
    rs2::points points;
    rs2::pipeline pipe;
    pipe.start();

    pcl::PointCloud<pcl::PointXYZ>::Ptr Scene_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    while (app) // Application still alive?
    {
        auto frames = pipe.wait_for_frames();
        auto color = frames.get_color_frame();
        if (!color)
            color = frames.get_infrared_frame();
        pc.map_to(color);

        auto depth = frames.get_depth_frame();
        points = pc.calculate(depth);
        Scene_cloud = points_to_pcl(points);


        // ***** OCtree Downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr Scene_cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);


        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(Octree_resolution);
        octree.setInputCloud(Scene_cloud);
        octree.addPointsFromInputCloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<int> Indices_of_points_in_one_voxel;
        auto octree_it = octree.begin();
        while (octree_it != octree.end()) {
            temp_cloud->clear();
            if (octree_it.isLeafNode()) {
                Indices_of_points_in_one_voxel = (octree_it.getLeafContainer().getPointIndicesVector());
                for (auto i: Indices_of_points_in_one_voxel) {
                    temp_cloud->push_back(Scene_cloud->points[i]);
                }
                Scene_cloud_downsampled->push_back(computeCentroidOfVoxel(*temp_cloud));

            }
            ++octree_it;

        }
        cout << "size after downsampling : " << Scene_cloud_downsampled->size() << endl;


        // estimate the normal of the Downsampled cloud
        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr Scene_cloud_downsampled_with_normals(new pcl::PointCloud<pcl::PointXYZLNormal>);
        pcl::PointCloud<pcl::Normal>::Ptr Norm_of_Scene_cloud_downsampled (new pcl::PointCloud<pcl::Normal>);



        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(Scene_cloud_downsampled);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(3 * OffLineRadius);
        ne.compute(*Norm_of_Scene_cloud_downsampled);
        pcl::concatenateFields(*Norm_of_Scene_cloud_downsampled, *Norm_of_Scene_cloud_downsampled, *Scene_cloud_downsampled_with_normals);



        // ********* Lets compute N of iterations N
        int n = Scene_cloud_downsampled_with_normals->size();
        int N = int((float(-n) * log(1 - Ps)) / (M * K * C));    // number of iterations needed

        cout << "number of iterations : " << N << endl;

        // ********* Sample one point randomly, and from this point sample points that are d away from this point

        int Num_of_detected_descriptor {0};

        pcl::search::KdTree<pcl::PointXYZLNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZLNormal>);
        tree2->setInputCloud(Scene_cloud_downsampled_with_normals);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZLNormal> ec;
        ec.setClusterTolerance(0.02); // Set your tolerance appropriately
        ec.setMinClusterSize(100);    // Set minimum cluster size
        ec.setMaxClusterSize(25000);  // Set maximum cluster size
        ec.setSearchMethod(tree2);
        ec.setInputCloud(Scene_cloud_downsampled_with_normals);
        ec.extract(cluster_indices);

        cout << "number of clusters in the scene is : " << cluster_indices.size() << endl;

        pcl::RandomSample<pcl::PointXYZ> random_sampler;

        for (int i = 0; i < 5000 ; ++i) {

            std::vector<int> sampled_cloud_indices;
            pcl::PointCloud<pcl::PointXYZLNormal>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZLNormal>());

            for (const pcl::PointIndices & C_id : cluster_indices)
            {
                for (const int &index : C_id.indices) {
                    sampled_cloud->points.push_back(Scene_cloud_downsampled_with_normals->points[index]); // Assuming Downsampled_with_normals is your original cloud
                }

                int index_of_pointA = int(AndreiUtils::double01Sampler.sample() * sampled_cloud->size());
                pcl::PointXYZLNormal Point_one = Scene_cloud_downsampled_with_normals->points[index_of_pointA];
                Eigen::Vector3f Point_one_p = {Point_one.x, Point_one.y, Point_one.z};
                Eigen::Vector3f Point_one_n = {Point_one.normal_x, Point_one.normal_y, Point_one.normal_z};


                pcl::PointXYZLNormal Point_two;

                pcl::KdTreeFLANN<pcl::PointXYZLNormal> kdtree_PointA;
                kdtree_PointA.setInputCloud(Scene_cloud_downsampled_with_normals);

                std::vector<int> pointIdxKNNSearch;
                std::vector<float> pointKNNSquaredDistance;

                pcl::PointCloud<pcl::PointXYZLNormal>::Ptr neighbors_of_PointA(new pcl::PointCloud<pcl::PointXYZLNormal>);

                if (kdtree_PointA.radiusSearch(Point_one, OffLineRadius, pointIdxKNNSearch, pointKNNSquaredDistance) > 0) {
                    for (auto Idx: pointIdxKNNSearch) {
                        neighbors_of_PointA->push_back(Scene_cloud_downsampled_with_normals->points[Idx]);
                    }

                    for (auto P : *neighbors_of_PointA)
                    {
                        if (Euclidean_Distance_two_Vectors(Point_one, P) > (OffLineRadius-tolerance) &&
                            Euclidean_Distance_two_Vectors(Point_one, P) < (OffLineRadius+tolerance))
                        {
                            Point_two = P;


                            Eigen::Vector3f Point_two_p = {Point_two.x, Point_two.y, Point_two.z};
                            Eigen::Vector3f Point_two_n = {Point_two.normal_x, Point_two.normal_y, Point_two.normal_z};

                            auto DESCRIPTOR = Compute_the_descriptor(Point_one, Point_two);
                            std::string DESCRIPTOR_in_str = fromVectorToString(DESCRIPTOR);

                            cout << "HashMap_scene" << DESCRIPTOR_in_str << endl;


                            auto findRes = generatedMap.find(DESCRIPTOR_in_str);
                            if (findRes != generatedMap.end()) {

                                auto InnerMap = generatedMap[DESCRIPTOR_in_str];

                                for (auto Mod : InnerMap)
                                {
                                    std::string detected_model = Mod.first;


                                    cout << "Point One" << Point_one << endl;
                                    cout << "Point Two" << Point_two << endl;

                                    for (auto V : Mod.second)
                                    {
                                        cout << "Point_one_Model " << V.first << endl;
                                        cout << "Point_two_Model " << V.second << endl;

                                    }
                                }
                            } else {
                                //  cout << "Didn't find " << DESCRIPTOR_in_str << endl;
                            }

                        }
                    }
                }
            }
        }
        cout << "num of outer iterations is : " << N /cluster_indices.size() << endl;
        cout << "I found " << Num_of_detected_descriptor << " descriptors " << endl;
    }

    return 0;
}







pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

