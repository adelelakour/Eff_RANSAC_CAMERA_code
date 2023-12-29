#include "Model_PreProsessing.h"

#include <iostream>
#include <Eigen/Core>
#include <algorithm>
#include <utility>
#include <vector>
#include <set>
#include <list>
#include <tuple>
#include <string>
#include <cmath>
#include <filesystem>


#include <pcl/octree/octree_search.h>
#include <pcl/common/distances.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/registration/correspondence_estimation.h>
#include <AndreiUtils/utilsString.h>
#include <AndreiUtils/classes/RandomNumberGenerator.hpp>


using namespace AndreiUtils;
using namespace Eigen;
using namespace std;
namespace fs = std::filesystem;

Eigen::Vector3f fromStringToVectorF(std::string const &vectorAsString) {
    auto res = AndreiUtils::splitString(vectorAsString, ";");
    assert(res.size() == 3);
    return {std::stof(res[0]), std::stof(res[1]), std::stof(res[2])};
}

std::vector<int> stringToVector(const std::string& str) {
    std::vector<int> vec;
    std::stringstream ss(str);
    int temp;
    while (ss >> temp) {
        vec.push_back(temp);
    }
    return vec;
}



std::string fromVectorToString(Eigen::Vector3f const &vector) {
    return std::to_string(vector.x()) + ";" + std::to_string(vector.y()) + ";" + std::to_string(vector.z());
}


Eigen::Vector3d fromStringToVectorD(std::string const &vectorAsString) {
    auto res = AndreiUtils::splitString(vectorAsString, ";");
    assert(res.size() == 3);
    return {std::stod(res[0]), std::stod(res[1]), std::stod(res[2])};
}


std::string fromVectorToString(Eigen::Vector3d const &vector) {
    return std::to_string(vector.x()) + ";" + std::to_string(vector.y()) + ";" + std::to_string(vector.z());
}

Eigen::Vector3f Hash_key = {0, 0, 0};
Eigen::Vector3f An;
Eigen::Vector3f Bn;
Eigen::Vector3f A_B;
Eigen::Vector3f B_A;
double dotProduct;
double magnitudeA;
double magnitudeB;
double angleRadians;
double angleDegrees;


float Angle_between_two_vectors ( Eigen::Vector3f U, Eigen::Vector3f V)
{
    float angle;
    auto dot = (U.x() * V.x()) + (U.y() * V.y()) + (U.z() * V.z());
    auto cross_x = (U.y() * V.z()) - (U.z()*V.y());
    auto cross_y = (U.z() * V.x()) - (U.x() * V.z());
    auto cross_z = ( U.x() * V.y()) - ( U.y() * V.x() );

    auto det = sqrt(cross_x*cross_x + cross_y*cross_y + cross_z*cross_z);
    angle = atan2(det, dot);

    return angle;
}




double Euclidean_Distance_two_Vectors(pcl::PointXYZLNormal A, pcl::PointXYZLNormal B) {
    double Distance = std::sqrt(
            std::pow(B.x - A.x, 2) +
            std::pow(B.y - A.y, 2) +
            std::pow(B.z - A.z, 2)
    );
    return Distance;
}



Eigen::Vector3f Compute_the_descriptor (pcl::PointXYZLNormal U, pcl::PointXYZLNormal V)
{
    Eigen::Vector3f Descriptor;
    Eigen::Vector3f P_u = {U.x, U.y, U.z};
    Eigen::Vector3f P_v = {V.x, V.y, V.z};

    Eigen::Vector3f N_u = {U.normal_x, U.normal_y, U.normal_z};
    Eigen::Vector3f N_v = {V.normal_x, V.normal_y, V.normal_z};

    Descriptor[0] = Angle_between_two_vectors(N_u, N_v);
    Descriptor[1] = Angle_between_two_vectors (N_u, P_v-P_u);
    Descriptor[2] = Angle_between_two_vectors (N_v, P_u-P_v);

    return Descriptor;

}




OuterMap Compute_HashTable(float Offline_radius_preProcessed, double tolerance_preProcessed, float octree_resolution, std::string path_to_models) {
    OuterMap myHashTable;       // Cell is a tuple of (pair) and (model Name)
    myHashTable.clear();

    pcl::PointCloud<pcl::PointXYZ>::Ptr Model_Cloud(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointXYZLNormal PointA_XYZLNormal;
    pcl::PointXYZLNormal PointB_XYZLNormal;


    Eigen::Vector3f N_u = Eigen::Vector3f::Zero();
    Eigen::Vector3f N_v = Eigen::Vector3f::Zero();
    Eigen::Vector3f P_u = Eigen::Vector3f::Zero();
    Eigen::Vector3f P_v = Eigen::Vector3f::Zero();

    One_PAIR Detected_pairs;
    double pointDistance;
    InnerMap OneCell;

    std::vector<int> point_indices;
    std::vector<float> point_distances;


    std::string directory_path = path_to_models;
    std::string extension = ".ply";
    std::string Model_Name;
    std::string file_path;


    for (const auto &entry: fs::directory_iterator(directory_path)) {
        if (entry.is_regular_file() && entry.path().extension() == extension) {
            file_path = entry.path().string();
            Model_Name = entry.path().filename().stem();
        }

        pcl::io::loadPLYFile(file_path, *Model_Cloud);

        pcl::PointCloud<pcl::Normal>::Ptr Model_Normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr Model_with_norm_estimation(
                new pcl::PointCloud<pcl::PointXYZLNormal>);


        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(Model_Cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(5 * Offline_radius_preProcessed);
        ne.compute(*Model_Normals);
        pcl::concatenateFields(*Model_Cloud, *Model_Normals,
                               *Model_with_norm_estimation);


        for (int i = 0; i < Model_with_norm_estimation->size(); ++i) {

            int index_of_pointA = int(AndreiUtils::double01Sampler.sample() * Model_with_norm_estimation->size());
            PointA_XYZLNormal = Model_with_norm_estimation->points[index_of_pointA];

            Eigen::Vector3f A_p = Eigen::Vector3f(PointA_XYZLNormal.x, PointA_XYZLNormal.y, PointA_XYZLNormal.z);
            Eigen::Vector3f A_n = Eigen::Vector3f(PointA_XYZLNormal.normal_x, PointA_XYZLNormal.normal_y,
                                                  PointA_XYZLNormal.normal_z);

            pcl::KdTreeFLANN<pcl::PointXYZLNormal> kdtree_PointA;
            kdtree_PointA.setInputCloud(Model_with_norm_estimation);


            // create a KD-tree (d search) around PointA_XYZLNormal
            std::vector<int> pointIdxKNNSearch_preProcessed;
            std::vector<float> pointKNNSquaredDistance_preProcessed;
            pcl::PointCloud<pcl::PointXYZLNormal>::Ptr neighbors_of_PointA(new pcl::PointCloud<pcl::PointXYZLNormal>);


            if (kdtree_PointA.radiusSearch(PointA_XYZLNormal, Offline_radius_preProcessed,
                                           pointIdxKNNSearch_preProcessed, pointKNNSquaredDistance_preProcessed) > 0) {

                for (int i = 0; i < pointIdxKNNSearch_preProcessed.size(); ++i) {
                    auto PointInRegion = Model_with_norm_estimation->points[pointIdxKNNSearch_preProcessed[i]];
                    if (Euclidean_Distance_two_Vectors(PointA_XYZLNormal, PointInRegion) >
                        (Offline_radius_preProcessed - tolerance_preProcessed) &&
                        Euclidean_Distance_two_Vectors(PointA_XYZLNormal, PointInRegion) <
                        (Offline_radius_preProcessed + tolerance_preProcessed)) {
                        PointB_XYZLNormal = PointInRegion;
                        Eigen::Vector3f B_p;
                        Eigen::Vector3f B_n;

                        B_p = {PointB_XYZLNormal.x, PointB_XYZLNormal.y, PointB_XYZLNormal.z};
                        B_n = {PointB_XYZLNormal.normal_x, PointB_XYZLNormal.normal_y, PointB_XYZLNormal.normal_z};


                        Hash_key[0] = Angle_between_two_vectors(A_n, B_n);
                        Hash_key[1] = Angle_between_two_vectors(A_n, B_p - A_p);
                        Hash_key[2] = Angle_between_two_vectors(B_n, A_p - B_p);

                        Detected_pairs.first = PointA_XYZLNormal;
                        Detected_pairs.second = PointB_XYZLNormal;

                        string hashKeyString = fromVectorToString(Hash_key);

                        //cout << "HashMap" << hashKeyString << endl;

                        auto findRequest = myHashTable.find(hashKeyString);
                        if (findRequest != myHashTable.end()) {    // it exists
                            findRequest->second[Model_Name].push_back(Detected_pairs);
                            // std::cout << "A new entry is added to existing key ++++++++++++++++++++++++++++:" << std::endl;
                        } else {
                            InnerMap newData;
                            newData[Model_Name].push_back(Detected_pairs);
                            myHashTable[hashKeyString] = newData;
                        }
                    }

                }
            }
        }
    }

    std::cout << "Size of myHashTable is: " << myHashTable.size() << std::endl;


    return myHashTable;
}

