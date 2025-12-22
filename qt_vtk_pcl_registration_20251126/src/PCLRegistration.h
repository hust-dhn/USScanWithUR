// version:1.0 20251202

// 定义 DLL 导出/导入宏
#ifdef MYLIB_EXPORTS
#define MYLIB_API __declspec(dllexport)
#else
#define MYLIB_API __declspec(dllimport)
#endif

#include <iostream>
#include <vector>
#include <chrono>
#include <filesystem>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/pca.h>

// using PointT = pcl::PointXYZ;
using PointT = pcl::PointXYZRGB;

namespace fs = std::filesystem;

using Clock = std::chrono::high_resolution_clock;
using Ms = std::chrono::duration<double, std::milli>;

//这个是调整参数结构体参数什么的全部在里面改
//1.体素边长，这个就是采样的时候采用的边长越大采样得到的点云数量越少，可以加速整个配准过程
//2.法向估计半径，因为在计算FPFH特征子的时候是需要计算点云的法向量的，这个估计半径就是在多大范围内你和平面球阀想
//3.FPFH 特征半径,是在统计FPFH局部几何特征时候的邻域范围也就是这个局部特征的大小，太大特征不唯一，太小可能会导致对噪声敏感
//4.最大RANSAC迭代次数这个不过多解释
//5.每次随机采样点数，这个是进行刚体估计的样本数，最少是三如果提高可能会导致找到解空间难度变大甚至找不到解空间
//6.每个样本点的特征候选数，调大可以提升匹配的鲁棒性但是收敛速度会降低，如果减少可能会被局部错误特征误导
//7.特征相似性阈值，可以理解为相似度，一般在0.9-0.95之间，相似度太高也可能找不到解
//8.容忍最大误差顾名思义建议范围1-2
//9.最小内点比例要有多少内点支撑这个解，调大匹配条件更严格，调小匹配质量下降
struct RegistrationParams {
    float voxel_leaf_size;
    float normal_radius;
    float fpfh_radius;
    int   ransac_max_iterations;
    int   ransac_num_samples;
    int   ransac_correspondence_randomness;
    float ransac_similarity_threshold;
    float ransac_max_corr_distance_factor;
    float ransac_inlier_fraction;
    float icp_max_corr_distance_factor;
    int   icp_max_iterations;
    float icp_fitness_score_threshold;
};


struct AlignResult {
    bool   converged = false;

    float  inlier_fraction = 0.0f;      // 内点比例
    std::size_t inlier_count = 0;       // 内点数量
    float  median_residual = std::numeric_limits<float>::infinity(); // 残差中位数

    // -------- 新增：ICP 结果 --------
    bool   icp_converged = false;
    double icp_fitness_score = std::numeric_limits<double>::infinity();

    Eigen::Matrix4f total_transform = Eigen::Matrix4f::Identity();   // 原始source → target
    pcl::PointCloud<PointT>::Ptr aligned =
            pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
};


class MYLIB_API PCLRegistration
{
public:
    PCLRegistration();

    template<typename PointT_>
    Eigen::Matrix4f computePCAAlignmentBetweenClouds(
            const typename pcl::PointCloud<PointT_>::Ptr& source,
            const typename pcl::PointCloud<PointT_>::Ptr& target);

    template <typename PointT_>
    typename pcl::PointCloud<PointT_>::Ptr voxel_downsample(
            const typename pcl::PointCloud<PointT_>::Ptr& cloud,
            float leaf_size);

    pcl::PointCloud<pcl::Normal>::Ptr estimate_normals(
            const pcl::PointCloud<PointT>::Ptr& cloud,
            float radius);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_fpfh(
            const pcl::PointCloud<PointT>::Ptr& cloud,
            const pcl::PointCloud<pcl::Normal>::Ptr& normals,
            const pcl::PointCloud<PointT>::Ptr& surface,
            float radius);

    template<typename PointT_>
    AlignResult register_once(
            const typename pcl::PointCloud<PointT_>::Ptr& target,
            const typename pcl::PointCloud<PointT_>::Ptr& source_raw,
            const RegistrationParams& params);

    double score_alignment(const AlignResult& R);

    pcl::PointCloud<PointT>::Ptr pcl_roiFilter(pcl::PointCloud<PointT>::Ptr cloud_input, Eigen::Vector4f min_point, Eigen::Vector4f max_point);

    void refine_with_icp(
            const pcl::PointCloud<PointT>::Ptr& target,
            const pcl::PointCloud<PointT>::Ptr& source_raw,
            const RegistrationParams& params,
            AlignResult& R);

    bool registration(const RegistrationParams& params, pcl::PointCloud<PointT>::Ptr target_crop,
                      pcl::PointCloud<PointT>::Ptr sourceA_crop, pcl::PointCloud<PointT>::Ptr sourceB_crop,
                      std::string seqA_id, std::string seqB_id, int detect_index, bool debug_flag,
                      std::string &winner_seq_id, Eigen::Matrix4f &total_transform);


// public:
//     int mean_k;
//     double stddev_mul;
//
//     float voxel_leaf_size;                  // 下采样体素边长
//     float normal_radius;                    // 法向估计半径
//     float fpfh_radius;                      // FPFH 特征半径
//
//     int ransac_max_iterations;              // 最大RANSAC迭代次数
//     int ransac_num_samples;                 // 每次随机采样点数
//     int ransac_correspondence_randomness;   // 每个样本点的特征候选数
//     float ransac_similarity_threshold;      // 特征相似性阈值
//     float ransac_max_corr_distance_factor;  // 匹配最大容忍距离 = factor * voxel_size
//     float ransac_inlier_fraction;           // 最小内点比例
//
//     int nr_iterations;
//     double distance_threshold;

};

