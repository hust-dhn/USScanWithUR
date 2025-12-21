#include <iostream>
#include <vector>
#include <chrono>
#include <filesystem>

#include "PCLRegistration.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

//================= 全局参数（由 YAML 读取） =================
float roi_xmin_target;
float roi_xmax_target;
float roi_ymin_target;
float roi_ymax_target;
float roi_zmin_target;
float roi_zmax_target;
float roi_xmin_source;
float roi_xmax_source;
float roi_ymin_source;
float roi_ymax_source;
float roi_zmin_source;
float roi_zmax_source;

float voxel_leaf_size;
float normal_radius;
float fpfh_radius;

int   ransac_max_iterations;
int   ransac_num_samples;
int   ransac_correspondence_randomness;
float ransac_similarity_threshold;
float ransac_max_corr_distance_factor;
float ransac_inlier_fraction;

// ICP 参数（给默认值，YAML 可覆盖）
float icp_max_corr_distance_factor;
int   icp_max_iterations;
float icp_fitness_score_threshold;

std::string debug_flag_str;
bool debug_flag = false;

std::string target_path;
std::string source_dir;

//================= 读取 YAML 参数 =================
void LoadGlobalParam(std::string& yaml_path) {
    if (yaml_path.empty()) {
        std::string str = "[ERROR] yaml_path is empty!";
        std::cerr << str << std::endl;
    }
    else {

        cv::FileStorage paramFs2(yaml_path, cv::FileStorage::READ);

        paramFs2["roi_xmin_target"] >> roi_xmin_target;
        paramFs2["roi_xmax_target"] >> roi_xmax_target;
        paramFs2["roi_ymin_target"] >> roi_ymin_target;
        paramFs2["roi_ymax_target"] >> roi_ymax_target;
        paramFs2["roi_zmin_target"] >> roi_zmin_target;
        paramFs2["roi_zmax_target"] >> roi_zmax_target;
        paramFs2["roi_xmin_source"] >> roi_xmin_source;
        paramFs2["roi_xmax_source"] >> roi_xmax_source;
        paramFs2["roi_ymin_source"] >> roi_ymin_source;
        paramFs2["roi_ymax_source"] >> roi_ymax_source;
        paramFs2["roi_zmin_source"] >> roi_zmin_source;
        paramFs2["roi_zmax_source"] >> roi_zmax_source;

        paramFs2["voxel_leaf_size"] >> voxel_leaf_size;
        paramFs2["normal_radius"] >> normal_radius;
        paramFs2["fpfh_radius"] >> fpfh_radius;

        paramFs2["ransac_max_iterations"] >> ransac_max_iterations;
        paramFs2["ransac_num_samples"] >> ransac_num_samples;
        paramFs2["ransac_correspondence_randomness"] >> ransac_correspondence_randomness;
        paramFs2["ransac_similarity_threshold"] >> ransac_similarity_threshold;
        paramFs2["ransac_max_corr_distance_factor"] >> ransac_max_corr_distance_factor;
        paramFs2["ransac_inlier_fraction"] >> ransac_inlier_fraction;

        // ---------- 可选：ICP 参数 ----------
        cv::FileNode node_icp_dist = paramFs2["icp_max_corr_distance_factor"];
        if (!node_icp_dist.empty()) node_icp_dist >> icp_max_corr_distance_factor;
        cv::FileNode node_icp_iter = paramFs2["icp_max_iterations"];
        if (!node_icp_iter.empty()) node_icp_iter >> icp_max_iterations;
        cv::FileNode node_icp_thr = paramFs2["icp_fitness_score_threshold"];
        if (!node_icp_thr.empty()) node_icp_thr >> icp_fitness_score_threshold;

        paramFs2["debug_flag_str"] >> debug_flag_str;
        if (std::strcmp(debug_flag_str.c_str(), "true") == 0) {
            debug_flag = true;
        }
        else if (std::strcmp(debug_flag_str.c_str(), "false") == 0) {
            debug_flag = false;
        }
        else {
            std::cerr << "[ERROR] debug_flag_str must be 'true' or 'false'" << std::endl;
            exit(-1);
        };

        paramFs2["target_path"] >> target_path;
        paramFs2["source_dir"] >> source_dir;

        std::cout << "roi_xmin_target:" << roi_xmin_target << std::endl;
        std::cout << "roi_xmax_target:" << roi_xmax_target << std::endl;
        std::cout << "roi_ymin_target:" << roi_ymin_target << std::endl;
        std::cout << "roi_ymax_target:" << roi_ymax_target << std::endl;
        std::cout << "roi_zmin_target:" << roi_zmin_target << std::endl;
        std::cout << "roi_zmax_target:" << roi_zmax_target << std::endl;
        std::cout << "roi_xmin_source:" << roi_xmin_source << std::endl;
        std::cout << "roi_xmax_source:" << roi_xmax_source << std::endl;
        std::cout << "roi_ymin_source:" << roi_ymin_source << std::endl;
        std::cout << "roi_ymax_source:" << roi_ymax_source << std::endl;
        std::cout << "roi_zmin_source:" << roi_zmin_source << std::endl;
        std::cout << "roi_zmax_source:" << roi_zmax_source << std::endl;

        std::cout << "voxel_leaf_size:" << voxel_leaf_size << std::endl;
        std::cout << "normal_radius:" << normal_radius << std::endl;
        std::cout << "fpfh_radius:" << fpfh_radius << std::endl;

        std::cout << "ransac_max_iterations:" << ransac_max_iterations << std::endl;
        std::cout << "ransac_num_samples:" << ransac_num_samples << std::endl;
        std::cout << "ransac_correspondence_randomness:" << ransac_correspondence_randomness << std::endl;
        std::cout << "ransac_similarity_threshold:" << ransac_similarity_threshold << std::endl;
        std::cout << "ransac_max_corr_distance_factor:" << ransac_max_corr_distance_factor << std::endl;
        std::cout << "ransac_inlier_fraction:" << ransac_inlier_fraction << std::endl;

        std::cout << "icp_max_corr_distance_factor:" << icp_max_corr_distance_factor << std::endl;
        std::cout << "icp_max_iterations:" << icp_max_iterations << std::endl;
        std::cout << "icp_fitness_score_threshold:" << icp_fitness_score_threshold << std::endl;

        std::cout << "debug_flag:" << std::boolalpha << debug_flag << std::endl;

        std::cout << "target_path:" << target_path << std::endl;
        std::cout << "source_dir:" << source_dir << std::endl;
        std::cout << std::endl << std::endl;
    }
}

//================= 在目录中找到唯一的 .ply =================
std::string find_single_ply_in_dir(const fs::path& dir)
{
    if (!fs::exists(dir) || !fs::is_directory(dir)) {
        std::cerr << "[ERROR] 目录不存在或不是目录: " << dir << "\n";
        return "";
    }

    std::string found_path;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;
        auto ext = entry.path().extension().string();
        for (auto& c : ext) c = static_cast<char>(::tolower(c));
        if (ext == ".ply") {
            if (!found_path.empty()) {
                std::cerr << "[ERROR] 目录中找到多于一个 .ply 文件: " << dir << "\n";
                return "";
            }
            found_path = entry.path().string();
        }
    }

    if (found_path.empty()) {
        std::cerr << "[ERROR] 目录中没有找到 .ply 文件: " << dir << "\n";
    }
    return found_path;
}



//================= main =================
int main(int argc, char* argv[]) {

    PCLRegistration *pcl_regist = new PCLRegistration();

    std::string yaml_path = "./config/config.yaml";
    LoadGlobalParam(yaml_path);

    // 通用参数结构体（单组模式和批处理模式共用）
    RegistrationParams params;
    params.voxel_leaf_size = voxel_leaf_size;
    params.normal_radius = normal_radius;
    params.fpfh_radius = fpfh_radius;
    params.ransac_max_iterations = ransac_max_iterations;
    params.ransac_num_samples = ransac_num_samples;
    params.ransac_correspondence_randomness = ransac_correspondence_randomness;
    params.ransac_similarity_threshold = ransac_similarity_threshold;
    params.ransac_max_corr_distance_factor = ransac_max_corr_distance_factor;
    params.ransac_inlier_fraction = ransac_inlier_fraction;

    params.icp_max_corr_distance_factor = icp_max_corr_distance_factor;
    params.icp_max_iterations = icp_max_iterations;
    params.icp_fitness_score_threshold = icp_fitness_score_threshold;

    auto T_start = Clock::now();
    std::cout << std::fixed << std::setprecision(6);

    //===============================================================================
    //===============================================================================
    // 获取源点云路径

    fs::path root_dir(source_dir);
    if (!fs::exists(root_dir) || !fs::is_directory(root_dir)) {
        std::cerr << "[ERROR] 根目录不存在或不是目录: " << root_dir << "\n";
        return -1;
    }

    std::vector<std::string> sourceA_path_vector;
    std::vector<std::string> sourceB_path_vector;

    // 遍历根目录下的组目录（1、2、3、...）
    for (const auto& group_entry : fs::directory_iterator(root_dir)) {
        if (!group_entry.is_directory()) continue;

        fs::path group_dir = group_entry.path();
        std::string group_name = group_dir.filename().string();

        // std::cout << "\n============================================\n";
        // std::cout << "[GROUP] 正在处理组目录: " << group_name << "\n";

        // 找到该组目录下的所有“序列号目录”（期望为2个）
        std::vector<fs::path> seq_dirs;
        for (const auto& seq_entry : fs::directory_iterator(group_dir)) {
            if (!seq_entry.is_directory()) continue;
            seq_dirs.push_back(seq_entry.path());
        }

        if (seq_dirs.size() != 2) {
            std::cerr << "[WARN] 组目录 " << group_name
                      << " 下的子目录数量不是 2，跳过该组。\n";
            continue;
        }

        fs::path seqA_dir = seq_dirs[0];
        fs::path seqB_dir = seq_dirs[1];
        // std::string seqA_id = seqA_dir.filename().string();
        // std::string seqB_id = seqB_dir.filename().string();

        // std::cout << "[INFO] 序列号A: " << seqA_id << "\n";
        // std::cout << "[INFO] 序列号B: " << seqB_id << "\n";

        // 找到各自目录中的唯一 .ply 文件
        std::string sourceA_path_local = find_single_ply_in_dir(seqA_dir);
        std::string sourceB_path_local = find_single_ply_in_dir(seqB_dir);
        if (sourceA_path_local.empty() || sourceB_path_local.empty()) {
            std::cerr << "[WARN] 组 " << group_name << " 中 .ply 文件查找失败，跳过该组。\n";
            continue;
        }
        sourceA_path_vector.push_back(sourceA_path_local);
        sourceB_path_vector.push_back(sourceB_path_local);

    }

    assert(sourceA_path_vector.size() == sourceB_path_vector.size());
    for (int k = 0; k < sourceA_path_vector.size(); ++k) {
        std::cerr << "[INFO] " << k+1 << ", 局部点云 " << sourceA_path_vector[k] << "\n";
    }
    std::cerr << "\n";
    for (int k = 0; k < sourceB_path_vector.size(); ++k) {
        std::cerr << "[INFO] " << k+1 << ", 局部点云 " << sourceB_path_vector[k] << "\n";
    }

    //===============================================================================
    //===============================================================================
    // 读取目标点云
    auto T_read_tgt_start = Clock::now();
    pcl::PointCloud<PointT>::Ptr target(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPLYFile(target_path, *target) < 0) {
        std::cerr << "[ERROR] Failed to load target PLY file: " << target_path << "\n";
        return -1;
    }
    auto T_read_tgt_end = Clock::now();
    std::cout << "[TIME] 目标点云读取耗时: " << Ms(T_read_tgt_end - T_read_tgt_start).count() << " ms\n";

    // ROI 裁剪
    Eigen::Vector4f minPoint_target(roi_xmin_target, roi_ymin_target, roi_zmin_target, 1);
    Eigen::Vector4f maxPoint_target(roi_xmax_target, roi_ymax_target, roi_zmax_target, 1);
    pcl::PointCloud<PointT>::Ptr target_crop = pcl_regist->pcl_roiFilter(target, minPoint_target, maxPoint_target);
    std::cout << "[INFO] target_crop->points.size(): " << target_crop->size() << std::endl;
    if (debug_flag) {
        if (target_crop->size() > 0)
            pcl::io::savePCDFileASCII("./save_img_pcd_dir/target_crop.pcd", *target_crop);
    }

    //===============================================================================
    //===============================================================================

    for (int detect_index = 0; detect_index < sourceA_path_vector.size(); ++detect_index) {

        std::cout << "\n============================================\n";
        std::cout << "[GROUP] 正在处理组目录: " << detect_index+1 << "\n";

        std::string filePath_A = sourceA_path_vector[detect_index];
        std::string filePath_B = sourceB_path_vector[detect_index];
        std::string temp1 = filePath_A.substr(0, filePath_A.find_last_of("\\"));
        std::string temp2 = filePath_B.substr(0, filePath_B.find_last_of("\\"));
        std::string seqA_id = temp1.substr(temp1.find_last_of("\\")+1, sizeof(temp1));
        std::string seqB_id = temp2.substr(temp2.find_last_of("\\")+1, sizeof(temp2));
        std::cout << "[INFO] 序列号A: " << seqA_id << "\n";
        std::cout << "[INFO] 序列号B: " << seqB_id << "\n";

        //===============================================================================
        //===============================================================================
        // 读取源点云
        auto T_read_src_start = Clock::now();
        pcl::PointCloud<PointT>::Ptr sourceA(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr sourceB(new pcl::PointCloud<PointT>);
        if (pcl::io::loadPLYFile(sourceA_path_vector[detect_index], *sourceA) < 0 ||
            pcl::io::loadPLYFile(sourceB_path_vector[detect_index], *sourceB) < 0)
        {
            std::cerr << "[ERROR] Failed to load source PLY files in group " << detect_index+1 << "\n";
            continue;
        }
        auto T_read_src_end = Clock::now();
        std::cout << "[TIME] 组 " << detect_index+1 << " 源点云读取耗时: "
                  << Ms(T_read_src_end - T_read_src_start).count() << " ms\n";

        //===============================================================================
        //===============================================================================
        // ROI 裁剪
        Eigen::Vector4f minPoint_source(roi_xmin_source, roi_ymin_source, roi_zmin_source, 1);
        Eigen::Vector4f maxPoint_source(roi_xmax_source, roi_ymax_source, roi_zmax_source, 1);
        pcl::PointCloud<PointT>::Ptr sourceA_crop = pcl_regist->pcl_roiFilter(sourceA, minPoint_source, maxPoint_source);
        pcl::PointCloud<PointT>::Ptr sourceB_crop = pcl_regist->pcl_roiFilter(sourceB, minPoint_source, maxPoint_source);

        std::cout << "[INFO] sourceA_crop->points.size(): " << sourceA_crop->points.size() << std::endl;
        std::cout << "[INFO] sourceB_crop->points.size(): " << sourceB_crop->points.size() << std::endl;

        // if (debug_flag) {
        //     if (sourceA_crop->size() > 0)
        //         pcl::io::savePCDFileASCII("./save_img_pcd_dir/sourceA_crop_" + group_name + ".pcd", *sourceA_crop);
        //     if (sourceB_crop->size() > 0)
        //         pcl::io::savePCDFileASCII("./save_img_pcd_dir/sourceB_crop_" + group_name + ".pcd", *sourceB_crop);
        // }

        //===============================================================================
        //===============================================================================
        // 点云配准
        std::string cam_seq;
        Eigen::Matrix4f best_transform;
        bool bRet = pcl_regist->registration(params, target_crop, sourceA_crop, sourceB_crop, seqA_id, seqB_id, detect_index + 1,
                                 debug_flag, cam_seq, best_transform);
        if(bRet){
            std::cout << "=====cam_seq: " << cam_seq << "\n";
            std::cout << "=====best_transform（原始 source → target）:\n" << best_transform << "\n";
        }

        //===============================================================================
        //===============================================================================

    }

    //===============================================================================
    //===============================================================================
    auto T_total_end = Clock::now();
    std::cout << "\n[TIME] 整个批处理耗时: " << Ms(T_total_end - T_start).count() << " ms\n";


    return 0;

}

