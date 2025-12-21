
#include "PCLRegistration.h"

PCLRegistration::PCLRegistration(){

}


template<typename PointT_>
Eigen::Matrix4f PCLRegistration::computePCAAlignmentBetweenClouds(
        const typename pcl::PointCloud<PointT_>::Ptr& source,
        const typename pcl::PointCloud<PointT_>::Ptr& target)
{
    // 质心
    Eigen::Vector4f src_centroid, tgt_centroid;
    pcl::compute3DCentroid(*source, src_centroid);
    pcl::compute3DCentroid(*target, tgt_centroid);

    // 去中心化点云
    typename pcl::PointCloud<PointT_>::Ptr src_centered(new pcl::PointCloud<PointT_>);
    typename pcl::PointCloud<PointT_>::Ptr tgt_centered(new pcl::PointCloud<PointT_>);
    pcl::demeanPointCloud(*source, src_centroid, *src_centered);
    pcl::demeanPointCloud(*target, tgt_centroid, *tgt_centered);

    // PCA
    pcl::PCA<PointT_> pca_src, pca_tgt;
    pca_src.setInputCloud(src_centered);
    pca_tgt.setInputCloud(tgt_centered);

    Eigen::Matrix3f R_src = pca_src.getEigenVectors();
    Eigen::Matrix3f R_tgt = pca_tgt.getEigenVectors();

    // 保证主轴方向一致（避免180°翻转）
    for (int i = 0; i < 3; ++i) {
        if (R_src.col(i).dot(R_tgt.col(i)) < 0.0f) {
            R_src.col(i) *= -1.0f;
        }
    }

    // 旋转
    Eigen::Matrix3f R = R_tgt * R_src.transpose();

    // 平移
    Eigen::Vector3f t = tgt_centroid.head<3>() - R * src_centroid.head<3>();

    // 组装4x4
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

template <typename PointT_>
typename pcl::PointCloud<PointT_>::Ptr PCLRegistration::voxel_downsample(
        const typename pcl::PointCloud<PointT_>::Ptr& cloud,
        float leaf_size)
{
    typename pcl::PointCloud<PointT_>::Ptr filtered(new pcl::PointCloud<PointT_>);
    pcl::VoxelGrid<PointT_> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*filtered);
    return filtered;
}

pcl::PointCloud<pcl::Normal>::Ptr PCLRegistration::estimate_normals(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        float radius)
{
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    ne.setSearchMethod(tree);

    ne.setRadiusSearch(radius);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);
    return normals;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr PCLRegistration::compute_fpfh(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr& normals,
        const pcl::PointCloud<PointT>::Ptr& surface,
        float radius)
{
    pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    fpfh.setSearchSurface(surface);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(radius);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh.compute(*descriptors);
    return descriptors;
}


template<typename PointT_>
AlignResult PCLRegistration::register_once(
        const typename pcl::PointCloud<PointT_>::Ptr& target,
        const typename pcl::PointCloud<PointT_>::Ptr& source_raw,
        const RegistrationParams& params)
{
    AlignResult R;

    // ---------- 7.1 PCA 预对齐 ----------
    Eigen::Matrix4f prealign =
            computePCAAlignmentBetweenClouds<PointT_>(source_raw, target);

    typename pcl::PointCloud<PointT_>::Ptr source(
            new pcl::PointCloud<PointT_>);
    pcl::transformPointCloud(*source_raw, *source, prealign);

    // ---------- 7.2 下采样 ----------
    typename pcl::PointCloud<PointT_>::Ptr target_ds =
            voxel_downsample<PointT_>(target, params.voxel_leaf_size);
    typename pcl::PointCloud<PointT_>::Ptr source_ds =
            voxel_downsample<PointT_>(source, params.voxel_leaf_size);

    // ---------- 7.3 法向 & FPFH ----------
    pcl::PointCloud<pcl::Normal>::Ptr target_normals =
            estimate_normals(target_ds, params.normal_radius);
    pcl::PointCloud<pcl::Normal>::Ptr source_normals =
            estimate_normals(source_ds, params.normal_radius);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh =
            compute_fpfh(target_ds, target_normals, target_ds, params.fpfh_radius);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh =
            compute_fpfh(source_ds, source_normals, source_ds, params.fpfh_radius);

    // ---------- 7.4 RANSAC 先验配准 ----------
    pcl::SampleConsensusPrerejective<PointT_, PointT_, pcl::FPFHSignature33> align;
    align.setInputSource(source_ds);
    align.setSourceFeatures(source_fpfh);
    align.setInputTarget(target_ds);
    align.setTargetFeatures(target_fpfh);

    align.setMaximumIterations(params.ransac_max_iterations);
    align.setNumberOfSamples(params.ransac_num_samples);
    align.setCorrespondenceRandomness(params.ransac_correspondence_randomness);
    align.setSimilarityThreshold(params.ransac_similarity_threshold);
    align.setMaxCorrespondenceDistance(
            params.ransac_max_corr_distance_factor * params.voxel_leaf_size);
    align.setInlierFraction(params.ransac_inlier_fraction);

    align.align(*R.aligned);

    R.converged = align.hasConverged();

    // ---------- 7.5 质量评估 ----------
    if (R.converged) {
        const std::vector<int>& inliers = align.getInliers();
        R.inlier_count = inliers.size();

        // 内点比例
        R.inlier_fraction =
                static_cast<float>(R.inlier_count) /
                std::max<std::size_t>(1, source_ds->size());

        // 计算中位数残差
        std::vector<float> residuals;
        residuals.reserve(inliers.size());

        pcl::KdTreeFLANN<PointT_> kdtree;
        kdtree.setInputCloud(target_ds);

        for (int idx : inliers) {
            std::vector<int> nn(1);
            std::vector<float> sqr(1);
            if (kdtree.nearestKSearch(R.aligned->points[idx], 1, nn, sqr) > 0) {
                residuals.push_back(std::sqrt(sqr[0]));
            }
        }

        if (!residuals.empty()) {
            std::sort(residuals.begin(), residuals.end());
            R.median_residual = residuals[residuals.size() / 2];
        }

        // 把RANSAC给出的最终刚体，加上PCA预对齐，得到原始source→target的总变换
        Eigen::Matrix4f ransacT = align.getFinalTransformation();
        R.total_transform = ransacT * prealign;
    }

    return R;
}

double PCLRegistration::score_alignment(const AlignResult& R)
{
    if (!R.converged) {
        return std::numeric_limits<double>::infinity();
    }
    if (R.icp_converged) {
        // 有 ICP 结果时，优先用 ICP fitness
        return R.icp_fitness_score;
    }
    // 否则退回粗配准的评价
    return static_cast<double>(R.median_residual) /
           std::max(1e-6f, R.inlier_fraction);
}


pcl::PointCloud<PointT>::Ptr PCLRegistration::pcl_roiFilter(pcl::PointCloud<PointT>::Ptr cloud_input, Eigen::Vector4f min_point, Eigen::Vector4f max_point)
{
    pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>());
    // Crop the scene to create ROI
    pcl::CropBox<PointT> roi;
    roi.setMin(min_point);
    roi.setMax(max_point);
    roi.setInputCloud(cloud_input);
    roi.filter(*cloud_out);

    return cloud_out;
}



void PCLRegistration::refine_with_icp(
        const pcl::PointCloud<PointT>::Ptr& target,
        const pcl::PointCloud<PointT>::Ptr& source_raw,
        const RegistrationParams& params,
        AlignResult& R)
{
    if (!R.converged) {
        return;
    }

    // 先用粗配准总变换变换原始 source
    pcl::PointCloud<PointT>::Ptr source_init(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*source_raw, *source_init, R.total_transform);

    // 下采样后再做 ICP
    pcl::PointCloud<PointT>::Ptr target_icp =
            voxel_downsample<PointT>(target, params.voxel_leaf_size);
    pcl::PointCloud<PointT>::Ptr source_icp =
            voxel_downsample<PointT>(source_init, params.voxel_leaf_size);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(source_icp);
    icp.setInputTarget(target_icp);
    icp.setMaxCorrespondenceDistance(
            params.icp_max_corr_distance_factor * params.voxel_leaf_size);
    icp.setMaximumIterations(params.icp_max_iterations);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-6);

    pcl::PointCloud<PointT>::Ptr icp_aligned_ds(new pcl::PointCloud<PointT>);
    icp.align(*icp_aligned_ds);

    R.icp_converged = icp.hasConverged();
    if (!R.icp_converged) {
        return;
    }

    R.icp_fitness_score = icp.getFitnessScore();
    if (R.icp_fitness_score > params.icp_fitness_score_threshold) {
        R.icp_converged = false;
        return;
    }

    // 累积变换
    Eigen::Matrix4f Ticp = icp.getFinalTransformation();
    R.total_transform = Ticp * R.total_transform;

    // 最终变换到原始分辨率点云
    pcl::transformPointCloud(*source_raw, *R.aligned, R.total_transform);
}


bool PCLRegistration::registration(const RegistrationParams& params, pcl::PointCloud<PointT>::Ptr target_crop,
                  pcl::PointCloud<PointT>::Ptr sourceA_crop, pcl::PointCloud<PointT>::Ptr sourceB_crop,
                  std::string seqA_id, std::string seqB_id, int detect_index, bool debug_flag,
                  std::string &winner_seq_id, Eigen::Matrix4f &total_transform)
{
    // --- 对 A 做配准 + ICP ---
    auto T_A_start = Clock::now();
    std::cout << "\n[INFO] === 对 A(序列号 " << seqA_id << ") 配准 ===\n";
    AlignResult RA = register_once<PointT>(target_crop, sourceA_crop, params);
    std::cout << "[INFO] 粗配准收敛: " << std::boolalpha << RA.converged
              << ", 内点比例: " << RA.inlier_fraction
              << ", 内点数: " << RA.inlier_count
              << ", 残差中位数: " << RA.median_residual << "\n";

    refine_with_icp(target_crop, sourceA_crop, params, RA);
    if (RA.icp_converged) {
        std::cout << "[INFO] ICP(A) 收敛, fitness: " << RA.icp_fitness_score << "\n";
    }
    else {
        std::cout << "[WARN] ICP(A) 未通过收敛或筛选\n";
    }
    double scoreA = score_alignment(RA);
    std::cout << "[INFO] 综合得分A(越小越好): " << scoreA << "\n";
    auto T_A_end = Clock::now();
    std::cout << "[TIME] A 配准(含 ICP)耗时: "
              << Ms(T_A_end - T_A_start).count() << " ms\n";

    // --- 对 B 做配准 + ICP ---
    auto T_B_start = Clock::now();
    std::cout << "\n[INFO] === 对 B(序列号 " << seqB_id << ") 配准 ===\n";
    AlignResult RB = register_once<PointT>(target_crop, sourceB_crop, params);
    std::cout << "[INFO] 粗配准收敛: " << std::boolalpha << RB.converged
              << ", 内点比例: " << RB.inlier_fraction
              << ", 内点数: " << RB.inlier_count
              << ", 残差中位数: " << RB.median_residual << "\n";

    refine_with_icp(target_crop, sourceB_crop, params, RB);
    if (RB.icp_converged) {
        std::cout << "[INFO] ICP(B) 收敛, fitness: " << RB.icp_fitness_score << "\n";
    }
    else {
        std::cout << "[WARN] ICP(B) 未通过收敛或筛选\n";
    }
    double scoreB = score_alignment(RB);
    std::cout << "[INFO] 综合得分B(越小越好): " << scoreB << "\n";
    auto T_B_end = Clock::now();
    std::cout << "[TIME] B 配准(含 ICP)耗时: "
              << Ms(T_B_end - T_B_start).count() << " ms\n";

    // --- 选这一组里更好的那个 ---
    // std::string winner_seq_id;
    // std::string winner_path;
    AlignResult* RW = nullptr;

    if (scoreA < scoreB) {
        winner_seq_id = seqA_id;
        // winner_path = sourceA_path_local;
        RW = &RA;

        if (debug_flag && sourceA_crop->size() > 0) {
            pcl::PointCloud<PointT>::Ptr sourceA_tf(new pcl::PointCloud<PointT>);
            pcl::transformPointCloud(*sourceA_crop, *sourceA_tf, RA.total_transform);
            pcl::io::savePCDFileASCII("./save_img_pcd_dir/" + std::to_string(detect_index) + "_" + seqA_id + ".pcd", *sourceA_crop);
            pcl::io::savePCDFileASCII("./save_img_pcd_dir/" + std::to_string(detect_index) + "_" + seqA_id + "_tf.pcd", *sourceA_tf);
        }
    }
    else {
        winner_seq_id = seqB_id;
        // winner_path = sourceB_path_local;
        RW = &RB;

        if (debug_flag && sourceB_crop->size() > 0) {
            pcl::PointCloud<PointT>::Ptr sourceB_tf(new pcl::PointCloud<PointT>);
            pcl::transformPointCloud(*sourceB_crop, *sourceB_tf, RB.total_transform);
            pcl::io::savePCDFileASCII("./save_img_pcd_dir/" + std::to_string(detect_index) + "_" + seqB_id + ".pcd", *sourceB_crop);
            pcl::io::savePCDFileASCII("./save_img_pcd_dir/" + std::to_string(detect_index) + "_" + seqB_id + "_tf.pcd", *sourceB_tf);
        }
    }

    std::cout << "\n[RESULT] 组 " << detect_index << " 的配准结果:\n";
    if (RW && RW->converged) {
        std::cout << "  [BEST] 更好的配准序列号: " << winner_seq_id << "\n";
        // std::cout << "  [BEST] 来源文件: " << winner_path << "\n";
        std::cout << "  [BEST] 内点比例=" << RW->inlier_fraction
                  << ", 内点数=" << RW->inlier_count
                  << ", 残差中位数=" << RW->median_residual << "\n";
        if (RW->icp_converged) {
            std::cout << "  [BEST] ICP fitness=" << RW->icp_fitness_score << "\n";
        }
        else {
            std::cout << "  [BEST] ICP 未通过筛选，使用粗配准结果\n";
        }
        total_transform = RW->total_transform;
        std::cout << "  [BEST] 全流程总变换矩阵（原始 source → target）:\n" << RW->total_transform << "\n";
        return true;
    }
    else {
        std::cout << "  [WARN] 该组中两个候选都未成功收敛\n";
        return false;
    }

}

