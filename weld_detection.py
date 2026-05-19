import os
import open3d as o3d
import numpy as np
from scipy.interpolate import UnivariateSpline

def run_weld_seam_detection(ply_path, output_dir=None):
    """
    点云焊缝拼缝检测与中心轨迹提取完整 pipeline。
    适用于：两块未焊接钢板拼拢后，中间存在一条「物理空隙/无点间隙」的场景。
    算法利用沿Y轴切片、在各切片内沿X轴寻找符合缝隙宽度范围的「中断空洞」，并进行连续性追踪。
    """
    if output_dir is None:
        output_dir = os.path.dirname(ply_path)
    
    print("========== 1. 加载点云 ==========")
    pcd = o3d.io.read_point_cloud(ply_path)
    print(f"原始点云包含点数: {len(pcd.points)}")
    
    # 1. 统计滤波去噪
    print("正在进行统计滤波去噪...")
    pcd_filtered, inlier_indices = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"去噪后点数: {len(pcd_filtered.points)}")
    
    # 2. 体素降采样 (1mm 间距，保留几何边界并大幅加速计算)
    voxel_size = 0.001
    print(f"正在进行下采样 (体素大小={voxel_size*1000:.1f}mm)...")
    pcd_down = pcd_filtered.voxel_down_sample(voxel_size=voxel_size)
    points = np.asarray(pcd_down.points)
    print(f"下采样后点数: {len(pcd_down.points)}")
    
    print("\n========== 2. 拟合钢板基准面 ==========")
    # 估计法线
    pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=30))
    # 拟合主平面（用于将轨迹投影回平滑平面）
    plane_model, inliers = pcd_down.segment_plane(distance_threshold=0.0015,
                                                 ransac_n=3,
                                                 num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"拟合基准面方程: {a:.6f}x + {b:.6f}y + {c:.6f}z + {d:.6f} = 0")
    
    print("\n========== 3. 沿走向切片并追踪缝隙中心 ==========")
    # 获取 Y 轴范围并分段切片
    y_coords = points[:, 1]
    y_min, y_max = np.min(y_coords), np.max(y_coords)
    
    num_slices = 150
    y_bins = np.linspace(y_min, y_max, num_slices + 1)
    
    detected_centers = []
    boundary_points = []
    last_x = None  # 记录上一次检测到的缝隙X中心，用于连续性追踪，防止误检钢板外边缘的虚空
    
    # 定义缝隙物理尺度的合理区间：缝隙宽度在 2.0mm 到 8.0mm 之间
    min_width = 0.0020
    max_width = 0.0080
    
    for i in range(num_slices):
        y_left = y_bins[i]
        y_right = y_bins[i+1]
        y_center = (y_left + y_right) / 2.0
        
        # 提取当前切片内的点
        mask = (y_coords >= y_left) & (y_coords < y_right)
        slice_pts = points[mask]
        
        if len(slice_pts) < 30:
            continue
            
        # 沿 X 轴（垂直于缝隙的方向）对点云排序
        x_vals = slice_pts[:, 0]
        sort_idx = np.argsort(x_vals)
        x_sorted = x_vals[sort_idx]
        pts_sorted = slice_pts[sort_idx]
        
        # 计算相邻点之间的 X 轴距离
        diffs = np.diff(x_sorted)
        
        # 寻找符合拼缝宽度区间 [2mm, 8mm] 的所有空隙索引
        valid_indices = np.where((diffs >= min_width) & (diffs <= max_width))[0]
        
        # 如果未找到，放宽阈值至 [1.5mm, 12mm] 再次搜寻
        if len(valid_indices) == 0:
            valid_indices = np.where((diffs >= 0.0015) & (diffs <= 0.0120))[0]
            
        if len(valid_indices) == 0:
            continue
            
        # 筛选左右两侧有真实钢板存在的候选空隙点（排除钢板外边缘）
        filtered_candidates = []
        for idx in valid_indices:
            pt1 = pts_sorted[idx]
            pt2 = pts_sorted[idx + 1]
            x_l = pt1[0]
            x_r = pt2[0]
            
            # 计算拼缝左右两侧（5mm 到 45mm 范围内）的点云数量
            left_count = np.sum((x_vals > x_l - 0.045) & (x_vals < x_l - 0.005))
            right_count = np.sum((x_vals > x_r + 0.005) & (x_vals < x_r + 0.045))
            
            # 两侧必须均有足够的点云覆盖，才判定为拼缝而不是外边缘
            if left_count >= 60 and right_count >= 60:
                filtered_candidates.append(idx)
                
        if len(filtered_candidates) == 0:
            continue
            
        # 根据连续性选择最匹配的间隙
        best_idx = None
        if last_x is None:
            # 第一切片：选择其中最靠近点云水平中轴的间隙
            x_centers = [(x_sorted[idx] + x_sorted[idx+1])/2.0 for idx in filtered_candidates]
            mid_x = (np.min(x_vals) + np.max(x_vals)) / 2.0
            best_idx = filtered_candidates[np.argmin([abs(xc - mid_x) for xc in x_centers])]
        else:
            # 后续切片：追踪 X 轴上最靠近上一个切片缝隙位置的间隙
            distances_to_last = []
            for idx in filtered_candidates:
                x_center = (x_sorted[idx] + x_sorted[idx+1]) / 2.0
                distances_to_last.append(abs(x_center - last_x))
            best_idx = filtered_candidates[np.argmin(distances_to_last)]
            
        pt1 = pts_sorted[best_idx]
        pt2 = pts_sorted[best_idx + 1]
        center_pt = (pt1 + pt2) / 2.0
        
        # 更新追踪的 X 轴参考位置
        last_x = center_pt[0]
        
        # 将 Z 投影回钢板基准面上，保证轨迹高程的平整性
        z_proj = -(a * center_pt[0] + b * y_center + d) / c
        clean_center = np.array([center_pt[0], y_center, z_proj])
        
        detected_centers.append(clean_center)
        # 保存两侧的缝隙边界点
        boundary_points.append(pt1)
        boundary_points.append(pt2)
        
    detected_centers = np.array(detected_centers)
    boundary_points = np.array(boundary_points)
    print(f"追踪算法初步提取到 {len(detected_centers)} 个缝隙中心点。")
    
    if len(detected_centers) == 0:
        raise ValueError("未能通过追踪提取到任何符合要求的拼缝间隙。")

    print("\n========== 4. 轨迹噪点过滤 (DBSCAN) ==========")
    centers_pcd = o3d.geometry.PointCloud()
    centers_pcd.points = o3d.utility.Vector3dVector(detected_centers)
    
    # 移除跳变点
    labels = np.array(centers_pcd.cluster_dbscan(eps=0.015, min_points=10))
    unique_labels, counts = np.unique(labels, return_counts=True)
    mask = unique_labels != -1
    unique_labels = unique_labels[mask]
    counts = counts[mask]
    
    if len(counts) > 0:
        largest_label = unique_labels[np.argmax(counts)]
        inlier_indices = np.where(labels == largest_label)[0]
        clean_centers = detected_centers[inlier_indices]
        # 对应的边界点也过滤
        boundary_inliers = []
        for idx in inlier_indices:
            boundary_inliers.append(boundary_points[2 * idx])
            boundary_inliers.append(boundary_points[2 * idx + 1])
        boundary_points = np.array(boundary_inliers)
    else:
        clean_centers = detected_centers
        
    print(f"过滤后保留 {len(clean_centers)} 个轨迹点。")
    
    # 保存边界点以供可视化
    boundary_pcd = o3d.geometry.PointCloud()
    boundary_pcd.points = o3d.utility.Vector3dVector(boundary_points)
    boundary_output_path = os.path.join(output_dir, "weld_gap_points.ply")
    o3d.io.write_point_cloud(boundary_output_path, boundary_pcd)
    print(f"--> 已保存缝隙两侧边界点至: {boundary_output_path}")

    print("\n========== 5. 轨迹曲线样条平滑拟合 ==========")
    # 沿 Y 轴排序
    sort_idx = np.argsort(clean_centers[:, 1])
    clean_centers = clean_centers[sort_idx]
    
    # 参数化弧长
    diffs = np.diff(clean_centers, axis=0)
    arc_lengths = np.sqrt(np.sum(diffs**2, axis=1))
    t_vals = np.zeros(len(clean_centers))
    t_vals[1:] = np.cumsum(arc_lengths)
    t_vals = t_vals / t_vals[-1]
    
    # 三维样条平滑
    s_factor = 0.00002  # 平滑系数
    spline_x = UnivariateSpline(t_vals, clean_centers[:, 0], s=s_factor)
    spline_y = UnivariateSpline(t_vals, clean_centers[:, 1], s=s_factor)
    spline_z = UnivariateSpline(t_vals, clean_centers[:, 2], s=s_factor)
    
    t_dense = np.linspace(0, 1, 250)
    smooth_traj = np.column_stack((spline_x(t_dense), spline_y(t_dense), spline_z(t_dense)))
    print(f"成功平滑拟合包含 {len(smooth_traj)} 个三维坐标点的轨迹线。")
    
    # 保存坐标为 CSV
    csv_output_path = os.path.join(output_dir, "weld_trajectory.csv")
    np.savetxt(csv_output_path, smooth_traj, delimiter=",", header="x,y,z", comments="")
    print(f"--> 已保存中心轨迹坐标至: {csv_output_path}")
    
    # 保存折线 PLY 文件
    lines = [[i, i+1] for i in range(len(smooth_traj) - 1)]
    colors = [[1.0, 0.0, 0.0] for _ in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(smooth_traj)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    
    line_output_path = os.path.join(output_dir, "weld_trajectory_line.ply")
    o3d.io.write_line_set(line_output_path, line_set)
    print(f"--> 已保存三维折线 PLY 文件至: {line_output_path}")
    
    print("\n========== 6. 3D 渲染窗口展示 ==========")
    pcd_down.paint_uniform_color([0.8, 0.8, 0.8])  # 原始钢板染色为灰色
    boundary_pcd.paint_uniform_color([0.0, 0.5, 1.0])  # 缝隙两侧边界染色为蓝色
    
    # 轨迹点节点渲染为红球
    spheres = []
    for pt in smooth_traj[::5]:
        sp = o3d.geometry.TriangleMesh.create_sphere(radius=0.0025)
        sp.translate(pt)
        sp.paint_uniform_color([1.0, 0.1, 0.1])
        spheres.append(sp)
        
    print("正在打开 Open3D 渲染窗口...")
    o3d.visualization.draw_geometries([pcd_down, boundary_pcd, line_set] + spheres,
                                      window_name="Steel Plate Gap Center Tracking",
                                      width=1280, height=720)

if __name__ == "__main__":
    ply_file = r"c:\Users\Administrator\Desktop\group_165842847\焊缝数据.ply"
    if os.path.exists(ply_file):
        run_weld_seam_detection(ply_file)
    else:
        print(f"Error: 找不到点云文件: {ply_file}")
