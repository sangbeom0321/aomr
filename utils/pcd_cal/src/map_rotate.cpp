#include <memory>
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

class PCDRotator {
public:
    PCDRotator() {
        input_pcd_path = "/root/map/real_map/GlobalMap.pcd";  // 입력 PCD 파일 경로
        output_base_path = "/root/map/rotated_map/GlobalMap";  // 출력 PCD 파일 기본 경로
    }

    void rotate_and_save() {
        // 원본 포인트 클라우드 로드
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_path, *cloud) == -1) {
            std::cerr << "Failed to load the PCD file." << std::endl;
            return;
        }
        std::cout << "Loaded " << cloud->size() << " points from " << input_pcd_path << std::endl;

        // 30도씩 6번 회전 (0, 30, 60, 90, 120, 150도)
        for (int i = 0; i < 6; i++) {
            float angle = i * 30.0f * M_PI / 180.0f;  // 각도를 라디안으로 변환
            
            // 변환 행렬 생성
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));  // Z축 기준 회전

            // 포인트 클라우드 회전
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

            // 파일 이름 생성 (예: GlobalMap_30.pcd, GlobalMap_60.pcd, ...)
            std::string output_path = output_base_path + "_" + std::to_string(i * 30) + ".pcd";

            // 회전된 포인트 클라우드 저장
            if (pcl::io::savePCDFileBinary(output_path, *transformed_cloud) == -1) {
                std::cerr << "Failed to save " << output_path << std::endl;
            } else {
                std::cout << "Saved rotated point cloud to " << output_path << std::endl;
            }
        }
    }

private:
    std::string input_pcd_path;
    std::string output_base_path;
};

int main(int argc, char *argv[]) {
    PCDRotator rotator;
    rotator.rotate_and_save();
    return 0;
}
