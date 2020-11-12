#include <string>
#include "utility/utility.h"

Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}


int Utility::iCountSave = 0;
void Utility::SaveMatrixToFile(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &mat,
                               Eigen::Matrix<double, Eigen::Dynamic, 1> &vec)
{
    std::string save_file = "./matrix_" + std::to_string(iCountSave++) + ".txt";
    std::ofstream ofs_matrix;
    ofs_matrix.open(save_file,std::fstream::app | std::fstream::out);
    ofs_matrix << std::fixed;
    ofs_matrix << mat.rows() << " " << mat.cols() << std::endl;

    for(int i = 0 ; i < mat.rows(); i++){
        for(int j = 0 ; j < mat.cols(); j++){
            ofs_matrix << mat(i,j) << " ";
        }
        ofs_matrix << std::endl;
    }

    ofs_matrix << vec.rows() << " " << vec.cols() << std::endl;

    for(int i = 0 ; i < vec.rows(); i++){
        for(int j = 0 ; j < vec.cols(); j++){
            ofs_matrix << vec(i,j) << " ";
        }
        ofs_matrix << std::endl;
    }

    ofs_matrix.close();
}

bool Utility::ofs_time_init = false;
std::ofstream Utility::ofs_time;
void Utility::WriteTimeConsume(double time_solve, double time_hessian)
{
    if(!ofs_time_init){
        std::string save_file = "./time.txt";
        ofs_time.open(save_file,std::fstream::app | std::fstream::out);
        ofs_time_init = true;
    }

    ofs_time << time_solve << " " << time_hessian << std::endl;

}
