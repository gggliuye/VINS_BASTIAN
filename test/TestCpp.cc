/*
* File:  TestCpp.cc
* Author:  BASTIAN
*
* Test functions for verify the efficience of my coding
*/

#include <iostream>

#include <Eigen/Sparse>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/plot.hpp>

#include "System.h"

void TestVectorCreation(int iCount=100000)
{
    std::cout << "Test the creation of vectors." << std::endl;
    TicToc t_o;
    for(int i = 0; i < iCount; i++){
        std::vector<cv::Point2f> vPoints;
    }
    printf(" --- costs: %fms\n", t_o.toc());
    TicToc t_i;
    for(int i = 0; i < iCount; i++){
        //std::vector<cv::Point2f> vPoints;
    }
    printf(" --- costs: %fms\n", t_i.toc());
    /*
    * Get result for iCount=100000
    *  --- costs: 0.000071ms
    *  --- costs: 0.000034ms
    *
    * the cost for create a empty vector is almost ignorable
    */
}

void TestVectorCreationN(int iCount=100000)
{
    std::cout << "Test the creation of vectors." << std::endl;
    TicToc t_o;
    std::vector<cv::Point2f> vFeatureVelocity;
    for(int i = 0; i < iCount; i++){
        vFeatureVelocity.push_back(cv::Point2f(0, 0));
    }
    printf(" --- costs: %fms\n", t_o.toc());
    TicToc t_i;
    std::vector<cv::Point2f> vFeatureVelocity2;
    vFeatureVelocity2 = std::vector<cv::Point2f>(iCount,cv::Point2f(0, 0));
    printf(" --- costs: %fms\n", t_i.toc());
    /*
    * Get result for iCount=100000
    * --- costs: 0.664955ms
    * --- costs: 0.225556ms
    */
}

///////////// Test CG / PCG / Direct Solver /////////////////

cv::Mat CvPlot(const std::vector<double> &vDataPlot)
{
    int n = vDataPlot.size();
    cv::Mat xData, yData, display;
    //std::cout << n << std::endl;
    xData.create(1, n, CV_64F);//1 Row, n columns, Double
    yData.create(1, n, CV_64F);
    for(int i = 0 ; i < n ; i++)
    {
        //std::cout << vDataPlot[i] << " ";
        xData.at<double>(i) = i;
        yData.at<double>(i) = vDataPlot[i];
    }
    cv::Ptr<cv::plot::Plot2d> plot = cv::plot::Plot2d::create( xData, yData );
    plot->setPlotSize(500, 200);
    //plot->setMaxX(10);
    //plot->setMinX(0);
    //plot->setMaxY(100);
    //plot->setMinY(-1);
    plot->render(display);

    return display;
}

///////////// CG //////////////
VecX CGbastian(const MatXX &Hpp, const VecX &bpp, double stop_threshold, int maxNum, std::vector<double> &vHistory)
{
    // from my matlab implementation
    // https://gitee.com/gggliuye/cg_pcg/blob/master/mine_cg.m
    // input should be a square matrix
    int n = Hpp.cols();
    VecX x(VecX::Zero(n));
    VecX r(bpp);

    double stop_rho = stop_threshold * sqrt(r.dot(r));

    double rho = r.dot(r);
    vHistory.clear();
    vHistory.push_back(rho);
    double last_rho = rho;

    VecX p(VecX::Zero(n));
    for(int k = 0 ; k < maxNum ; k ++){

        if(sqrt(rho) < stop_rho){
            break;
        }
        if(k == 0){
            p = r;
        } else {
            p = r + (rho/last_rho) * p;
        }
        VecX w = Hpp * p;
        double alpha = rho / (p.dot(w));
        x += alpha * p;
        r -= alpha * w;

        last_rho = rho;
        rho = r.dot(r);
        vHistory.push_back(rho);
    }

    return x;
}

///////////// PCG //////////////
VecX PCGbastian(const MatXX &Hpp, const VecX &bpp, double stop_threshold, int maxNum, std::vector<double> &vHistory)
{
    // from my matlab implementation
    // https://gitee.com/gggliuye/cg_pcg/blob/master/mine_pcg.m
    // input should be a square matrix
    int n = Hpp.cols();
    VecX x(VecX::Zero(n));
    VecX r(bpp);
    double stop_rho =  stop_threshold * sqrt(r.dot(r));
    stop_rho = stop_rho*stop_rho;

    // build the preconditonor, use sparse matrix to accelerate.
    SparseMatrix<double> M_inv_sparse(n, n);

    // build as diagonal matrix
    for(int i = 0 ; i < n; i++){
        M_inv_sparse.insert(i,i) = 1/Hpp(i,i);
    }

    // build with more knowledge about the matrix
    // which is a 171*171 matrix, with 171 = 6 + (6+9)*11
    /*
    for(int i = 0 ; i < 11; i++){
        int idx = 6 + 15*i;
        std::cout << idx << std::endl;
        MatXX mat_t = Hpp.block(idx,idx,6,6).inverse();
        std::cout << mat_t << std::endl;
        for(int j = 0 ; j < 6; j++){
            for(int k = 0; k < 6; k++){
                M_inv_sparse.insert(idx+j,idx+k) = mat_t(idx+j,idx+k);
            }
        }
    }
    */

    VecX z = M_inv_sparse * r;
    std::cout << z.transpose() << std::endl;
    double rho = r.dot(z);
    vHistory.clear();
    vHistory.push_back(rho);
    double last_rho = rho;

    VecX p(VecX::Zero(n));
    for(int k = 0 ; k < maxNum ; k ++){
        //std::cout << sqrt(rho) << " " << stop_rho << std::endl;
        if(rho < stop_rho){
            break;
        }
        if(k == 0){
            p = M_inv_sparse * r;
        } else {
            p = z + (rho/last_rho) * p;
        }
        VecX w = Hpp * p;
        double alpha = rho / (p.dot(w));
        x += alpha * p;
        r -= alpha * w;
        z = M_inv_sparse * r;
        last_rho = rho;
        rho = r.dot(z);
        vHistory.push_back(rho);
    }

    return x;
}

void TestLinearSolver(std::string data_file)
{
    ifstream fsProblem;
    fsProblem.open(data_file.c_str());
    if (!fsProblem.is_open()){
        cerr << "Failed to open imu data file! " << data_file << endl;
        return;
    }
    std::cout << "==> Read problem from " << data_file << std::endl;
    std::string s_problem_line;
    int rows; int cols;

    // get rows and cols
    {
        std::getline(fsProblem, s_problem_line);
        std::istringstream ss_problem_data(s_problem_line);
        ss_problem_data >> rows >> cols;
    }

    std::cout << "  --- Read H : " << rows << " * " << cols << std::endl;
    MatXX Hpp(MatXX::Zero(rows, cols));
    for(int i = 0 ; i < rows; i++){
        std::getline(fsProblem, s_problem_line);
        std::istringstream ss_problem_data(s_problem_line);
        for(int j = 0 ; j < cols; j++){
            ss_problem_data >> Hpp(i,j);
        }

    }
    //std::cout << Hpp.transpose();

    // get rows and cols
    {
        std::getline(fsProblem, s_problem_line);
        std::istringstream ss_problem_data(s_problem_line);
        ss_problem_data >> rows >> cols;
    }
    std::cout << "  --- Read b : " << rows << " * " << cols << std::endl;
    MatXX bpp(MatXX::Zero(rows, cols));
    for(int i = 0 ; i < rows; i++){
        std::getline(fsProblem, s_problem_line);
        std::istringstream ss_problem_data(s_problem_line);
        for(int j = 0 ; j < cols; j++){
            ss_problem_data >> bpp(i,j);
        }
    }
    //std::cout << bpp.transpose();
    std::cout << "==> Problem Read." << std::endl << std::endl;


    // solve directly
    //std::cout << "==> Solve Directly " << std::endl;
    TicToc t_linearsolver;
    VecX delta_x_pp =  Hpp.ldlt().solve(bpp);
    std::cout << "==> Solve Directly Cost: " << t_linearsolver.toc() << std::endl;

    TicToc t_linearsolver_cg;
    std::vector<double> vHistory_cg;
    VecX delta_x_pp_cg =  CGbastian(Hpp, bpp, 1e-6, Hpp.rows(), vHistory_cg);
    //cv::Mat plot_cg = CvPlot(vHistory_cg);
    std::cout << "==> Solve CG Cost: " << t_linearsolver_cg.toc() << std::endl;

    TicToc t_linearsolver_pcg;
    std::vector<double> vHistory_pcg;
    VecX delta_x_pp_pcg =  PCGbastian(Hpp, bpp, 1e-7, Hpp.rows(), vHistory_pcg);
    std::cout << "==> Solve PCG Cost: " << t_linearsolver_pcg.toc() << std::endl;

    cv::Mat plot_pcg = CvPlot(vHistory_pcg);
    //cv::imwrite("./plot_pcg.jpg", plot_pcg);
    cv::imshow("plot_pcg", plot_pcg);
    cv::waitKey();

    /*
    * ==> Solve Directly Cost: 0.425073
    * ==> Solve CG Cost: 1.12453
    * ==> Solve PCG Cost: 0.327722
    */


}




int main(int argc, char* argv[]) {
    std::cout << "Test for development of the functions in this project." << std::endl;

    //TestVectorCreationN();

    TestLinearSolver("./matrix_40.txt");

}
