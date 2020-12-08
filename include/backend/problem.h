#ifndef MYSLAM_BACKEND_PROBLEM_H
#define MYSLAM_BACKEND_PROBLEM_H

#include <unordered_map>
#include <map>
#include <memory>

#include <Eigen/Sparse>

#include "eigen_types.h"
#include "edge.h"
#include "backend/edge_reprojection.h"
#include "vertex.h"



typedef unsigned long ulong;

namespace myslam {
namespace backend {

typedef unsigned long ulong;
//    typedef std::unordered_map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
typedef std::map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
typedef std::unordered_map<unsigned long, std::shared_ptr<Edge>> HashEdge;
typedef std::unordered_multimap<unsigned long, std::shared_ptr<Edge>> HashVertexIdToEdge;

class Problem {
public:

    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *
     * seperate SLAM problem and general problem
     *
     *   -> if it is SLAM problem, pose and landmark are seperated and Hessian saved as sparse matrix
     *   -> and SLAM problem only accept certain vertex and edeg
     *
     *   -> if general problem, Hessian is dense as default, unless some vertex be set marginalized
     *
     */
    enum class ProblemType {
        SLAM_PROBLEM,
        GENERIC_PROBLEM
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Problem(ProblemType problemType);

    ~Problem();

    bool AddVertex(std::shared_ptr<Vertex> vertex);

    /**
     * remove a vertex
     * @param vertex_to_remove
     */
    bool RemoveVertex(std::shared_ptr<Vertex> vertex);

    bool AddEdge(std::shared_ptr<Edge> edge);

    bool RemoveEdge(std::shared_ptr<Edge> edge);

    /**
     * get the outliers judged by the optimzation
     * for the front end to delete it
     * @param outlier_edges
     */
    void GetOutlierEdges(std::vector<std::shared_ptr<Edge>> &outlier_edges);

    /**
     * solve the problem
     * @param iterations
     * @return
     */
    bool Solve(int iterations = 10);

    // marginalize a frame and the landmark "owned" by it (the landmarks whose host is this frame)
    bool Marginalize(std::shared_ptr<Vertex> frameVertex,
                     const std::vector<std::shared_ptr<Vertex>> &landmarkVerticies);

    bool Marginalize(const std::shared_ptr<Vertex> frameVertex);
    bool Marginalize(const std::vector<std::shared_ptr<Vertex> > frameVertex,int pose_dim);

    MatXX GetHessianPrior(){ return H_prior_;}
    VecX GetbPrior(){ return b_prior_;}
    VecX GetErrPrior(){ return err_prior_;}
    MatXX GetJtPrior(){ return Jt_prior_inv_;}

    void SetHessianPrior(const MatXX& H){H_prior_ = H;}
    void SetbPrior(const VecX& b){b_prior_ = b;}
    void SetErrPrior(const VecX& b){err_prior_ = b;}
    void SetJtPrior(const MatXX& J){Jt_prior_inv_ = J;}

    void ExtendHessiansPriorSize(int dim);

    //test compute prior
    void TestComputePrior();

private:

    /// Solve general probelm
    bool SolveGenericProblem(int iterations);

    /// Solve SLAM problem
    bool SolveSLAMProblem(int iterations);

    /// set the order of every index : ordering_index
    void SetOrdering();

    /// set ordering for new vertex in slam problem
    void AddOrderingSLAM(std::shared_ptr<Vertex> v);

    /// build the whole hessian matrix
    void MakeHessian();

    /// schur to solve SBA problem
    void SchurSBA();

    /// solve the linea problem
    bool SolveLinearSystem();

    /// update the states
    void UpdateStates();

    // sometimes the residual become larger after update -> bad update
    // require to roll back the former state
    void RollbackStates();

    /// compute and update the prior part
    void ComputePrior();

    /// whether a vertex is a pose
    bool IsPoseVertex(std::shared_ptr<Vertex> v);

    /// whether a vertex is a landmark
    bool IsLandmarkVertex(std::shared_ptr<Vertex> v);

    /// after adding a new vertex, the size changed of Hessian matrix
    void ResizePoseHessiansWhenAddingPose(std::shared_ptr<Vertex> v);

    /// check the state order
    bool CheckOrdering();

    void LogoutVectorSize();

    /// get the edges connect to a vertex
    std::vector<std::shared_ptr<Edge>> GetConnectedEdges(std::shared_ptr<Vertex> vertex);

    /// Levenberg

    /// initialize the Lambda parameter
    void ComputeLambdaInitLM();

    /// Hessian add/substract  Lambda in the dinagal elements of Hessian
    void AddLambdatoHessianLM();

    void RemoveLambdaHessianLM();

    /// LM whether the lambda is good and how to update it
    bool IsGoodStepInLM();

    /// PCG iteration solver
    VecX PCGSolver(const MatXX &Hpp, const VecX &bpp, int maxIter, double stop_threshold);

public:

    bool verbose = true;

    // for save marginalized points
    bool ifRecordMap = true;
    std::vector<Vec3> marginalized_pts;
    std::vector<Vec3> current_pts;

private:

    double currentLambda_;
    double currentChi_;
    double dCurrenNormDx_ = 0;
    double stopThresholdLM_;    // LM threshold to quit
    double ni_;                 // the control the change of lambda

    ProblemType problemType_;

    /// the whole hessian
    MatXX Hessian_;
    VecX b_;
    VecX delta_x_;

    /// prior part of the system
    MatXX H_prior_;
    VecX b_prior_;
    VecX b_prior_backup_;
    VecX err_prior_backup_;

    MatXX Jt_prior_inv_;
    VecX err_prior_;

    /// pose of SBA problem
    MatXX H_pp_schur_;
    VecX b_pp_schur_;

    // landmark(ll) and pose(pp) of hessian
    MatXX H_pp_;
    VecX b_pp_;
    MatXX H_ll_;
    VecX b_ll_;

    /// all vertices
    HashVertex verticies_;

    /// all edges
    HashEdge edges_;

    /// find edge by vertex id
    HashVertexIdToEdge vertexToEdge_;

    /// Ordering related
    ulong ordering_poses_ = 0;
    ulong ordering_landmarks_ = 0;
    ulong ordering_generic_ = 0;
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_pose_vertices_;        // order the pose vertex
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_landmark_vertices_;    // order the landmark vertex

    // verticies need to marg. <Ordering_id_, Vertex>
    HashVertex verticies_marg_;

    bool bDebug = false;
    double t_hessian_cost_ = 0.0;
    double t_PCGsovle_cost_ = 0.0;
};

}
}

#endif
