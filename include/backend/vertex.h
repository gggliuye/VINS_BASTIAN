#ifndef MYSLAM_BACKEND_VERTEX_H
#define MYSLAM_BACKEND_VERTEX_H

#include "eigen_types.h"

namespace myslam {
namespace backend {
extern unsigned long global_vertex_id;

/**
 * @brief vertex for the problem
 */
class Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * @param num_dimension
     * @param local_dimension : if -1 , will be the same as num_dimension
     */
    explicit Vertex(int num_dimension, int local_dimension = -1);

    virtual ~Vertex();

    /// get the dimension
    int Dimension() const;

    /// get the local dimension
    int LocalDimension() const;

    /// get vertex id
    unsigned long Id() const { return id_; }

    /// get all the parameters
    VecX Parameters() const { return parameters_; }

    /// get a reference of the parameters
    VecX &Parameters() { return parameters_; }

    /// set the parameters value
    void SetParameters(const VecX &params) { parameters_ = params; }

    // backup and rollback to pervious values,
    // for the case, when the current update is relative bad.
    void BackUpParameters() { parameters_backup_ = parameters_; }
    void RollBackParameters() { parameters_ = parameters_backup_; }

    /// add the changement to parameters
    virtual void Plus(const VecX &delta);

    /// the name of the vertex
    virtual std::string TypeInfo() const = 0;
    virtual VertexEdgeTypes TypeId() const = 0;

    int OrderingId() const { return ordering_id_; }

    void SetOrderingId(unsigned long id) { ordering_id_ = id; };

    /// set this vertex to be fixed
    void SetFixed(bool fixed = true) {
        fixed_ = fixed;
    }

    /// check whether it is fixed
    bool IsFixed() const { return fixed_; }

protected:
    VecX parameters_;
    VecX parameters_backup_;
    int local_dimension_;
    unsigned long id_;  // vertex id, will be initialized automaticly

    /// ordering id是在problem中排序后的id，用于寻找雅可比对应块
    /// ordering id带有维度信息，例如ordering_id=6则对应Hessian中的第6列
    /// start from zero
    unsigned long ordering_id_ = 0;

    bool fixed_ = false;
};

}
}

#endif
