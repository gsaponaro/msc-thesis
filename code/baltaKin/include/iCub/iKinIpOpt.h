/**
 * \defgroup iKinIpOpt iKinIpOpt 
 *  
 * @ingroup iKin 
 *
 * Classes for inverse kinematics of serial-links chains and 
 * iCub limbs based on IpOpt lib. To install IpOpt see
 * http://eris.liralab.it/wiki/Installing_IPOPT 
 *
 * Date: first release 23/06/2008
 *
 * Parameters:
 *
 * \see iKinIpOpt
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __IKINIPOPT_H__
#define __IKINIPOPT_H__

#include <iCub/iKinInv.h>
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>


namespace iKin
{
    class iKinIterateCallback;
    class iKin_NLP;
    class iKinIpOptMin;
}

/**
* \ingroup iKinIpOpt
*
* Class for defining iteration callback
*/
class iKin::iKinIterateCallback
{
private:
    // Copy constructor: not implemented.
    iKinIterateCallback(const iKinIterateCallback&);
    // Assignment operator: not implemented.
    iKinIterateCallback &operator=(const iKinIterateCallback&);

public:
    iKinIterateCallback() { }

    /**
    * Defines the callback body to be called at each iteration.
    * @param q current estimation of joint angles.
    */ 
    virtual void exec(Vector q) = 0;
};


/**
* \ingroup iKinIpOpt
*
* Class for defining IpOpt NLP problem
*/
class iKin::iKin_NLP : public Ipopt::TNLP
{
private:
    // Copy constructor: not implemented.
    iKin_NLP(const iKin_NLP&);
    // Assignment operator: not implemented.
    iKin_NLP &operator=(const iKin_NLP&);

protected:
    iKinChain &chain;
    iKinChain &chain2ndTask;

    unsigned int dim;
    unsigned int dim_2nd;
    unsigned int ctrlPose;

    Vector &xd;
    Vector &xd_2nd;
    Vector &qd_3rd;
    Vector  qd;
    Vector  q0;
    Vector  q;
    bool   *exhalt;

    Vector  e_xyz;
    Vector  e_ang;
    Vector  e_2nd;
    Vector  e_3rd;
    Matrix  J_xyz;
    Matrix  J_ang;
    Matrix  J_2nd;

    Vector *e_1st;
    Matrix *J_1st;

    Ipopt::Number __obj_scaling;
    Ipopt::Number __x_scaling;
    Ipopt::Number __g_scaling;

    iKinIterateCallback *callback;

    bool enable2ndTask;
    bool enable3rdTask;
    bool firstGo;

    virtual void computeQuantities(const Ipopt::Number *x);

public:
    /** default constructor */
    iKin_NLP(iKinChain &c, unsigned int _ctrlPose, const Vector &_q0, Vector &_xd,
             bool _enable2ndTask, iKinChain &_chain2ndTask, Vector &_xd_2nd,
             bool _enable3rdTask, Vector &_qd_3rd, bool *_exhalt=NULL);

    /** returns the solution */
    Vector get_qd() { return qd; }

    /** sets callback */
    void set_callback(iKinIterateCallback *_callback) { callback=_callback; }

    /** sets scaling factors */
    void set_scaling(Ipopt::Number _obj_scaling, Ipopt::Number _x_scaling, Ipopt::Number _g_scaling)
    {
        __obj_scaling=_obj_scaling;
        __x_scaling  =_x_scaling;
        __g_scaling  =_g_scaling;
    }

    /** default destructor */
    virtual ~iKin_NLP() { }

    /** Method to return some info about the nlp */
    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);
    
    /** Method to return the bounds for my problem */
    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);
    
    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                    bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                    Ipopt::Index m, bool init_lambda,
                                    Ipopt::Number* lambda);
    
    /** Method to return the objective value */
    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value);
    
    /** Method to return the gradient of the objective */
    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f);
    
    /** Method to return the constraint residuals */
    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g);
    
    /** Method to return:
    *   1) The structure of the jacobian (if "values" is NULL)
    *   2) The values of the jacobian (if "values" is not NULL)
    */
    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,Ipopt::Index m,
                            Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
                            Ipopt::Number* values);
    
    /** Method to return:
    *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
    *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
    */
    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                        Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                        bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                        Ipopt::Index* jCol, Ipopt::Number* values);

    /** overload this method to return scaling parameters. This is
    *  only called if the options are set to retrieve user scaling.
    *  There, use_x_scaling (or use_g_scaling) should get set to true
    *  only if the variables (or constraints) are to be scaled.  This
    *  method should return true only if the scaling parameters could
    *  be provided.
    */
    virtual bool get_scaling_parameters(Ipopt::Number& obj_scaling,
                                        bool& use_x_scaling, Ipopt::Index n, Ipopt::Number* x_scaling,
                                        bool& use_g_scaling, Ipopt::Index m, Ipopt::Number* g_scaling);

    /** This method is called once per iteration, after the iteration
     *  summary output has been printed.
     */
    virtual bool intermediate_callback(Ipopt::AlgorithmMode mode, Ipopt::Index iter, Ipopt::Number obj_value,
                                       Ipopt::Number inf_pr, Ipopt::Number inf_du, Ipopt::Number mu, Ipopt::Number d_norm,
                                       Ipopt::Number regularization_size, Ipopt::Number alpha_du, Ipopt::Number alpha_pr,
                                       Ipopt::Index ls_trials, const Ipopt::IpoptData* ip_data,
                                       Ipopt::IpoptCalculatedQuantities* ip_cq);
    
    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    virtual void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x,
                                   const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m,
                                   const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                   const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq);
};


/**
* \ingroup iKinIpOpt
*
* Class for inverting chain's kinematics based on IpOpt lib
*/
class iKin::iKinIpOptMin
{
private:
    // Default constructor: not implemented.
    iKinIpOptMin();
    // Copy constructor: not implemented.
    iKinIpOptMin(const iKinIpOptMin&);
    // Assignment operator: not implemented.
    iKinIpOptMin &operator=(const iKinIpOptMin&);

    // cannot be accessed from outside
    Ipopt::IpoptApplication *App;

protected:
    iKinChain &chain;
    iKinChain chain2ndTask;

    unsigned int ctrlPose;    

    Ipopt::Number obj_scaling;
    Ipopt::Number x_scaling;
    Ipopt::Number g_scaling;

    /**
    * Provides access to IpoptApplication's OptimizeTNLP method.
    */
    Ipopt::ApplicationReturnStatus optimize(const Ipopt::SmartPtr<Ipopt::TNLP>& tnlp);

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_ALL => complete pose control.
    *  IKINCTRL_POSE_XYZ => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG => rotational part of pose controlled. 
    * @param tol exits if norm(xd-x)<tol.
    * @param max_iter exits if iter>=max_iter (max_iter<0 disables
    *                 this check, IKINCTRL_DISABLED(==-1) by
    *                 default).
    * @param verbose is a integer number which progressively enables 
    *                different levels of warning messages or status
    *                dump. The larger this value the more detailed
    *                is the output (0=>off by default).
    * @param useHessian relies on exact Hessian computation or  
    *                enable Quasi-Newton approximation (true by
    *                default).
    */
    iKinIpOptMin(iKinChain &c, unsigned int _ctrlPose,
                 const double tol, const int max_iter=IKINCTRL_DISABLED,
                 const unsigned int verbose=0, bool useHessian=true);

    /**
    * Sets the state of Pose control settings.
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_ALL => complete pose control.
    *  IKINCTRL_POSE_XYZ => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG => rotational part of pose controlled.
    */
    void set_ctrlPose(unsigned int _ctrlPose);

    /**
    * Returns the state of Pose control settings.
    * @return Pose control settings.
    */
    unsigned int get_ctrlPose() { return ctrlPose; }

    /**
    * Selects 2nd Task End-Effector by giving the number n of joints
    * belonging to the secondary chain. 
    * @param n is the number of joints belonging to the secondary 
    *          chain.
    * @return a reference to the secondary chain. 
    */
    iKinChain &specify2ndTaskEndEff(unsigned int n);

    /**
    * Sets Tolerance.
    * @param tol exits if norm(xd-x)<tol.
    */
    void setTol(const double tol);

    /**
    * Sets Maximum Iteration.
    * @param max_iter exits if iter>=max_iter (max_iter<0 
    *                 (IKINCTRL_DISABLED) disables this check).
    */ 
    void setMaxIter(const int max_iter);

    /**
    * Sets Verbosity.
    * @param verbose is a integer number which progressively enables 
    *                different levels of warning messages or status
    *                dump. The larger this value the more detailed
    *                is the output.
    */
    void setVerbosity(const unsigned int verbose);

    /**
    * Selects whether to rely on exact Hessian computation or enable
    * Quasi-Newton approximation (Hessian is enabled at start-up by 
    * default). 
    * @param useHessian true if Hessian computation is enabled.
    */
    void setHessianOpt(const bool useHessian);

    /**
    * Enables/disables user scaling factors.
    * @param useUserScaling true if user scaling is enabled. 
    * @param obj_scaling user scaling factor for the objective 
    *                    function.
    * @param x_scaling user scaling factor for variables. 
    * @param g_scaling user scaling factor for constraints. 
    */
    void setUserScaling(const bool useUserScaling, Ipopt::Number _obj_scaling,
                        Ipopt::Number _x_scaling, Ipopt::Number _g_scaling);

    /**
    * Enable\disable derivative test at each call to solve method 
    * (disabled at start-up by default). Useful to check the 
    * derivatives implementation of NLP. 
    * @param enableTest true if derivative test shall be enabled. 
    * @param enable2ndDer true to enable second derivative test as 
    *                     well (false by default).
    */
    void setDerivativeTest(const bool enableTest, const bool enable2ndDer=false);

    /**
    * Executes the IpOpt algorithm trying to converge on target. 
    * @param q0 is the vector of initial joint angles values. 
    * @param xd is the End-Effector target Pose to be attained. 
    * @param enable2ndTask this switch if true enables the second 
    *                      task.
    * @param xd_2nd is the second target task traslational Pose to 
    *             be attained (typically the previous elbow xyz
    *             position).
    * @param enable3rdTask this switch if true enables the third 
    *                      task.
    * @param qd_3rd is the third task joint angles target 
    *             positions to be attained.
    * @param exit_code stores the exit code (NULL by default). Test 
    *                  for one of this:
    *                   SUCCESS
    *                   MAXITER_EXCEEDED
    *                   STOP_AT_TINY_STEP
    *                   STOP_AT_ACCEPTABLE_POINT
    *                   LOCAL_INFEASIBILITY
    *                   USER_REQUESTED_STOP
    *                   FEASIBLE_POINT_FOUND
    *                   DIVERGING_ITERATES
    *                   RESTORATION_FAILURE
    *                   ERROR_IN_STEP_COMPUTATION
    *                   INVALID_NUMBER_DETECTED
    *                   TOO_FEW_DEGREES_OF_FREEDOM
    *                   INTERNAL_ERROR
    * @param exhalt checks for an external request to exit (NULL by 
    *               default).
    * @param iterate pointer to a callback object (NULL by default).
    */
    virtual Vector solve(const Vector &q0, Vector &xd,
                         bool enable2ndTask, Vector &xd_2nd,
                         bool enable3rdTask, Vector &qd_3rd,
                         Ipopt::ApplicationReturnStatus *exit_code=NULL, bool *exhalt=NULL,
                         iKinIterateCallback *iterate=NULL);

    /**
    * Default destructor.
    */
    virtual ~iKinIpOptMin();
};



#endif


