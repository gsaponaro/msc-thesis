
#include <iostream>
#include <iomanip>

#include <iCub/iKinIpOpt.h>

using namespace iKin;
using namespace Ipopt;


/************************************************************************/
iKin_NLP::iKin_NLP(iKinChain &c, unsigned int _ctrlPose, const yarp::sig::Vector &_q0, yarp::sig::Vector &_xd,
                   bool _enable2ndTask, iKinChain &_chain2ndTask, yarp::sig::Vector &_xd_2nd,
                   bool _enable3rdTask, yarp::sig::Vector &_qd_3rd,
                   bool *_exhalt) :
                   chain(c), q0(_q0), xd(_xd),
                   chain2ndTask(_chain2ndTask), xd_2nd(_xd_2nd),
                   enable3rdTask(_enable3rdTask), qd_3rd(_qd_3rd),
                   exhalt(_exhalt)
{
    dim=chain.getDOF();
    dim_2nd=chain2ndTask.getDOF();

    ctrlPose=_ctrlPose;

    if (ctrlPose>IKINCTRL_POSE_ANG)
        ctrlPose=IKINCTRL_POSE_ANG;

    enable2ndTask=_enable2ndTask && (dim_2nd>0);

    qd.resize(dim);

    size_t n=q0.length();
    n=n>dim ? dim : n;

    unsigned int i;
    for (i=0; i<n; i++)
        qd[i]=q0[i];

    for (; i<dim; i++)
        qd[i]=0.0;

    q=qd;

    e_xyz.resize(3);   e_xyz=0.0;
    e_ang.resize(3);   e_ang=0.0;
    e_2nd.resize(3);   e_2nd=0.0;
    e_3rd.resize(dim); e_3rd=0.0;

    J_xyz.resize(3,dim); J_xyz.zero();
    J_ang.resize(3,dim); J_ang.zero();
    J_2nd.resize(3,dim); J_2nd.zero();

    if (ctrlPose==IKINCTRL_POSE_XYZ)
    {
        e_1st=&e_xyz;
        J_1st=&J_xyz;
    }
    else
    {
        e_1st=&e_ang;
        J_1st=&J_ang;
    }

    firstGo=true;

    __obj_scaling=1.0;
    __x_scaling  =1.0;
    __g_scaling  =1.0;

    callback=NULL;
}


/************************************************************************/
void iKin_NLP::computeQuantities(const Number *x)
{
    yarp::sig::Vector new_q(dim);

    for (Index i=0; i<(int)dim; i++)
        new_q[i]=x[i];

    if (!(q==new_q) || firstGo)
    {
        firstGo=false;
        q=new_q;

        yarp::sig::Vector v(4);
        v[0]=xd[3];
        v[1]=xd[4];
        v[2]=xd[5];
        v[3]=xd[6];
        yarp::sig::Matrix Des=axis2dcm(v);
        Des(0,3)=xd[0];
        Des(1,3)=xd[1];
        Des(2,3)=xd[2];
    
        q=chain.setAng(q);
        yarp::sig::Matrix H=chain.getH();
        yarp::sig::Matrix E=Des*pinv(H);
        v=dcm2axis(E);
        
        e_xyz[0]=xd[0]-H(0,3);
        e_xyz[1]=xd[1]-H(1,3);
        e_xyz[2]=xd[2]-H(2,3);
        e_ang[0]=v[3]*v[0];
        e_ang[1]=v[3]*v[1];
        e_ang[2]=v[3]*v[2];

        yarp::sig::Matrix J=chain.GeoJacobian();
        submatrix(J,J_xyz,0,2,0,dim-1);
        submatrix(J,J_ang,3,5,0,dim-1);

        if (enable2ndTask)
        {
            yarp::sig::Matrix H_2nd=chain2ndTask.getH();
            e_2nd[0]=xd_2nd[0]-H_2nd(0,3);
            e_2nd[1]=xd_2nd[1]-H_2nd(1,3);
            e_2nd[2]=xd_2nd[2]-H_2nd(2,3);

            submatrix(chain2ndTask.GeoJacobian(),J_2nd,0,2,0,dim_2nd-1);
        }

        if (enable3rdTask)
            for (unsigned int i=0; i<dim; i++)
                e_3rd[i]=qd_3rd[i]-q[i];
    }
}


/************************************************************************/
bool iKin_NLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n=dim;
    
    if (ctrlPose==IKINCTRL_POSE_ALL)
    {
        m=1;
        nnz_jac_g=dim;
    }
    else
    {
        m=0;
        nnz_jac_g=0;
    }

    nnz_h_lag=(dim*(dim+1))>>1;
    
    index_style=TNLP::C_STYLE;
    
    return true;
}


/************************************************************************/
bool iKin_NLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
                               Index m, Number* g_l, Number* g_u)
{
    for (Index i=0; i<(int)dim; i++)
    {
        x_l[i]=chain(i).getMin();
        x_u[i]=chain(i).getMax();
    }
    
    if (m)
    {
        g_l[0]=0.0;
        g_u[0]=1e-5;
    }

    return true;
}


/************************************************************************/
bool iKin_NLP::get_starting_point(Index n, bool init_x, Number* x,
                                  bool init_z, Number* z_L, Number* z_U,
                                  Index m, bool init_lambda,
                                  Number* lambda)
{
    for (Index i=0; i<(int)dim; i++)
        x[i]=q0[i];

    return true;
}


/************************************************************************/
bool iKin_NLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
    computeQuantities(x);

    obj_value=0.5*norm2(*e_1st);

    if (enable2ndTask)
        obj_value+=0.5*norm2(e_2nd);

    if (enable3rdTask)
        obj_value+=0.5*norm2(e_3rd);

    return true;
}


/************************************************************************/
bool iKin_NLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
    computeQuantities(x);

    yarp::sig::Vector grad=-1.0*(J_1st->transposed() * *e_1st);

    if (enable2ndTask)
        grad=grad-1.0*(J_2nd.transposed()*e_2nd);

    if (enable3rdTask)
        grad=grad-1.0*e_3rd;

    for (Index i=0; i<(int)dim; i++)
        grad_f[i]=grad[i];

    return true;
}


/************************************************************************/
bool iKin_NLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
    computeQuantities(x);

    g[0]=0.5*norm2(e_xyz);

    return true;
}


/************************************************************************/
bool iKin_NLP::eval_jac_g(Index n, const Number* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          Number* values)
{
    if (m)
    {
        if (!values)
        {
            Index idx=0;
    
            for (Index col=0; col<(int)dim; col++)
            {
                iRow[idx]=0;
                jCol[idx]=col;
                idx++;
            }
        }
        else
        {
            computeQuantities(x);
        
            yarp::sig::Vector grad=-1.0*(J_xyz.transposed()*e_xyz);
        
            for (Index i=0; i<(int)dim; i++)
                values[i]=grad[i];
        }
    }

    return true;
}


/************************************************************************/
bool iKin_NLP::eval_h(Index n, const Number* x, bool new_x,
                      Number obj_factor, Index m, const Number* lambda,
                      bool new_lambda, Index nele_hess, Index* iRow,
                      Index* jCol, Number* values)
{
    if (!values)
    {
        Index idx=0;
    
        for (Index row=0; row<(int)dim; row++)
            for (Index col=0; col<=row; col++)
            {
                iRow[idx]=row;
                jCol[idx]=col;
                idx++;
            }
    }
    else
    {
        // Given the task: min f(q)=1/2*||xd-F(q)||^2
        // the Hessian Hij is: <dF/dqi,dF/dqj> - <d2F/dqidqj,e>

        computeQuantities(x);

        chain.prepareForHessian();

        if (enable2ndTask)
            chain2ndTask.prepareForHessian();

        Index idx=0;

        for (Index row=0; row<(int)dim; row++)
            for (Index col=0; col<=row; col++)
            {
                yarp::sig::Vector h=chain.fastHessian_ij(row,col);
                yarp::sig::Vector h_xyz(3), h_ang(3);
                h_xyz[0]=h[0];
                h_xyz[1]=h[1];
                h_xyz[2]=h[2];
                h_ang[0]=h[3];
                h_ang[1]=h[4];
                h_ang[2]=h[5];

                yarp::sig::Vector *h_1st;
                if (ctrlPose==IKINCTRL_POSE_XYZ)
                    h_1st=&h_xyz;
                else
                    h_1st=&h_ang;

                values[idx]=obj_factor*(dot(J_1st->getCol(row),J_1st->getCol(col))-dot(*h_1st,*e_1st));

                if (m)
                    values[idx]+=lambda[0]*(dot(J_xyz.getCol(row),J_xyz.getCol(col))-dot(h_xyz,e_xyz));

                if (enable2ndTask && row<(int)dim_2nd && col<(int)dim_2nd)
                {    
                    yarp::sig::Vector h2=chain2ndTask.fastHessian_ij(row,col);
                    yarp::sig::Vector h_2nd(3);
                    h_2nd[0]=h2[0];
                    h_2nd[1]=h2[1];
                    h_2nd[2]=h2[2];

                    values[idx]+=obj_factor*(dot(J_2nd.getCol(row),J_2nd.getCol(col))-dot(h_2nd,e_2nd));
                }

                idx++;
            }
    }
    
    return true;
}


/************************************************************************/
bool iKin_NLP::get_scaling_parameters(Number& obj_scaling,
                                      bool& use_x_scaling, Index n, Number* x_scaling,
                                      bool& use_g_scaling, Index m, Number* g_scaling)
{
    obj_scaling=__obj_scaling;

    for (Index i=0; i<(int)n; i++)
        x_scaling[i]=__x_scaling;

    for (Index j=0; j<(int)m; j++)
        g_scaling[j]=__g_scaling;

    use_x_scaling=use_g_scaling=true;

    return true;
}


/************************************************************************/
bool iKin_NLP::intermediate_callback(AlgorithmMode mode, Index iter, Number obj_value,
                                     Number inf_pr, Number inf_du, Number mu, Number d_norm,
                                     Number regularization_size, Number alpha_du, Number alpha_pr,
                                     Index ls_trials, const IpoptData* ip_data,
                                     IpoptCalculatedQuantities* ip_cq)
{
    if (callback)
        callback->exec(q);

    if (exhalt)
        return !(*exhalt);
    else
        return true;
}


/************************************************************************/
void iKin_NLP::finalize_solution(SolverReturn status,
                                 Index n, const Number* x, const Number* z_L, const Number* z_U,
                                 Index m, const Number* g, const Number* lambda,
                                 Number obj_value,
                                 const IpoptData* ip_data,
                                 IpoptCalculatedQuantities* ip_cq)
{
    for (Index i=0; i<n; i++)
        qd[i]=x[i];

    qd=chain.setAng(qd);
}


/************************************************************************/
iKinIpOptMin::iKinIpOptMin(iKinChain &c, unsigned int _ctrlPose, const double tol, const int max_iter,
                           const unsigned int verbose, bool useHessian) : chain(c)
{
    ctrlPose=_ctrlPose;

    if (ctrlPose>IKINCTRL_POSE_ANG)
        ctrlPose=IKINCTRL_POSE_ANG;

    chain.setAllConstraints(false); // this is required since IpOpt initially relaxes constraints

    App=new IpoptApplication();

    App->Options()->SetNumericValue("tol",tol);
    App->Options()->SetNumericValue("acceptable_tol",tol);
    App->Options()->SetIntegerValue("acceptable_iter",10);
    App->Options()->SetStringValue("mu_strategy","adaptive");
    App->Options()->SetIntegerValue("print_level",verbose);    

    if (max_iter>0)
        App->Options()->SetIntegerValue("max_iter",max_iter);
    else
        App->Options()->SetIntegerValue("max_iter",(Index)2e9);

    if (!useHessian)
        App->Options()->SetStringValue("hessian_approximation","limited-memory");

    App->Initialize();
}


/************************************************************************/
void iKinIpOptMin::set_ctrlPose(unsigned int _ctrlPose)
{
    ctrlPose=_ctrlPose;

    if (ctrlPose>IKINCTRL_POSE_ANG)
        ctrlPose=IKINCTRL_POSE_ANG;
}


/************************************************************************/
iKinChain &iKinIpOptMin::specify2ndTaskEndEff(unsigned int n)
{
    chain2ndTask.clear();
    chain2ndTask.setH0(chain.getH0());

    for (unsigned int i=0; i<n; i++)
        chain2ndTask << chain[i];

    return chain2ndTask;
}


/************************************************************************/
void iKinIpOptMin::setTol(const double tol)
{
    App->Options()->SetNumericValue("tol",tol);
    App->Options()->SetNumericValue("acceptable_tol",tol);

    App->Initialize();
}


/************************************************************************/
void iKinIpOptMin::setMaxIter(const int max_iter)
{
    if (max_iter>0)
        App->Options()->SetIntegerValue("max_iter",max_iter);
    else
        App->Options()->SetIntegerValue("max_iter",(Index)2e9);

    App->Initialize();
}


/************************************************************************/
void iKinIpOptMin::setVerbosity(const unsigned int verbose)
{
    App->Options()->SetIntegerValue("print_level",verbose);

    App->Initialize();
}


/************************************************************************/
void iKinIpOptMin::setHessianOpt(const bool useHessian)
{
    if (useHessian)
        App->Options()->SetStringValue("hessian_approximation","exact");
    else
        App->Options()->SetStringValue("hessian_approximation","limited-memory");

    App->Initialize();
}


/************************************************************************/
void iKinIpOptMin::setUserScaling(const bool useUserScaling, Ipopt::Number _obj_scaling,
                                  Ipopt::Number _x_scaling, Ipopt::Number _g_scaling)
{
    if (useUserScaling)
    {
        obj_scaling=_obj_scaling;
        x_scaling  =_x_scaling;
        g_scaling  =_g_scaling;

        App->Options()->SetStringValue("nlp_scaling_method","user-scaling");
    }
    else
        App->Options()->SetStringValue("nlp_scaling_method","gradient-based");

    App->Initialize();
}


/************************************************************************/
void iKinIpOptMin::setDerivativeTest(const bool enableTest, const bool enable2ndDer)
{
    if (enableTest)
    {
        if (enable2ndDer)
            App->Options()->SetStringValue("derivative_test","second-order");
        else
            App->Options()->SetStringValue("derivative_test","first-order");

        App->Options()->SetStringValue("derivative_test_print_all","yes");
    }
    else
        App->Options()->SetStringValue("derivative_test","none");

    App->Initialize();
}


/************************************************************************/
yarp::sig::Vector iKinIpOptMin::solve(const yarp::sig::Vector &q0, yarp::sig::Vector &xd,
                                      bool enable2ndTask, yarp::sig::Vector &xd_2nd,
                                      bool enable3rdTask, yarp::sig::Vector &qd_3rd,
                                      ApplicationReturnStatus *exit_code, bool *exhalt,
                                      iKinIterateCallback *iterate)
{
    SmartPtr<iKin_NLP> nlp=new iKin_NLP(chain,ctrlPose,q0,xd,
                                        enable2ndTask,chain2ndTask,xd_2nd,
                                        enable3rdTask,qd_3rd,exhalt);

    nlp->set_scaling(obj_scaling,x_scaling,g_scaling);
    nlp->set_callback(iterate);

    ApplicationReturnStatus status=App->OptimizeTNLP(GetRawPtr(nlp));

    if (exit_code)
        *exit_code=status;

    return nlp->get_qd();
}


/************************************************************************/
ApplicationReturnStatus iKinIpOptMin::optimize(const SmartPtr<TNLP>& tnlp)
{
    return App->OptimizeTNLP(tnlp);
}


/************************************************************************/
iKinIpOptMin::~iKinIpOptMin()
{
    delete App;
}
