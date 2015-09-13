/**
 * \defgroup iKinInv iKinInv
 *  
 * @ingroup iKin 
 *
 * Classes for inverse kinematics of serial-links chains and 
 * iCub limbs 
 *
 * Date: first release 16/06/2008
 *
 * Parameters:
 *
 * \see iKinInv
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __IKININV_H__
#define __IKININV_H__

#include <gsl/gsl_multimin.h>
#include <yarp/math/SVD.h>

#include <iCub/iKinFwd.h>

#define IKINCTRL_STATE_RUNNING      0
#define IKINCTRL_STATE_INTARGET     1
#define IKINCTRL_STATE_DEADLOCK     2

#define IKINCTRL_POSE_ALL           0
#define IKINCTRL_POSE_XYZ           1
#define IKINCTRL_POSE_ANG           2

#define IKINCTRL_STEEP_JT           0
#define IKINCTRL_STEEP_PINV         1

#define IKINCTRL_RET_TOLX           0
#define IKINCTRL_RET_TOLSIZE        1
#define IKINCTRL_RET_TOLQ           2
#define IKINCTRL_RET_MAXITER        3
#define IKINCTRL_RET_EXHALT         4
                                    
#define GSLALGOTYPE_STEEPEST        0
#define GSLALGOTYPE_CONJ_FR         1
#define GSLALGOTYPE_CONJ_PR         2
#define GSLALGOTYPE_BFGS            3
#define GSLALGOTYPE_NMSIMPLEX       4

#define IKINCTRL_DISABLED           -1


namespace iKin
{
    class Integrator;
    class iKinCtrl;
    class SteepCtrl;
    class VarKpSteepCtrl;
    class LMCtrl;
    class LMCtrl_GPM;
    class GSLMinCtrl;
    class MultiRefDynCtrl;
}


/**
* \ingroup iKinInv
*
* A class for defining a saturated integrator based on Tustin 
* formula: 1/s => Ts/2*(z+1)/(z-1)
*/
class iKin::Integrator
{
private:
    unsigned int dim;
    Vector y;
    Vector x_old;
    Matrix lim;
    double Ts;
    bool   applySat;

    // Default constructor: not implemented.
    Integrator();

    void   _allocate(const Integrator &I);
    Vector saturate(const Vector &v);

public:
    /**
    * Constructor. 
    * @param _Ts is the integrator sampling time.
    * @param y0 is the initial value of the output vector.
    * @param _lim is a Nx2 matrix describing for each row i the 
    *             lower (1st column) and the upper limit (2nd
    *             column) of the ith component:
    *             _lim(i,1)<=output[i]<=_lim(i,2)
    */
    Integrator(double _Ts, const Vector &y0, const Matrix &_lim);

    /**
    * Creates a new Integrator from an already existing object.
    * @param I is the Integrator to be copied.
    */
    Integrator(const Integrator &I) { _allocate(I); }

    /**
    * Copies a Integrator object into the current one.
    * @param I is a reference to an object of type Integrator.
    * @return a reference to the current object.
    */
    Integrator &operator=(const Integrator &I) { _allocate(I); return *this; }

    /**
    * Executes one-step integration of input vector. 
    * To be called each Ts seconds.
    * @param x is the input vector to be integrated.
    * @return the current output vector.
    */
    Vector integrate(const Vector &x);

    /**
    * Sets the saturation status.
    * @param _applySat if true then the saturation is applied 
    *                  (initialized as true).
    */
    void setSaturation(bool _applySat) { applySat=_applySat; }

    /**
    * Returns the current saturation status. 
    * @return current saturation status.
    */
    bool getSaturation() { return applySat; }

    /**
    * Sets the output vector constraints matrix. 
    * @param _lim is the constraints matrix.
    */
    void setLim(const Matrix &_lim) { lim=_lim; }

    /**
    * Returns the constraints matrix. 
    * @return the constraints matrix.
    */
    Matrix getLim() { return lim; }

    /**
    * Resets the output vector. 
    * @param y0 is new value of output vector.
    */
    void reset(const Vector &y0) { y=saturate(y0); x_old=0.0; }

    /**
    * Returns the current output vector. 
    * @return the current output vector.
    */
    Vector get() { return y; }
};


/**
* \ingroup iKinInv
*
* Abstract class for inverting chain's kinematics.
*/
class iKin::iKinCtrl
{
private:
    // Default constructor: not implemented.
    iKinCtrl();
    // Copy constructor: not implemented.
    iKinCtrl(const iKinCtrl&);
    // Assignment operator: not implemented.
    iKinCtrl &operator=(const iKinCtrl&);

protected:
    iKinChain &chain;
    unsigned int ctrlPose;

    Vector x_set;
    Vector x;
    Vector e;
    Vector q;
    Matrix J;
    Matrix Jt;
    Matrix pinvJ;
    Vector grad;

    Vector q_old;

    double inTargetTol;
    double watchDogTol;

    unsigned int dim;
    unsigned int iter;

    int  State;

    bool watchDogOn;
    int  watchDogCnt;
    int  watchDogMaxIter;

    /**
    * Computes the error according to the current controller
    * settings (complete pose/translational/rotational part). 
    * Note that x must be previously set.
    * @return the error.
    */
    virtual Vector calc_e();

    /**
    * Updates the control state.
    */
    virtual void updateState();

    /**
    * Handles the watchDog.
    */
    virtual void watchDog();

    /**
    * Checks each joint velocity and sets it to zero if it steers 
    * the joint out of range. 
    * @param _qdot is the joint velocities vector to be checked. 
    * @param _Ts is the joint velocities sampling time.
    * @return the new velocity. 
    */
    virtual Vector checkVelocity(const Vector &_qdot, double _Ts);

    /**
    * Method called whenever in target. 
    * Shall be implemented. 
    */
    virtual void inTargetFcn() = 0;

    /**
    * Method called whenever the watchDog is triggered. Put here the
    * code to recover from deadLock. Shall be implemented. 
    */
    virtual void deadLockRecoveryFcn() = 0;

    /**
    * Dumps warning or status messages.
    * @param verbose is a integer whose 32 bits are intended as
    *                follows. The lowest word (16 bits)
    *                progressively enables different levels of
    *                warning messages or status dump: the larger
    *                this value the more detailed is the output
    *                (0x####0000=>off by default). The highest word
    *                indicates how many successive calls to the dump
    *                shall be skipped in order to reduce the amount
    *                of information on the screen (ex:
    *                0x0000####=>print all iterations,
    *                0x0001####=>print one iteration and skip the
    *                next one, 0x0002####=> print one iteration and
    *                skip the next two).
    * Shall be implemented.
    */
    virtual void printIter(const unsigned int verbose=0) = 0;

    /**
    * Method to be called within the printIter routine inherited by 
    * children in order to handle the highest word of verbose 
    * integer. 
    */
    unsigned int printHandling(const unsigned int verbose=0);

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_ALL => complete pose control.
    *  IKINCTRL_POSE_XYZ => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG => rotational part of pose controlled.
    * @param q0 is the vector of initial joint angles values.
    */
    iKinCtrl(iKinChain &c, unsigned int _ctrlPose, const Vector &q0);

    /**
    * Enables/Disables joint angles constraints.
    * @param _constrained if true then constraints are applied.
    */
    virtual void setChainConstraints(bool _constrained) { chain.setAllConstraints(_constrained); }

    /**
    * Executes one iteration of the control algorithm 
    * @param xd is the End-Effector target Pose to be tracked.
    * @param verbose is a integer whose 32 bits are intended as
    *                follows. The lowest word (16 bits)
    *                progressively enables different levels of
    *                warning messages or status dump: the larger
    *                this value the more detailed is the output
    *                (0x####0000=>off by default). The highest word
    *                indicates how many successive calls to the dump
    *                shall be skipped in order to reduce the amount
    *                of information on the screen (ex:
    *                0x0000####=>print all iterations,
    *                0x0001####=>print one iteration and skip the
    *                next one, 0x0002####=> print one iteration and
    *                skip the next two).
    * @return current estimation of joints configuration.
    * Shall be implemented. 
    */
    virtual Vector iterate(Vector &xd, const unsigned int verbose=0) = 0;

    /**
    * Iterates the control algorithm trying to converge on the 
    * target. 
    * @param xd is the End-Effector target Pose to be tracked. 
    * \see setInTargetTol 
    * \see getInTargetTol 
    * @param tol_size exits if test_convergence(tol_size) is true 
    *                 (tol_size<0 disables this check, default).
    * @param max_iter exits if iter>=max_iter (max_iter<0 disables
    *                 this check, default).
    * @param verbose is a integer whose 32 bits are intended as
    *                follows. The lowest word (16 bits)
    *                progressively enables different levels of
    *                warning messages or status dump: the larger
    *                this value the more detailed is the output
    *                (0x####0000=>off by default). The highest word
    *                indicates how many successive calls to the dump
    *                shall be skipped in order to reduce the amount
    *                of information on the screen (ex:
    *                0x0000####=>print all iterations,
    *                0x0001####=>print one iteration and skip the
    *                next one, 0x0002####=> print one iteration and
    *                skip the next two).
    * @param exit_code stores the exit code (NULL by default). Test 
    *                  for one of this:
    *                 IKINCTRL_RET_TOLX
    *                 IKINCTRL_RET_TOLSIZE
    *                 IKINCTRL_RET_TOLQ
    *                 IKINCTRL_RET_MAXITER
    *                 IKINCTRL_RET_EXHALT
    * @param exhalt checks for an external request to exit (NULL by
    *               default).
    */
    virtual Vector solve(Vector &xd, const double tol_size=IKINCTRL_DISABLED, const int max_iter=IKINCTRL_DISABLED,
                         const unsigned int verbose=0, int *exit_code=NULL, bool *exhalt=NULL);

    /**
    * Tests convergence by comparing the size of the algorithm 
    * internal structure (may be the gradient norm or the simplex
    * size or whatever) to a certain tolerance. 
    * @param tol_size is tolerance to compare to.
    * Shall be implemented.
    */
    virtual bool test_convergence(const double tol_size) = 0;

    /**
    * Reinitializes the algorithm's internal state and resets the 
    * starting point. 
    * @param q0 is the new starting point. 
    */
    virtual void restart(const Vector &q0) { State=IKINCTRL_STATE_RUNNING; iter=0; set_q(q0); }

    /**
    * Returns the algorithm's name.
    * @return algorithm name as string.
    * Shall be implemented. 
    */
    virtual string getAlgoName() = 0;

    /**
    * Switch on/off the watchDog mechanism to trigger deadLocks. 
    * A deadLock is triggered whenerver norm(q(k)-q(k-1))<tol_q for 
    * a specified number of iterations. 
    * @param sw control the watchDog activation. 
    */
    void switchWatchDog(bool sw) { watchDogOn=sw; }

    /**
    * Sets tolerance for in-target check (1e-2 by default). 
    * @param tol_x is the tolerance
    */
    virtual void setInTargetTol(double tol_x) { inTargetTol=tol_x; }

    /**
    * Returns tolerance for in-target check. 
    * @return tolerance
    */
    virtual double getInTargetTol() { return inTargetTol; }

    /**
    * Sets tolerance for watchDog check (1e-4 by default). 
    * @param tol_q is the tolerance
    */
    virtual void setWatchDogTol(double tol_q) { watchDogTol=tol_q; }

    /**
    * Returns tolerance for watchDog check. 
    * @return tolerance
    */
    virtual double getWatchDogTol() { return watchDogTol; }

    /**
    * Sets maximum number of iterations to trigger the watchDog (200
    * by default). 
    * @param maxIter is the iterations limit.
    */
    virtual void setWatchDogMaxIter(int maxIter) { watchDogMaxIter=maxIter; }

    /**
    * Returns maximum number of iterations to trigger the watchDog
    * @return iterations limit.
    */
    virtual int getWatchDogMaxIter() { return watchDogMaxIter; }

    /**
    * Checks if the End-Effector is in target. 
    * @return true if in target.
    */
    virtual bool isInTarget() { return dist()<inTargetTol; }
                                                                          
    /**
    * Returns the algorithm's state.
    * @return algorithm's state:
    * IKINCTRL_STATE_RUNNING 
    * IKINCTRL_STATE_INTARGET 
    * IKINCTRL_STATE_DEADLOCK 
    */
    int get_State() { return State; }

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
    * Returns the number of Chain DOF
    * @return number of Chain DOF.
    */
    unsigned int get_dim() { return dim; }

    /**
    * Returns the number of performed iterations.
    * @return number of performed iterations.
    */
    unsigned int get_iter() { return iter; }

    /**
    * Returns the actual cartesian position of the End-Effector.
    * @return actual cartesian position of the End-Effector.
    */
    virtual Vector get_x() { return x; }

    /**
    * Returns the actual cartesian position error.
    * @return actual cartesian position error.
    */
    virtual Vector get_e() { return e; }

    /**
    * Sets the joint angles values.
    * @param q0 is the joint angles vector. 
    */
    virtual void set_q(const Vector &q0);

    /**
    * Returns the actual joint angles values.
    * @return actual joint angles values.
    */
    virtual Vector get_q() { return q; }

    /**
    * Returns the actual gradient.
    * @return actual gradient.
    */
    virtual Vector get_grad() { return grad; }

    /**
    * Returns the actual Jacobian used in computation.
    * @return actual Jacobian.
    */
    virtual Matrix get_J() { return J; }

    /**
    * Returns the actual distance from the target in cartesian space
    * (euclidean norm is used). 
    * @return actual distance from the target.
    */
    virtual double dist() { return norm(e); }

    /**
    * Default destructor.
    */
    virtual ~iKinCtrl() { }
};


/**
* \ingroup iKinInv
*
* A class derived from iKinCtrl implementing two standard 
* algorithms based on steepest descent qdot=-Kp*grad. 
* 1) grad=-Jt*e 
* 2) grad=-pinv(J)*e. 
*/
class iKin::SteepCtrl : public iKin::iKinCtrl
{
private:
    // Default constructor: not implemented.
    SteepCtrl();
    // Copy constructor: not implemented.
    SteepCtrl(const SteepCtrl&);
    // Assignment operator: not implemented.
    SteepCtrl &operator=(const SteepCtrl&);

protected:
    double      Ts;
    Integrator *I;

    unsigned int type;
    bool constrained;

    double Kp;
    Vector qdot;
    Vector gpm;

    virtual void   inTargetFcn()         { }
    virtual void   deadLockRecoveryFcn() { }
    virtual void   printIter(const unsigned int verbose);
    virtual Vector update_qdot();

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param _type one of the following: 
    *  IKINCTRL_STEEP_JT   => implements J transposed method.
    *  IKINCTRL_STEEP_PINV => implements J pseudo-inverse method.
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_ALL => complete pose control.
    *  IKINCTRL_POSE_XYZ => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG => rotational part of pose controlled.
    * @param q0 is the vector of initial joint angles values. 
    * @param _Ts is the controller sampling time.
    * @param _Kp is constant gain.
    */
    SteepCtrl(iKinChain &c, unsigned int _type, unsigned int _ctrlPose,
              const Vector &q0, double _Ts, double _Kp);

    /**
    * Returns the further contribution to the qdot=pinvJ*xdot 
    * equation according to the Gradient Projection Method, i.e. 
    * qdot=pinvJ*xdot+(I-pinvJ*J)*w 
    * @return shall return the quantity (I-pinvJ*J)*w. 
    * \note This method shall be inherited and handled accordingly 
    *       (here a vector of 0s is returned). To do that, J and
    *       pinvJ are already computed when this method is called.
    */
    virtual Vector computeGPM() { Vector ret(dim); ret=0.0; return ret; }

    virtual void   setChainConstraints(bool _constrained);
    virtual Vector iterate(Vector &xd, const unsigned int verbose=0);
    virtual void   restart(const Vector &q0);
    virtual bool   test_convergence(const double tol_size) { return norm(grad)<tol_size; }
    virtual string getAlgoName()                           { return "steepest-descent";  }

    /**
    * Resets integral status at the current joint angles.
    */
    void resetInt() { I->reset(q); }

    /**
    * Returns the actual derivative of joint angles.
    * @return the actual derivative of joint angles. 
    */
    Vector get_qdot() { return qdot; }

    /**
    * Returns the actual value of Gradient Projected.
    * @return the actual value of Gradient Projected.
    */
    Vector get_gpm() { return gpm; }

    /**
    * Returns the gain.
    * @return the gain.
    */
    double get_Kp() { return Kp; }

    /**
    * Destructor.
    */                                                         
    virtual ~SteepCtrl() { delete I; }
};


/**
* \ingroup iKinInv
*
* A class derived from SteepCtrl implementing the variable gain 
* algorithm 
*  
* r(k)=dist(k)/dist(k-1) 
* r(k)<1 => Kp(k)=Kp(k-1)*Kp_inc; 
* r(k)>max_per_inc => Kp(k)=Kp(k-1)*Kp_dec;
*/
class iKin::VarKpSteepCtrl : public iKin::SteepCtrl
{
private:
    // Default constructor: not implemented.
    VarKpSteepCtrl();
    // Copy constructor: not implemented.
    VarKpSteepCtrl(const VarKpSteepCtrl&);
    // Assignment operator: not implemented.
    VarKpSteepCtrl &operator=(const VarKpSteepCtrl&);

protected:
    double Kp0;
    double Kp_inc;
    double Kp_dec;
    double Kp_max;
    double max_perf_inc;

    double dist_old;

    void           reset_Kp();
    virtual void   inTargetFcn() { reset_Kp(); }
    virtual Vector update_qdot();

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param _type one of the following: 
    *  IKINCTRL_STEEP_JT   => implements J transposed method.
    *  IKINCTRL_STEEP_PINV => implements J pseudo-inverse method.
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_ALL => complete pose control.
    *  IKINCTRL_POSE_XYZ => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG => rotational part of pose controlled.
    * @param q0 is the vector of initial joint angles values. 
    * @param _Ts is the controller sampling time.
    * @param _Kp0 is the initial gain. 
    * @param _Kp_inc is the increasing factor.
    * @param _Kp_dec is the drecreasing factor.
    * @param _Kp_max is the maximum value for Kp.
    * @param _max_perf_inc is the threshold value to decreas Kp. 
                                                                */
    VarKpSteepCtrl(iKinChain &c, unsigned int _type, unsigned int _ctrlPose, const Vector &q0, double _Ts,
                   double _Kp0, double _Kp_inc, double _Kp_dec, double _Kp_max, double _max_perf_inc);

    virtual void   restart(const Vector &q0) { SteepCtrl::restart(q0); reset_Kp();      }
    virtual string getAlgoName()             { return "variable-gain-steepest-descent"; }
};


/**
* \ingroup iKinInv
*
* A class derived from iKinCtrl implementing the
* Levenberg-Marquardt algorithm: 
*  
* qdot=-pinv(Jt*J+mu*I)*grad 
*
* r(k)=dist(k)/dist(k-1)
* r(k)<1 => mu(k)=mu(k-1)*mu_dec;
* r(k)>1 => mu(k)=mu(k-1)*mu_inc;
*
* H=Jt*J is the approximation of Hessian matrix
*/
class iKin::LMCtrl : public iKin::iKinCtrl
{
private:
    // Default constructor: not implemented.
    LMCtrl();
    // Copy constructor: not implemented.
    LMCtrl(const LMCtrl&);
    // Assignment operator: not implemented.
    LMCtrl &operator=(const LMCtrl&);

protected:
    double      Ts;
    Integrator *I;

    bool constrained;

    Vector qdot;
    Vector gpm;

    double mu;
    double mu0;
    double mu_inc;
    double mu_dec;
    double mu_min;
    double mu_max;

    double dist_old;

    void           reset_mu();
    virtual double update_mu();
    virtual void   inTargetFcn()         { reset_mu(); }
    virtual void   deadLockRecoveryFcn() { }
    virtual void   printIter(const unsigned int verbose);

public:
    /**
    * Constructor.
    * @param c is the Chain object on which the control operates. Do
    *          not change Chain DOF from this point onwards!!
    * @param _ctrlPose one of the following:
    *  IKINCTRL_POSE_ALL => complete pose control.
    *  IKINCTRL_POSE_XYZ => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG => rotational part of pose controlled.
    * @param q0 is the vector of initial joint angles values. 
    * @param _Ts is the controller sampling time. 
    * @param _mu0 is the initial value for weighting factor mu.
    * @param _mu_inc is the increasing factor.
    * @param _mu_dec is the drecreasing factor.
    * @param _mu_min is the minimum value for mu.
    * @param _mu_max is the maximum value for mu.
    */
    LMCtrl(iKinChain &c, unsigned int _ctrlPose, const Vector &q0, double _Ts, double _mu0,
           double _mu_inc, double _mu_dec, double _mu_min, double _mu_max);

    /**
    * Returns the further contribution to the qdot=pinvJ*xdot 
    * equation according to the Gradient Projection Method, i.e. 
    * qdot=pinvJ*xdot+(I-pinvJ*J)*w 
    * @return shall return the quantity (I-pinvJ*J)*w. 
    * \note This method shall be inherited and handled accordingly 
    *       (here a vector of 0s is returned). To do that, J and
    *       pinvJ are already computed when this method is called.
    */
    virtual Vector computeGPM() { Vector ret(dim); ret=0.0; return ret; }

    virtual void   setChainConstraints(bool _constrained);
    virtual Vector iterate(Vector &xd, const unsigned int verbose=0);
    virtual void   restart(const Vector &q0);
    virtual bool   test_convergence(const double tol_size) { return norm(grad)<tol_size;   }
    virtual string getAlgoName()                           { return "levenberg-marquardt"; }

    /**
    * Resets integral status at the current joint angles.
    */
    void resetInt() { I->reset(q); }

    /**
    * Returns the actual derivative of joint angles.
    * @return the actual derivative of joint angles. 
    */
    Vector get_qdot() { return qdot; }

    /**
    * Returns the actual value of Gradient Projected.
    * @return the actual value of Gradient Projected.
    */
    Vector get_gpm() { return gpm; }

    /**
    * Returns the current weighting factor mu.
    * @return the current weighting factor mu.
    */
    double get_mu() { return mu; }

    /**
    * Destructor.
    */                                                         
    virtual ~LMCtrl() { delete I; }
};


/**
* \ingroup iKinInv
*
* A class derived from LMCtrl implementing the Gradient 
* Projection Method according to the paper available at
* http://robotics.hanyang.ac.kr/new/papers/TA02-4.pdf . 
*/
class iKin::LMCtrl_GPM : public iKin::LMCtrl
{
private:
    // Default constructor: not implemented.
    LMCtrl_GPM();
    // Copy constructor: not implemented.
    LMCtrl_GPM(const LMCtrl_GPM&);
    // Assignment operator: not implemented.
    LMCtrl_GPM &operator=(const LMCtrl_GPM&);

protected:
    double safeAreaRatio;
    double K;

    Vector span;
    Vector alpha_min;
    Vector alpha_max;

public:
    /**
    * Constructor.
    */
    LMCtrl_GPM(iKinChain &c, unsigned int _ctrlPose, const Vector &q0, double _Ts, double _mu0,
               double _mu_inc, double _mu_dec, double _mu_min, double _mu_max);

    virtual Vector computeGPM();

    /**
    * Sets the GPM gain (shall be positive).
    * @param _K GPM gain.
    */
    void set_K(const double _K) { K=_K>0 ? _K : -_K; }

    /**
    * Returns the GPM gain (1.0 by default).
    * @return the GPM gain.
    */
    double get_K() { return K; }

    /**
    * Sets the safe area ratio [0-1], which is for each joint the
    * ratio between the angle span within which the chain can be 
    * operated with 0-GPM and the overall angle span.
    * @param _safeAreaRatio the safe area ratio.
    */
    void set_safeAreaRatio(const double _safeAreaRatio);

    /**
    * Returns the safe area ratio (0.9 by default).
    * @return the safe area ratio.
    */
    double get_safeAreaRatio() { return safeAreaRatio; }
};


/**
* \ingroup iKinInv
*
* A class derived from iKinCtrl solving the minimization 
* task: 
*  
* min( 1/2*|xd-x|^2 + Sum_on_some_i( 1/2*|qfi-qi|^2 ) ) 
*  
* Implemented algorithms relying on GSL library are: 
*  
* 1) Steepest Descent Gradient 
* 2) Conjugate Gradient FR (Fletcher-Reeves) 
* 3) Conjugate Gradient PR (Polak-Ribiere)
* 4) Quasi Newton BFGS (Broyden-Fletcher-Goldfarb-Shanno) 
* 5) Nelder-Mead Simplex
*/ 
class iKin::GSLMinCtrl : public iKin::iKinCtrl
{
private:
    // Default constructor: not implemented.
    GSLMinCtrl();
    // Copy constructor: not implemented.
    GSLMinCtrl(const GSLMinCtrl&);
    // Assignment operator: not implemented.
    GSLMinCtrl &operator=(const GSLMinCtrl&);

protected:
    unsigned int algo_type;

    Vector q_set;
    deque<int> hash_qf;
    int q_set_len;

    double step_size;
    double tol;

    bool fdfOn;

    const gsl_multimin_fdfminimizer_type *T1;
    gsl_multimin_fdfminimizer            *s1;
    gsl_multimin_function_fdf             des1;

    const gsl_multimin_fminimizer_type   *T2;
    gsl_multimin_fminimizer              *s2;
    gsl_multimin_function                 des2;

    void         reset(const Vector &q0);
    virtual void inTargetFcn()         { }
    virtual void deadLockRecoveryFcn() { }
    virtual void printIter(const unsigned int verbose);

    friend double _f(const gsl_vector *v, void *params);
    friend void   _df(const gsl_vector *v, void *params, gsl_vector *g);
    friend void   _fdf(const gsl_vector *v, void *params, double *f, gsl_vector *g);

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_ALL => complete pose control.
    *  IKINCTRL_POSE_XYZ => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG => rotational part of pose controlled.
    * @param q0 is the vector of initial joint angles values. 
    * @param _algo_type is algorithm type; it can be one of: 
    *   GSLALGOTYPE_STEEPEST
    *   GSLALGOTYPE_CONJ_FR
    *   GSLALGOTYPE_CONJ_PR
    *   GSLALGOTYPE_BFGS
    *   GSLALGOTYPE_NMSIMPLEX
    * @param step_size the size of the first trial step
    * @param tol is the accuracy of the line minimization; it is
    *            considered successful if dot(p,g)<tol*|p|*|g|,
    *            where p is the line direction and g is the
    *            gradient.
    */
    GSLMinCtrl(iKinChain &c, unsigned int _ctrlPose, const Vector &q0,
               const unsigned int _algo_type, double _step_size, double _tol);

    virtual Vector iterate(Vector &xd, const unsigned int verbose=0);
    virtual void   restart(const Vector &q0) { iKinCtrl::restart(q0); reset(q0); }
    virtual bool   test_convergence(const double tol_size);
    virtual string getAlgoName();

    /**
    * Enables the joint angles constraints check.
    * @param sw is a vector containing non-zero elements in the 
    *           corresponding position of the joint angle whose
    *           value needs to be controlled.
    */
    void switch_qf(const Vector &sw);

    /**
    * Sets the controlled joint angles target values.
    * @param qf is a vector of target joint angles values.
    */
    void set_qf(const Vector &qf);

    /**
    * Returns the controlled joint angles target values. 
    * @return the vector of actual target joint angles.
    */
    Vector get_qf() { return q_set; }

    virtual void   set_q(const Vector &q0) { iKinCtrl::set_q(q0); reset(q0); }
    virtual Vector get_x();
    virtual Vector get_e();
    virtual Vector get_grad();
    virtual Matrix get_J();
    virtual double dist();
    void           set_size(double _size);
    double         get_size();

    /**
    * Default destructor.
    */                                                         
    virtual ~GSLMinCtrl();
};


/**
* \ingroup iKinInv
*
* A class derived from iKinCtrl implementing the 
* multi-referential dynamical systems approach described at
* http://infoscience.epfl.ch/record/114045 .
*/
class iKin::MultiRefDynCtrl : public iKin::iKinCtrl
{
private:
    // Default constructor: not implemented.
    MultiRefDynCtrl();
    // Copy constructor: not implemented.
    MultiRefDynCtrl(const MultiRefDynCtrl&);
    // Assignment operator: not implemented.
    MultiRefDynCtrl &operator=(const MultiRefDynCtrl&);

protected:
    Integrator *I_xdot2;
    Integrator *I_qdot2;
    Integrator *I_qdot;
    Vector      qdot;
    Vector      xdot;
    Vector      qdot2;
    Vector      xdot2;
    Matrix      W;

    double Ts;
    double alpha;
    double beta;
    double gamma;
    double guardRatio;

    Vector qGuardMin;
    Vector qGuardMax;

    void computeGuard();

    virtual Vector calc_e();
    virtual void   inTargetFcn()         { }
    virtual void   deadLockRecoveryFcn() { }
    virtual void   printIter(const unsigned int verbose);

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_ALL => complete pose control.
    *  IKINCTRL_POSE_XYZ => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG => rotational part of pose controlled.
    * @param q0 is the vector of initial joint angles values. 
    * @param _Ts is the controller sampling time.
    */
    MultiRefDynCtrl(iKinChain &c, unsigned int _ctrlPose, const Vector &q0, double _Ts);

    /**
    * Executes one iteration of the control algorithm 
    * @param xd is the End-Effector target Pose to be tracked. 
    * @param qd is the target joint angles (it shall satisfy the
    *           forward kinematic function xd=f(qd)).
    * \note The reason why qd is provided externally instead of 
    *       computed here is to discouple the inverse kinematic
    *       problem (which may require some computational effort
    *       depending on the current pose xd and the chosen
    *       algorithm, should not interrupt the control loop and
    *       whose function calling rate is worth to be programmable)
    *       from the reaching issue.
    * @param verbose is a integer whose 32 bits are intended as
    *                follows. The lowest word (16 bits)
    *                progressively enables different levels of
    *                warning messages or status dump: the larger
    *                this value the more detailed is the output
    *                (0x####0000=>off by default). The highest word
    *                indicates how many successive calls to the dump
    *                shall be skipped in order to reduce the amount
    *                of information on the screen (ex:
    *                0x0000####=>print all iterations,
    *                0x0001####=>print one iteration and skip the
    *                next one, 0x0002####=> print one iteration and
    *                skip the next two).
    * @return current estimation of joints configuration. 
    */
    virtual Vector iterate(Vector &xd, Vector &qd, const unsigned int verbose=0);

    virtual void restart(const Vector &q0);

    virtual string getAlgoName() { return "multi-referential-dynamical-systems"; }

    // disable unused father's methods
    virtual bool   test_convergence(const double tol_size)           { return false;     }
    virtual Vector iterate(Vector &xd, const unsigned int verbose=0) { return Vector(0); }
    virtual Vector solve(Vector &xd, const double tol_size=IKINCTRL_DISABLED, const int max_iter=IKINCTRL_DISABLED,
                         const unsigned int verbose=0, int *exit_code=NULL, bool *exhalt=NULL)
                                                                     { return Vector(0); }
    /**
    * Resets integral status at the current joint angles.
    */
    void resetIntPos() { I_qdot->reset(q); }

    /**
    * Resets integral status at the current joint velocities.
    */
    void resetIntVel() { I_qdot2->reset(qdot); I_xdot2->reset(xdot); }

    /**
    * Returns the guard ratio for the joints span (0.1 by default). 
    * \note The weights W_theta^-1 are non-zero only within the 
    *       range [ q_min+0.5*guardRatio*D, q_max-0.5*guardRatio*D ]
    *       for each join, where D=q_max-q_min.
    * @return guard ratio.
    */
    double get_guardRatio() { return guardRatio; }

    /**
    * Returns the parameter alpha (0.04 by default). 
    * @return alpha.
    */
    double get_alpha() { return alpha; }

    /**
    * Returns the parameter beta (0.01 by default). 
    * @return beta.
    */
    double get_beta() { return beta; }

    /**
    * Returns the parameter gamma (0.03 by default). 
    * @return gamma.
    */
    double get_gamma() { return gamma; }

    /**
    * Sets the guard ratio (in [0 1]). 
    * @param _guardRatio. 
    */
    void set_guardRatio(double _guardRatio);

    /**
    * Sets the parameter alpha (in [0,1]). 
    * @param _alpha. 
    */
    void set_alpha(double _alpha);

    /**
    * Sets the parameter beta (in [0,1]). 
    * @param _beta. 
    */
    void set_beta(double _beta);

    /**
    * Sets the parameter gamma. 
    * @param _gamma. 
    */
    void set_gamma(double _gamma) { gamma=_gamma; }

    /**
    * Sets the joint angles values.
    * @param q0 is the joint angles vector. 
    */
    virtual void set_q(const Vector &q0);

    /**
    * Sets the derivative of joint angles.
    * @param _qdot is the new derivative of joint angles vector. 
    */
    virtual void set_qdot(const Vector &_qdot);

    /**
    * Returns the actual derivative of joint angles.
    * @return the actual derivative of joint angles. 
    */
    Vector get_qdot() { return qdot; }

    /**
    * Returns the actual derivative of End-Effector Pose.
    * @return the actual derivative of End-Effector Pose. 
    */
    Vector get_xdot() { return xdot; }

    /**
    * Returns the actual second derivative of joint angles.
    * @return the actual second derivative of joint angles. 
    */
    Vector get_qdot2() { return qdot2; }

    /**
    * Returns the actual second derivative of End-Effector Pose.
    * @return the actual second derivative of End-Effector Pose. 
    */
    Vector get_xdot2() { return xdot2; }

    /**
    * Destructor.
    */                                                         
    virtual ~MultiRefDynCtrl();
};



#endif


