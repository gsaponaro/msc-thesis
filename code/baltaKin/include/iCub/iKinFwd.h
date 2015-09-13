/**
 * \defgroup iKin iKin
 *  
 * @ingroup icub_module 
 *  
 * Classes for forward-inverse kinematics of serial-links chains
 * and iCub limbs 
 *  
 * \note SI units adopted: meters for lengths and radians for
 *       angles.
 *  
 * \author Ugo Pattacini 
 *  
 * \defgroup iKinFwd iKinFwd 
 *  
 * @ingroup iKin  
 *
 * Classes for forward kinematics of serial-links chains 
 * and iCub limbs
 *
 * Date: first release 16/06/2008
 *
 * Parameters:
 *
 * \see iKinFwd
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __IKINFWD_H__
#define __IKINFWD_H__

#include <gsl/gsl_math.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <string>
#include <deque>


using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;

namespace iKin
{
    class iKinLink;
    class iKinChain;
    class iKinLimb;
    class iCubArm;
    class iCubLeg;
    class iCubEye;
    class iCubEyeNeckRef;
}


//Returns the squared norm of the vector
inline double norm2(const Vector &v)
{
    return dot(v,v);
}

//Returns the norm of the vector
inline double norm(const Vector &v)
{
    return sqrt(norm2(v));
}

//Returns the cross product between two vectors
Vector cross(const Vector &a, const Vector &b, unsigned int verbose=0);

//Converts a rotation matrix R to axis/angle representation
Vector dcm2axis(const Matrix &R, unsigned int verbose=0);

//Returns a rotation matrix R from axis representation
Matrix axis2dcm(Vector v, unsigned int verbose=0);

Matrix operator*(const double k, const Matrix &M);
Matrix operator*(const Matrix &M, const double k);



class iKin::iKinChain;


/**
* \ingroup iKinFwd
*
* A Base class for defining a Link with standard 
* Denavit-Hartenberg convention.
*/
class iKin::iKinLink
{
private:
    double       A;
    double       D;
    double       Alpha;
    double       Offset;
    double       c_alpha;
    double       s_alpha;
    double       Min;
    double       Max;
    double       Ang;
    bool         blocked;
    bool         cumulative;
    bool         constrained;
    unsigned int verbose;
    Matrix       H;
    Matrix       cumH;
    Matrix       DnH;

    friend class iKinChain;

    // Default constructor: not implemented.
    iKinLink();

    void _allocate_link(const iKinLink &l);
    bool isBlocked()                   { return blocked;             }
    bool isCumulative()                { return cumulative;          }
    void block()                       { blocked=true;               }
    void block(double _Ang)            { setAng(_Ang); blocked=true; }
    void release()                     { blocked=false;              }
    void addCumH(const Matrix &_cumH);
    void rmCumH()                      { cumulative=false;           }

public:
    /**
    * Constructor. 
    * @param _A is the Link length.
    * @param _D is the Link offset. 
    * @param _Alpha is the Link twist. 
    * @param _Offset is the joint angle offset in [-pi,pi]
    * @param _Min is the joint angle lower bound in [-pi,pi] (-pi by
    *             default).
    * @param _Max is the joint angle higher bound in [-pi,pi] (pi by
    *             default).
    */
    iKinLink(double _A, double _D, double _Alpha, double _Offset,
             double _Min=-M_PI, double _Max=M_PI);

    /**
    * Creates a new Link from an already existing Link object.
    * @param l is the Link to be copied.
    */
    iKinLink(const iKinLink &l);

    /**
    * Copies a Link object into the current one.
    * @param l is a reference to an object of type iKinLink.
    * @return a reference to the current object.
    */
    iKinLink &operator=(const iKinLink &l);

    /**
    * Sets the constraint status.
    * @param _constrained if true the joint angle cannot be set out 
    *                     of the its limits (initialized as true).
    */
    void setConstraint(bool _constrained) { constrained=_constrained; }

    /**
    * Returns the constraint status.
    * @return current constraint status.
    */
    bool getConstraint() { return constrained; }

    /**
    * Sets Link verbosity level.
    * @param _verbose is a integer number which progressively 
    *                 enables different levels of warning messages.
    *                 The larger this value the more detailed is the
    *                 output.
    */
    void setVerbosity(unsigned int _verbose) { verbose=_verbose; }

    /**
    * Returns the current Link verbosity level.
    * @return Link verbosity level.
    */
    unsigned int getVerbosity() { return verbose; }

    /**
    * Returns the Link length A.
    * @return Link length A.
    */
    double getA() { return A; }

    /**
    * Returns the Link offset D.
    * @return Link offset D.
    */
    double getD() { return D; }

    /**
    * Returns the Link twist Alpha.
    * @return Link twist Alpha.
    */
    double getAlpha() { return Alpha; }

    /**
    * Returns the joint angle offset.
    * @return joint angle offset.
    */
    double getOffset() { return Offset; }

    /**
    * Returns the joint angle lower bound.
    * @return joint angle lower bound.
    */
    double getMin() { return Min; }

    /**
    * Sets the joint angle lower bound.
    * @param _Min is the joint angle lower bound in [-pi,pi].
    */
    void setMin(const double _Min);

    /**
    * Returns the joint angle upper bound.
    * @return joint angle upper bound.
    */
    double getMax() { return Max; }

    /**
    * Sets the joint angle higher bound.
    * @param _Max is the joint angle higher bound in [-pi,pi].
    */
    void setMax(const double _Max);

    /**
    * Returns the current joint angle value.
    * @return current joint angle value.
    */
    double getAng() { return Ang; }

    /**
    * Sets the joint angle value. 
    * @param _Ang new value for joint angle.
    * @return actual joint angle value (constraints are evaluated).
    */
    double setAng(double _Ang);

    /**
    * Computes the homogeneous transformation matrix H of the Link.
    * @param c_override if true avoid accumulating the computation 
    *                   of previous links in the chain, i.e. H=Hn
    *                   and not H=Hn*Hn-1 (false by default)
    * @return a reference to H.
    */
    Matrix getH(bool c_override=false);

    /**
    * Same as getH() with specification of new joint angle position. 
    * \see getH
    * @param _Ang is the new joint angle position. 
    * @param c_override.
    * @return a reference to H.
    */
    Matrix getH(double _Ang, bool c_override=false);

    /**
    * Computes the derivative of order n of the homogeneous 
    * transformation matrix H with tespect to the joint angle. 
    * @param n is the order of the derivative (1 by default)
    * @param c_override if true avoid accumulating the computation 
    *                   of previous links in the chain (false by
    *                   default).
    * @return a reference to derivative of order n.
    */
    Matrix getDnH(unsigned int n=1, bool c_override=false);

    /**
    * Default destructor. 
    */
    ~iKinLink() { };
};



/**
* \ingroup iKinFwd
*
* A Base class for defining a Serial Link Chain. 
*/
class iKin::iKinChain
{
protected:
    unsigned int     N;
    unsigned int     DOF;
    unsigned int     verbose;
    Matrix           H0;
    Vector           curr_q;

    deque<iKinLink*> allList;
    deque<iKinLink*> quickList;

    deque<unsigned int> hash;
    deque<unsigned int> hash_dof;

    deque<Matrix>  hess_H;
    deque<Matrix> *hess_DH;

    void _allocate_chain(const iKinChain &c);
    void buildChain();
	void _dispose_chain();

    Vector RotAng(const Matrix &R);
    Vector dRotAng(const Matrix &R, const Matrix &dR);
    Vector d2RotAng(const Matrix &R, const Matrix &dRi,
                    const Matrix &dRj, const Matrix &d2R);

public:
    /**
    * Default constructor. 
    */
    iKinChain();

    /**
    * Constructor.
    * @param _H0 describes the rigid roto-translation from the root 
    *            reference frame to points in the 0th frame in the
    *            standard D-H convention.
    */
    iKinChain(const Matrix &_H0);

    /**
    * Creates a new Chain from an already existing Chain object.
    * @param c is the Chain to be copied.
    */
    iKinChain(const iKinChain &c);

    /**
    * Copies a Chain object into the current one.
    * @param c is a reference to an object of type iKinChain.
    * @return a reference to the current object.
    */
    iKinChain &operator=(const iKinChain &c);

    /**
    * Adds a Link at the bottom of the Chain.
    * @param l is the Link to be added.
    * @return a reference to the current object.
    */
    iKinChain &operator<<(iKinLink &l);

    /**
    * Removes a Link from the bottom of the Chain.
    * @return a reference to the current object.
    */
    iKinChain &operator--(int);

    /**
    * Returns a reference to the ith Link of the Chain.
    * @param i is the Link position within the Chain.
    * @return a reference to the ith Link object.
    */
    iKinLink &operator[](const unsigned int i) { return *allList[i]; }

    /**
    * Returns a reference to the ith Link of the Chain considering 
    * only those Links related to DOF.
    * @param i is the Link position within the Chain (in DOF order)
    * @return a reference to the ith Link object.
    */
    iKinLink &operator()(const unsigned int i) { return *quickList[hash_dof[i]]; }

    /**
    * Adds a Link at the position ith within the Chain. 
    * @param i is the ith position where the Link is to be added.
    * @param l is the Link to be added.
    * @return true if successful (e.g. param i is in range).
    */
    bool addLink(const unsigned int i, iKinLink &l);

    /**
    * Removes the ith Link from the Chain. 
    * @param i is the ith position from which the Link is to be 
    *          removed.
    * @return true if successful (e.g. param i is in range).
    */
    bool rmLink(const unsigned int i);

    /**
    * Adds a Link at the bottom of the Chain.
    * @param l is the Link to be added. 
    * \see operator<< 
    */
    void pushLink(iKinLink &l);

    /**
    * Removes all Links.
    */
    void clear();

    /**
    * Removes a Link from the bottom of the Chain. 
    * \see operator--  
    */
    void popLink();

    /**
    * Blocks the ith Link at the a certain value of its joint angle.
    * Chain DOF reduced by one. 
    * @param i is the Link position. 
    * @param Ang is the value of joint angle to which the Link is 
    *            blocked.
    * @return true if successful (e.g. param i is in range).
    */
    bool blockLink(const unsigned int i, double Ang);

    /**
    * Blocks the ith Link at the current value of its joint angle.
    * Chain DOF reduced by one. 
    * @param i is the Link position. 
    * @return true if successful (e.g. param i is in range).
    */
    bool blockLink(const unsigned int i) { return blockLink(i,getAng(i)); }

    /**
    * Releases the ith Link. 
    * Chain DOF augmented by one.
    * @param i is the Link position. 
    * @return true if successful (e.g. param i is in range).
    */
    bool releaseLink(const unsigned int i);

    /**
    * Sets the constraint status of all chain links.
    * @param _constrained is the new constraint status. 
    */
    void setAllConstraints(bool _constrained);

    /**
    * Sets the constraint status of ith link.
    * @param _constrained is the new constraint status. 
    */
    void setConstraint(unsigned int i, bool _constrained) { allList[i]->setConstraint(_constrained); }

    /**
    * Returns the constraint status of ith link. 
    * @return current constraint status.
    */
    bool getConstraint(unsigned int i) { return allList[i]->getConstraint(); }

    /**
    * Sets the verbosity level of all Links belonging to the Chain.
    * @param _verbose is the verbosity level. 
    */
    void setAllLinkVerbosity(unsigned int _verbose);

    /**
    * Sets the verbosity level of the Chain.
    * @param _verbose is a integer number which progressively 
    *                 enables different levels of warning messages.
    *                 The larger this value the more detailed is the
    *                 output.
    */
    void setVerbosity(unsigned int _verbose) { verbose=_verbose; }

    /**
    * Returns the current Chain verbosity level.
    * @return Link verbosity level.
    */
    unsigned int getVerbosity() { return verbose; }

    /**
    * Returns the number of Links belonging to the Chain.
    * @return number of Links.
    */
    unsigned int getN() { return N; }

    /**
    * Returns the current number of Chain's DOF. 
    * DOF=N-M, where N=# of Links, M=# of blocked Links. 
    * @return number of DOF.
    */
    unsigned int getDOF() { return DOF; }

    /**
    * Returns H0, the rigid roto-translation matrix from the root 
    * reference frame to the 0th frame. 
    * @return H0
    */
    Matrix getH0() { return H0; }

    /**
    * Sets H0, the rigid roto-translation matrix from the root 
    * reference frame to the 0th frame. 
    * @param H0
    */
    void setH0(const Matrix &_H0);

    /**
    * Sets the free joint angles to values of q[i].
    * @param q is a vector containing values for DOF.
    * @return the actual DOF values (angles constraints are 
    *         evaluated).
    */
    Vector setAng(const Vector &q);

    /**
    * Returns the current free joint angles values.
    * @return the actual DOF values.
    */
    Vector getAng();

    /**
    * Sets the ith joint angle. 
    * @param i is the Link position. 
    * @param Ang the new angle's value. 
    * @return current ith joint angle (angle constraint is 
    *         evaluated).
    */
    double setAng(const unsigned int i, double _Ang);

    /**
    * Returns the current angle of ith joint. 
    * @param i is the Link position.  
    * @return current ith joint angle.
    */
    double getAng(const unsigned int i);

    /**
    * Returns the rigid roto-translation matrix from the root 
    * reference frame to the ith frame in Denavit-Hartenberg 
    * notation. The second parameter if true enables the spannig 
    * over the full set of links, i.e. the blocked links as well. 
    * @param i is the Link position. 
    * @param allLink if true enables the spanning over the full set 
    *                of links (false by default).
    * @return Hi 
    */
    Matrix getH(const unsigned int i, const bool allLink=false);

    /**
    * Returns the rigid roto-translation matrix from the root 
    * reference frame to the Nth frame (End-Effector) in 
    * Denavit-Hartenberg notation. 
    * @return Hn
    */
    Matrix getH();

    /**
    * Returns the coordinates of ith Link. Two notations are
    * provided: the first with Euler Angles (XYZ form=>6x1 output 
    * vector) and second with axis/angle representation 
    * (default=>7x1 output vector). 
    * @param i is the Link position. 
    * @param axisRep if true returns the axis/angle notation. 
    * @return the ith Link Pose.
    */
    Vector Pose(const unsigned int i, const bool axisRep=true);

    /**
    * Returns the coordinates of End-Effector. Two notations are 
    * provided: the first with Euler Angles (XYZ form=>6x1 output 
    * vector) and second with axis/angle representation 
    * (default=>7x1 output vector). 
    * @param axisRep if true returns the axis/angle notation. 
    * @return the Nth Link Pose (End-Effector). 
    */
    Vector EndEffPose(const bool axisRep=true);

    /**
    * Returns the coordinates of End-Effector computed in q. Two
    * notations are provided: the first with Euler Angles (XYZ 
    * form=>6x1 output vector) and second with axis/angle 
    * representation (default=>7x1 output vector). 
    * @param q is the vector of new DOF values. 
    * @param axisRep if true returns the axis/angle notation.  
    * @return the Nth Link Pose (End-Effector).
    */
    Vector EndEffPose(const Vector &q, const bool axisRep=true);

    /**
    * Returns the analitical Jacobian.
    * @return the analitical Jacobian.
    */
    Matrix AnaJacobian();

    /**
    * Returns the analitical Jacobian computed in q. 
    * @param q is the vector of new DOF values. 
    * @return the analitical Jacobian.
    */
    Matrix AnaJacobian(const Vector &q);

    /**
    * Returns the geometric Jacobian.
    * @return the geometric Jacobian.
    */
    Matrix GeoJacobian();

    /**
    * Returns the geometric Jacobian computed in q. 
    * @param q is the vector of new DOF values. 
    * @return the geometric Jacobian.
    */
    Matrix GeoJacobian(const Vector &q);

    /**
    * Returns the 6x1 vector d2F(q)/dqidqj, where F(q) is the 
    * forward kinematic function and (qi,qj) is the DOF couple.
    * @param i is the index of the first DOF. 
    * @param j is the index of the second DOF.
    * @return the 6x1 vector d2F/dqidqj.
    */
    Vector Hessian_ij(const unsigned int i, const unsigned int j);

    /**
    * Prepares computation for a successive call to 
    * fastHessian_ij(). 
    * \see fastHessian_ij
    */
    void prepareForHessian();

    /**
    * Returns the 6x1 vector d2F(q)/dqidqj, where F(q) is the 
    * forward kinematic function and (qi,qj) is the DOF couple. Fast
    * Version: to be used in conjunction with prepareForHessian(). 
    * \note It is advisable to use this version when successive 
    * computations with different indexes values are needed. 
    * \see prepareForHessian 
    * @param i is the index of the first DOF. 
    * @param j is the index of the second DOF.
    * @return the 6x1 vector d2F/dqidqj.
    */
    Vector fastHessian_ij(const unsigned int i, const unsigned int j);

    /**
    * Destructor. 
    */
    ~iKinChain();
};


/**
* \ingroup iKinFwd
*
* A class for defining generic Limb
*/
class iKin::iKinLimb : protected iKin::iKinChain
{
protected:
    deque<iKinLink*> linkList;
    string           type;

    virtual void _allocate_limb(const string &_type);
    virtual void _copy_limb(const iKinLimb &limb);
    virtual void _dispose_limb();

public:
    /**
    * Default constructor. 
    */
    iKinLimb();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" limb
    */
    iKinLimb(const string &_type);

    /**
    * Creates a new Limb from an already existing Limb object.
    * @param limb is the Limb to be copied.
    */
    iKinLimb(const iKinLimb &limb);

    /**
    * Copies a Limb object into the current one.
    * @param limb is a reference to an object of type iKinLimb.
    * @return a reference to the current object.
    */
    iKinLimb &operator=(const iKinLimb &limb);

    /**
    * Returns the Limb as a Chain object. Useful to create a copy as
    * a Chain and then to operate on it 
    * @return a Chain object with the same Links of Limb.
    */
	iKinChain asChain() { return *static_cast<iKinChain*>(this); }

    /**
    * Returns the Limb type as a string. 
    * @return the type as a string {"left", "right"}.
    */
    string getType() { return type; }

    // Since iKinLimb is derived with protected modifier from iKinChain in order to make some methods hidden
    // to the user such as addLink, rmLink and so on, all the remaining public methods have to be
    // redeclared hereafter and simply inherited
                                                                     
    unsigned int getN()                                                     { return N;                                   }
    unsigned int getDOF()                                                   { return DOF;                                 }
    bool         blockLink(const unsigned int i, double Ang)                { return iKinChain::blockLink(i,Ang);         }
    bool         blockLink(const unsigned int i)                            { return iKinChain::blockLink(i);             }
    bool         releaseLink(const unsigned int i)                          { return iKinChain::releaseLink(i);           }
    void         setAllConstraints(bool _constrained)                       { iKinChain::setAllConstraints(_constrained); }
    void         setConstraint(unsigned int i, bool _constrained)           { iKinChain::setConstraint(i,_constrained);   }
    bool         getConstraint(unsigned int i)                              { return iKinChain::getConstraint(i);         }
    void         setAllLinkVerbosity(unsigned int _verbose)                 { iKinChain::setAllLinkVerbosity(_verbose);   }
    void         setVerbosity(unsigned int _verbose)                        { iKinChain::setVerbosity(_verbose);          }
    unsigned int getVerbosity()                                             { return iKinChain::getVerbosity();           }
    Vector       setAng(const Vector &q)                                    { return iKinChain::setAng(q);                }
    Vector       getAng()                                                   { return iKinChain::getAng();                 }
    double       setAng(const unsigned int i, double _Ang)                  { return iKinChain::setAng(i,_Ang);           }
    double       getAng(const unsigned int i)                               { return iKinChain::getAng(i);                }
    Matrix       getH(const unsigned int i, const bool allLink=false)       { return iKinChain::getH(i,allLink);          }
    Matrix       getH()                                                     { return iKinChain::getH();                   }
    Vector       Pose(const unsigned int i, const bool axisRep=true)        { return iKinChain::Pose(i,axisRep);          }
    Vector       EndEffPose(const bool axisRep=true)                        { return iKinChain::EndEffPose(axisRep);      }
    Vector       EndEffPose(const Vector &q, const bool axisRep=true)       { return iKinChain::EndEffPose(q,axisRep);    }
    Matrix       AnaJacobian()                                              { return iKinChain::AnaJacobian();            }
    Matrix       AnaJacobian(const Vector &q)                               { return iKinChain::AnaJacobian(q);           }
    Matrix       GeoJacobian()                                              { return iKinChain::GeoJacobian();            }
    Matrix       GeoJacobian(const Vector &q)                               { return iKinChain::GeoJacobian(q);           }
    Vector       Hessian_ij(const unsigned int i, const unsigned int j)     { return iKinChain::Hessian_ij(i,j);          }
    void         prepareForHessian()                                        { iKinChain::prepareForHessian();             }
    Vector       fastHessian_ij(const unsigned int i, const unsigned int j) { return iKinChain::fastHessian_ij(i,j);      }

    ~iKinLimb();
};


/**
* \ingroup iKinFwd
*
* A class for defining the 7-DOF iCub Arm
*/
class iKin::iCubArm : public iKin::iKinLimb
{
protected:
    virtual void _allocate_limb(const string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubArm();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" arm
    */
    iCubArm(const string &_type);

    /**
    * Creates a new Arm from an already existing Arm object.
    * @param arm is the Arm to be copied.
    */
    iCubArm(const iCubArm &arm);
};


/**
* \ingroup iKinFwd
*
* A class for defining the 6-DOF iCub Leg
*/
class iKin::iCubLeg : public iKin::iKinLimb
{
protected:
    virtual void _allocate_limb(const string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubLeg();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" leg
    */
    iCubLeg(const string &_type);

    /**
    * Creates a new Leg from an already existing Leg object.
    * @param leg is the Leg to be copied.
    */
    iCubLeg(const iCubLeg &leg);
};


/**
* \ingroup iKinFwd
*
* A class for defining the 5-DOF iCub Eye
*/
class iKin::iCubEye : public iKin::iKinLimb
{
protected:
    virtual void _allocate_limb(const string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubEye();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" eye
    */
    iCubEye(const string &_type);

    /**
    * Creates a new Eye from an already existing Eye object.
    * @param eye is the Eye to be copied.
    */
    iCubEye(const iCubEye &eye);
};


/**
* \ingroup iKinFwd
*
* A class for defining the 5-DOF iCub Eye with the root 
* reference frame attached to the neck. 
*/
class iKin::iCubEyeNeckRef : public iKin::iCubEye
{
protected:
    virtual void _allocate_limb(const string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubEyeNeckRef();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" eye
    */
    iCubEyeNeckRef(const string &_type);

    /**
    * Creates a new Eye from an already existing Eye object.
    * @param eye is the Eye to be copied.
    */
    iCubEyeNeckRef(const iCubEyeNeckRef &eye);
};



#endif



