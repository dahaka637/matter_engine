#include "Engine/Control/BipedController3D.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {
using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;
using V3 = Eigen::Vector3d;
V3 ev(Vec3 v) { return {v.x,v.y,v.z}; }
Vec3 mv(const V3& v) { return {float(v.x()),float(v.y()),float(v.z())}; }
V3 error(Quaternion desired, Quaternion measured) {
    auto q=(desired*measured.conjugate()).normalized();
    if(q.w<0) q={-q.x,-q.y,-q.z,-q.w};
    V3 v(q.x,q.y,q.z); double n=v.norm();
    return v*(n<1e-8 ? 2.0 : 2*std::atan2(n,q.w)/n);
}
Matrix skew(V3 v) {
    Matrix s(3,3); s<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;return s;
}
struct QP {
    Matrix H, E, C;
    Vector g, b, d;
    explicit QP(int n):H(Matrix::Identity(n,n)*1e-6),E(0,n),C(0,n),g(Vector::Zero(n)),b(0),d(0) {}
    void task(const Matrix& A,const Vector& target,double weight) {
        H.noalias()+=weight*A.transpose()*A;g.noalias()-=weight*A.transpose()*target;
    }
    void limit(const Vector& a,double bound) {
        int r=C.rows(); C.conservativeResize(r+1,Eigen::NoChange);d.conservativeResize(r+1);
        double n=std::max(1e-9,a.norm());C.row(r)=a.transpose()/n;d[r]=bound/n;
    }
    bool solve(Vector& x,int& iterations,double& violation) {
        Eigen::LDLT<Matrix> factor(H);
        Vector free=factor.solve(-g);
        std::vector<int> active;
        for(iterations=0;iterations<96;++iterations) {
            Matrix A(E.rows()+active.size(),H.rows());Vector rhs(A.rows());
            A.topRows(E.rows())=E;rhs.head(E.rows())=b;
            for(int i=0;i<int(active.size());++i){A.row(E.rows()+i)=C.row(active[i]);rhs[E.rows()+i]=d[active[i]];}
            Matrix Y=factor.solve(A.transpose());
            Matrix schur=A*Y;
            schur.diagonal().array()+=1e-10;
            Vector lambda=schur.ldlt().solve(A*free-rhs);
            x=free-Y*lambda;
            int remove=-1;double worst=-1e-6;
            for(int i=0;i<int(active.size());++i)if(lambda[E.rows()+i]<worst){worst=lambda[E.rows()+i];remove=i;}
            if(remove>=0){active.erase(active.begin()+remove);continue;}
            Vector residual=C*x-d;
            int add=-1;worst=1e-5;
            for(int i=0;i<C.rows();++i)if(residual[i]>worst && std::find(active.begin(),active.end(),i)==active.end()){worst=residual[i];add=i;}
            if(add<0){violation=std::max(0.,residual.maxCoeff());return x.allFinite() && violation<1e-3;}
            active.push_back(add);
        }
        violation=std::max(0.,(C*x-d).maxCoeff());return false;
    }
};
}
struct BipedController3D::Impl {
    Telemetry t;
    std::vector<RagdollDriveTarget3D> output;
    int foot[2]{-1,-1};
    float yaw=0, nominalHeight=0;
    V3 velocity=V3::Zero(), anchor[2], comTarget=V3::Zero();
    Quaternion upright;
    Phase phase=Phase::Standing;
    double phaseTime=0;
    int swing=0, steps=0;
    V3 transferStart=V3::Zero(), swingStart=V3::Zero(), landing=V3::Zero(), pairCenter=V3::Zero();
    V3 plannedCom=V3::Zero();
    double width=.178, groundHeight=0;
    Matrix previousJ;
    void update(const RagdollProfile3D& p,const RagdollState3D& s,const RagdollDynamics3D& dyn,float dt) {
        output.clear();t.solved=false;
        if(!dyn.valid || s.links.size()!=p.links.size() || dt<=0 || dt>.05f)return;
        const int n=dyn.generalizedDofCount, nc=2, size=n+6*nc;
        Matrix J(dyn.jacobianRowCount,n),M(n,n);
        for(int r=0;r<J.rows();++r)for(int c=0;c<n;++c)J(r,c)=dyn.denseJacobian[r*n+c];
        for(int r=0;r<n;++r)for(int c=0;c<n;++c)M(r,c)=dyn.massMatrix[r*n+c];
        Vector qd(n),bias(n);for(int i=0;i<n;++i){qd[i]=dyn.generalizedVelocity[i];bias[i]=dyn.biasForce[i];}
        Vector jd=Vector::Zero(J.rows());
        if(previousJ.rows()==J.rows())jd=(J-previousJ)*qd/dt;
        previousJ=J;
        Matrix Jcom=Matrix::Zero(3,n);V3 com=V3::Zero(),vcom=V3::Zero(),acom=V3::Zero();
        for(int i=0;i<int(p.links.size());++i){double w=p.links[i].massFraction;int row=dyn.linkJacobianRow[i];
            Jcom+=w*J.block(row,0,3,n);com+=w*ev(s.links[i].position);vcom+=w*ev(s.links[i].linearVelocity);acom+=w*jd.segment<3>(row);}
        t.com=mv(com);t.velocity=mv(vcom);t.height=com.z();t.upright=s.links[0].orientation.rotate({0,0,1}).z;
        t.footLoad[0]=t.footLoad[1]=0;
        for(auto& c:s.contacts)for(int k=0;k<2;++k)if(int(c.linkIndex)==foot[k] && c.normal.z>.5f)t.footLoad[k]+=c.normalImpulseNewtonSeconds/dt;
        phaseTime+=dt;
        V3 desiredFoot[2]={anchor[0],anchor[1]};
        V3 footVelocity[2]={V3::Zero(),V3::Zero()},footAcceleration[2]={V3::Zero(),V3::Zero()};
        if(phase==Phase::Standing && velocity.head<2>().norm()>.01 && phaseTime>1.0) {
            phase=Phase::Transfer;phaseTime=0;swing=velocity.y()>=0?0:1;
            pairCenter=(anchor[0]+anchor[1])*.5;plannedCom=pairCenter;transferStart=plannedCom;
        }
        V3 desiredCom=(anchor[0]+anchor[1])*.5;
        if(phase==Phase::Transfer) {
            double u=std::clamp(phaseTime/.5,0.,1.);u=u*u*(3-2*u);
            plannedCom=transferStart*(1-u)+anchor[1-swing]*u;
            desiredCom=plannedCom;
            double load=t.footLoad[0]+t.footLoad[1];
            if(phaseTime>.45 && (com.head<2>()+vcom.head<2>()/3.2-anchor[1-swing].head<2>()).norm()<.035 && load>p.totalMassKg*9.81*.5 && t.footLoad[1-swing]/load>.75 && t.footLoad[swing]/load<.25) {
                phase=Phase::Swing;phaseTime=0;swingStart=anchor[swing];
                if(steps%2==0)pairCenter+=ev(upright.rotate(mv(velocity.normalized()*.12)));
                landing=pairCenter+ev(upright.rotate({0,float(swing==0?width*.5:-width*.5),0}));landing.z()=groundHeight;
            }
        }
        if(phase==Phase::Swing || phase==Phase::Landing) {
            constexpr double T=.65;
            double u=std::clamp(phaseTime/T,0.,1.);
            double blend=u*u*u*(10-15*u+6*u*u);
            double speed=30*u*u*(1-u)*(1-u)/T;
            double acc=60*u*(1-u)*(1-2*u)/(T*T);
            V3 delta=landing-swingStart;
            desiredFoot[swing]=swingStart+blend*delta;
            footVelocity[swing]=speed*delta;footAcceleration[swing]=acc*delta;
            double h=.08;
            desiredFoot[swing].z()+=h*64*u*u*u*(1-u)*(1-u)*(1-u);
            footVelocity[swing].z()+=h*192*u*u*(1-u)*(1-u)*(1-2*u)/T;
            footAcceleration[swing].z()+=h*384*u*(1-u)*(1-5*u+5*u*u)/(T*T);
            desiredCom=anchor[1-swing];
            if(u>=1) {phase=Phase::Landing;desiredFoot[swing].z()-=std::min(.04,(phaseTime-T)*.05);}
            if(phaseTime>T*.85 && t.footLoad[swing]>p.totalMassKg*9.81*.025) {
                anchor[swing]=landing;anchor[swing].z()=groundHeight;
                ++steps;t.steps=steps;transferStart=desiredCom;swing=1-swing;phase=Phase::Transfer;phaseTime=0;
                if(velocity.norm()<.01){phase=Phase::Standing;phaseTime=0;}
            }
        }
        t.phase=phase;
        QP qp(size);
        auto task=[&](Matrix a,Vector b,double w){Matrix A=Matrix::Zero(a.rows(),size);A.leftCols(n)=a;qp.task(A,b,w);};
        desiredCom.z()=nominalHeight;
        V3 comAcceleration=25*(desiredCom-com)-10*vcom;
        V3 desiredCop=com-comAcceleration*((com.z()-groundHeight)/9.81);
        V3 supportLine=anchor[1]-anchor[0];supportLine.z()=0;
        double rightShare=std::clamp((desiredCop-anchor[0]).dot(supportLine)/std::max(.001,supportLine.squaredNorm()),0.,1.);
        task(Jcom,comAcceleration-acom,100);
        for(int link: {0,2}) {
            int row=dyn.linkJacobianRow[link]+3;
            task(J.block(row,0,3,n),100*error(upright,s.links[link].orientation)-20*ev(s.links[link].angularVelocity)-jd.segment<3>(row),5);
        }
        Matrix contact=Matrix::Zero(12,n);
        for(int k=0;k<2;++k) {
            int row=dyn.linkJacobianRow[foot[k]];
            V3 offset(0,0,-p.links[foot[k]].collider.boxHalfExtents.z);
            offset=ev(s.links[foot[k]].orientation.rotate(mv(offset)));
            Matrix jc=J.block(row,0,6,n);jc.topRows(3)-=skew(offset)*jc.bottomRows(3);
            contact.block(k*6,0,6,n)=jc;
            V3 pos=ev(s.links[foot[k]].position)+offset;
            V3 vel=jc.topRows(3)*qd;
            Vector target(6);target.head<3>()=180*(desiredFoot[k]-pos)+26*(footVelocity[k]-vel)+footAcceleration[k];
            target.tail<3>()=180*error(upright,s.links[foot[k]].orientation)-26*ev(s.links[foot[k]].angularVelocity);
            Vector pointBias=jd.segment(row,6);pointBias.head<3>()-=skew(offset)*pointBias.tail<3>();
            V3 omega=ev(s.links[foot[k]].angularVelocity);pointBias.head<3>()+=omega.cross(omega.cross(offset));
            task(jc,target-pointBias,100);
            const int idx=n+k*6;
            bool supporting=!(k==swing && (phase==Phase::Swing || phase==Phase::Landing));
            if(!supporting) {
                for(int axis=0;axis<6;++axis){Vector rowv=Vector::Zero(size);rowv[idx+axis]=1;qp.limit(rowv,0);qp.limit(-rowv,0);}
            }
            for(int axis=0;axis<6;++axis){Vector rowv=Vector::Zero(size);rowv[idx+axis]=1;
                double share=k==0?1-rightShare:rightShare;
                if(phase==Phase::Swing || phase==Phase::Landing)share=k==swing?0:1;
                double desired=axis==2?p.totalMassKg*9.81*share:0;
                qp.task(rowv.transpose(),Vector::Constant(1,desired),axis==2?.01:axis<3?1e-5:1e-3);}
            Vector a=Vector::Zero(size);a[idx+2]=-1;qp.limit(a,0);
            const double bounds[5]={.6,.6,.040,.105,.02};const int axes[5]={0,1,3,4,5};
            for(int j=0;j<5;++j)for(int sign:{-1,1}){a.setZero();a[idx+axes[j]]=sign;a[idx+2]=-bounds[j];qp.limit(a,0);}
        }
        for(int link=1;link<int(p.links.size());++link)for(int a=0;a<3;++a){auto index=dyn.jointGeneralizedDof[link][a];if(index==RagdollDynamics3D::InvalidIndex)continue;
            Matrix row=Matrix::Zero(1,n);row(0,index)=1;float desired=0;
            task(row,Vector::Constant(1,25*(desired-s.joints[link].positionRadians[a])-10*qd[index]),.05);
        }
        // Floating-base equations must be supplied by contact reactions.
        Matrix equation=Matrix::Zero(n,size);equation.leftCols(n)=M;equation.rightCols(12)=-contact.transpose();
        qp.E=equation.topRows(6);qp.b=-bias.head(6);
        for(int link=1;link<int(p.links.size());++link)for(int a=0;a<3;++a){auto i=dyn.jointGeneralizedDof[link][a];if(i==RagdollDynamics3D::InvalidIndex)continue;
            const double cap=p.links[link].inboundJoint.axes[a].maximumTorque;
            qp.limit(equation.row(i).transpose(),cap-bias[i]);qp.limit(-equation.row(i).transpose(),cap+bias[i]);
        }
        Vector solution;double violation=0;
        t.solved=qp.solve(solution,t.solverIterations,violation);t.constraintViolation=violation;
        if(!t.solved)return;
        Vector torque=equation*solution+bias;t.dynamicsResidual=torque.head(6).norm();
        for(int link=1;link<int(p.links.size());++link)for(int a=0;a<3;++a){auto i=dyn.jointGeneralizedDof[link][a];if(i==RagdollDynamics3D::InvalidIndex)continue;
            RagdollDriveTarget3D out;out.linkIndex=link;out.axis=static_cast<RagdollAxis3D>(a);
            out.positionRadians=s.joints[link].positionRadians[a];out.stiffnessScale=0;out.dampingScale=0;out.maximumTorqueScale=1;
            out.feedforwardTorqueNewtonMeters=torque[i];output.push_back(out);
        }
    }
};
BipedController3D::BipedController3D():m(std::make_unique<Impl>()){}
BipedController3D::~BipedController3D()=default;
BipedController3D::BipedController3D(BipedController3D&&) noexcept=default;
BipedController3D& BipedController3D::operator=(BipedController3D&&) noexcept=default;
void BipedController3D::reset(const RagdollProfile3D& p,const RagdollState3D& s){
    m=std::make_unique<Impl>();
    Vec3 f=s.links[0].orientation.rotate({1,0,0});m->yaw=std::atan2(f.y,f.x);m->upright=Quaternion::fromAxisAngle({0,0,1},m->yaw);
    V3 com=V3::Zero();for(int i=0;i<int(p.links.size());++i){com+=p.links[i].massFraction*ev(s.links[i].position);
        for(int k=0;k<2;++k)if(p.links[i].id==(k==0?"LeftFoot":"RightFoot")){m->foot[k]=i;m->anchor[k]=ev(s.links[i].position)-V3(0,0,p.links[i].collider.boxHalfExtents.z);}}
    m->nominalHeight=com.z()-.10;m->comTarget=com;m->groundHeight=(m->anchor[0].z()+m->anchor[1].z())*.5;m->width=(m->anchor[0]-m->anchor[1]).norm();
}
void BipedController3D::setVelocity(Vec3 v){m->velocity=ev(v);}
void BipedController3D::update(const RagdollProfile3D& p,const RagdollState3D& s,const RagdollDynamics3D& d,float dt,bool manipulated){if(!manipulated)m->update(p,s,d,dt);else m->output.clear();}
const std::vector<RagdollDriveTarget3D>& BipedController3D::targets()const{return m->output;}
const BipedController3D::Telemetry& BipedController3D::telemetry()const{return m->t;}
}
