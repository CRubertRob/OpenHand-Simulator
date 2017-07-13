// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "commonmanipulation.h"

/// samples rays from the projected OBB and returns true if the test function returns true
/// for all the rays. Otherwise, returns false
/// allowableoutliers - specifies the % of allowable outliying rays
bool SampleProjectedOBBWithTest(const OBB& obb, dReal delta, const boost::function<bool(const Vector&)>& testfn,dReal allowableocclusion=0)
{
    dReal fscalefactor = 0.95f; // have to make box smaller or else rays might miss
    Vector vpoints[8] = { obb.pos + fscalefactor*(obb.right*obb.extents.x + obb.up*obb.extents.y + obb.dir*obb.extents.z),
                          obb.pos + fscalefactor*(obb.right*obb.extents.x + obb.up*obb.extents.y - obb.dir*obb.extents.z),
                          obb.pos + fscalefactor*(obb.right*obb.extents.x - obb.up*obb.extents.y + obb.dir*obb.extents.z),
                          obb.pos + fscalefactor*(obb.right*obb.extents.x - obb.up*obb.extents.y - obb.dir*obb.extents.z),
                          obb.pos + fscalefactor*(-obb.right*obb.extents.x + obb.up*obb.extents.y + obb.dir*obb.extents.z),
                          obb.pos + fscalefactor*(-obb.right*obb.extents.x + obb.up*obb.extents.y - obb.dir*obb.extents.z),
                          obb.pos + fscalefactor*(-obb.right*obb.extents.x - obb.up*obb.extents.y + obb.dir*obb.extents.z),
                          obb.pos + fscalefactor*(-obb.right*obb.extents.x - obb.up*obb.extents.y - obb.dir*obb.extents.z)};
    //    Vector vpoints3d[8];
    //    for(int j = 0; j < 8; ++j) vpoints3d[j] = tcamera*vpoints[j];

    for(int i =0; i < 8; ++i) {
        dReal fz = 1.0f/vpoints[i].z;
        vpoints[i].x *= fz;
        vpoints[i].y *= fz;
        vpoints[i].z = 1;
    }

    int faceindices[3][4];
    if( obb.right.z >= 0 ) {
        faceindices[0][0] = 4; faceindices[0][1] = 5; faceindices[0][2] = 6; faceindices[0][3] = 7;
    }
    else {
        faceindices[0][0] = 0; faceindices[0][1] = 1; faceindices[0][2] = 2; faceindices[0][3] = 3;
    }
    if( obb.up.z >= 0 ) {
        faceindices[1][0] = 2; faceindices[1][1] = 3; faceindices[1][2] = 6; faceindices[1][3] = 7;
    }
    else {
        faceindices[1][0] = 0; faceindices[1][1] = 1; faceindices[1][2] = 4; faceindices[1][3] = 5;
    }
    if( obb.dir.z >= 0 ) {
        faceindices[2][0] = 1; faceindices[2][1] = 3; faceindices[2][2] = 5; faceindices[2][3] = 7;
    }
    else {
        faceindices[2][0] = 0; faceindices[2][1] = 2; faceindices[2][2] = 4; faceindices[2][3] = 6;
    }

    int nallowableoutliers=0;
    if( allowableocclusion > 0 ) {
        // have to compute the area of all the faces!
        dReal farea=0;
        for(int i = 0; i < 3; ++i) {
            Vector v0 = vpoints[faceindices[i][0]];
            Vector v1 = vpoints[faceindices[i][1]]-v0;
            Vector v2 = vpoints[faceindices[i][2]]-v0;
            Vector v = v1.cross(v2);
            farea += v.lengthsqr3();
        }
        nallowableoutliers = (int)(allowableocclusion*farea*0.5/(delta*delta));
    }

    for(int i = 0; i < 3; ++i) {
        Vector v0 = vpoints[faceindices[i][0]];
        Vector v1 = vpoints[faceindices[i][1]]-v0;
        Vector v2 = vpoints[faceindices[i][2]]-v0;
        Vector v3 = vpoints[faceindices[i][3]]-v0;
        dReal f3length = RaveSqrt(v3.lengthsqr2());
        Vector v3norm = v3 * (1.0f/f3length);
        Vector v3perp(-v3norm.y,v3norm.x,0,0);
        dReal f1proj = RaveFabs(v3perp.x*v1.x+v3perp.y*v1.y), f2proj = RaveFabs(v3perp.x*v2.x+v3perp.y*v2.y);

        int n1 = (int)(f1proj/delta);
        dReal n1scale = 1.0f/n1;
        Vector vdelta1 = v1*n1scale;
        Vector vdelta2 = (v1-v3)*n1scale;
        dReal fdeltalen = (RaveFabs(v3norm.x*v1.x+v3norm.y*v1.y) + RaveFabs(v3norm.x*(v1.x-v3.x)+v3norm.y*(v1.y-v3.y)))*n1scale;
        dReal ftotalen = f3length;
        Vector vcur1 = v0, vcur2 = v0+v3;
        for(int j = 0; j <= n1; ++j, vcur1 += vdelta1, vcur2 += vdelta2, ftotalen -= fdeltalen ) {
            int numsteps = (int)(ftotalen/delta);
            Vector vdelta = (vcur2-vcur1)*(1.0f/numsteps), vcur = vcur1;
            for(int k = 0; k <= numsteps; ++k, vcur += vdelta) {
                if( !testfn(vcur) ) {
                    if( nallowableoutliers-- <= 0 )
                        return false;
                }
            }
        }

        //        Vector vtripoints[6] = {vpoints3d[faceindices[i][0]], vpoints3d[faceindices[i][3]], vpoints3d[faceindices[i][1]],
        //                                vpoints3d[faceindices[i][0]], vpoints3d[faceindices[i][1]], vpoints3d[faceindices[i][3]]};
        //        penv->drawtrimesh(vtripoints[0], 16, NULL, 2);

        int n2 = (int)(f2proj/delta);
        if( n2 == 0 )
            continue;

        dReal n2scale = 1.0f/n2;
        vdelta1 = v2*n2scale;
        vdelta2 = (v2-v3)*n2scale;
        fdeltalen = (RaveFabs(v3norm.x*v2.x+v3norm.y*v2.y) + RaveFabs(v3norm.x*(v2.x-v3.x)+v3norm.y*(v2.y-v3.y)))*n2scale;
        ftotalen = f3length;
        vcur1 = v0; vcur2 = v0+v3;
        vcur1 += vdelta1; vcur2 += vdelta2; ftotalen -= fdeltalen; // do one step
        for(int j = 0; j < n2; ++j, vcur1 += vdelta1, vcur2 += vdelta2, ftotalen -= fdeltalen ) {
            int numsteps = (int)(ftotalen/delta);
            Vector vdelta = (vcur2-vcur1)*(1.0f/numsteps), vcur = vcur1;
            for(int k = 0; k <= numsteps; ++k, vcur += vdelta) {
                if( !testfn(vcur) ) {
                    if( nallowableoutliers-- <= 0 )
                        return false;
                }
            }
        }
    }

    return true;
}

class VisualFeedback : public ModuleBase
{
public:
    inline boost::shared_ptr<VisualFeedback> shared_problem() {
        return boost::dynamic_pointer_cast<VisualFeedback>(shared_from_this());
    }
    inline boost::shared_ptr<VisualFeedback const> shared_problem_const() const {
        return boost::dynamic_pointer_cast<VisualFeedback const>(shared_from_this());
    }
    friend class VisibilityConstraintFunction;

    //        void Init(const SensorBase::CameraIntrinsics& KK,int width, int height)
    //    {
    //        ffovx = atanf(0.5f*width/KK.fx);
    //        fcosfovx = cosf(ffovx); fsinfovx = sinf(ffovx);
    //        ffovy = atanf(0.5f*height/KK.fy);
    //        fcosfovy = cosf(ffovy); fsinfovy = sinf(ffovy);
    //    }

    class VisibilityConstraintFunction
    {
        class SampleRaysScope
        {
public:
            SampleRaysScope(VisibilityConstraintFunction& vcf) : _vcf(vcf) {
                _vcf._bSamplingRays = true;
            }
            ~SampleRaysScope() {
                _vcf._bSamplingRays = false;
            }
private:
            VisibilityConstraintFunction& _vcf;
        };
public:
        static void GetAABBFromOBB(const OBB& obb, Vector& vMin, Vector& vMax)
        {
            vMax.x = fabsf(obb.right.x) * obb.extents.x + fabsf(obb.up.x) * obb.extents.y + fabsf(obb.dir.x) * obb.extents.z;
            vMax.y = fabsf(obb.right.y) * obb.extents.x + fabsf(obb.up.y) * obb.extents.y + fabsf(obb.dir.y) * obb.extents.z;
            vMax.z = fabsf(obb.right.z) * obb.extents.x + fabsf(obb.up.z) * obb.extents.y + fabsf(obb.dir.z) * obb.extents.z;
            vMin = obb.pos - vMax;
            vMax += obb.pos;
        }

        VisibilityConstraintFunction(boost::shared_ptr<VisualFeedback> vf) : _vf(vf) {
            _report.reset(new CollisionReport());

            // create the dummy box
            {
                KinBody::KinBodyStateSaver saver(_vf->_target,KinBody::Save_LinkTransformation);
                _vf->_target->SetTransform(Transform());

                _vTargetOBBs.reserve(_vf->_target->GetLinks().size());
                FOREACHC(itlink, _vf->_target->GetLinks()) {
                    if( (*itlink)->IsVisible() ) {
                        _vTargetOBBs.push_back(geometry::OBBFromAABB((*itlink)->ComputeLocalAABB(),(*itlink)->GetTransform()));
                    }
                }

                vector<AABB> vboxes;
                if( _vTargetOBBs.size() > 0 ) {
                    Vector vmin, vmax;
                    GetAABBFromOBB(_vTargetOBBs.at(0), vmin, vmax);
                    for(size_t iobb = 1; iobb < _vTargetOBBs.size(); ++iobb) {
                        Vector vmin2, vmax2;
                        GetAABBFromOBB(_vTargetOBBs.at(iobb), vmin2, vmax2);
                        if( vmin.x > vmin2.x ) {
                            vmin.x = vmin2.x;
                        }
                        if( vmin.y > vmin2.y ) {
                            vmin.y = vmin2.y;
                        }
                        if( vmin.z > vmin2.z ) {
                            vmin.z = vmin2.z;
                        }
                        if( vmax.x < vmax2.x ) {
                            vmax.x = vmax2.x;
                        }
                        if( vmax.y < vmax2.y ) {
                            vmax.y = vmax2.y;
                        }
                        if( vmax.z < vmax2.z ) {
                            vmax.z = vmax2.z;
                        }
                    }

                    _abTarget.pos = 0.5*(vmin + vmax);
                    _abTarget.extents = vmax - _abTarget.pos;
                    _abTarget.extents.x += 0.0001;
                    _abTarget.extents.y += 0.0001;
                    _abTarget.extents.z += 0.0001;
                    vboxes.push_back(_abTarget);
                }

                // compute the AABB of _vTargetOBBs (could be different from _vf->_target->ComputeAABB())
                //_abTarget = _vf->_target->ComputeAABB();
                // have to increase its dimensions a little!

                _ptargetbox = RaveCreateKinBody(_vf->_target->GetEnv());
                _ptargetbox->InitFromBoxes(vboxes,true);
                _ptargetbox->SetName("__visualfeedbacktest__");
                _ptargetbox->GetEnv()->Add(_ptargetbox,true); // need to set to visible, otherwise will be ignored
                _ptargetbox->Enable(false);
                _ptargetbox->SetTransform(_vf->_target->GetTransform());
            }
            _ikreturn.reset(new IkReturn(IKRA_Success));
            _bSamplingRays = false;
            if( _vf->_bIgnoreSensorCollision && !!_vf->_sensorrobot ) {
                _collisionfn = _vf->_target->GetEnv()->RegisterCollisionCallback(boost::bind(&VisibilityConstraintFunction::_IgnoreCollisionCallback,this,_1,_2));
            }
        }
        virtual ~VisibilityConstraintFunction() {
            _ptargetbox->GetEnv()->Remove(_ptargetbox);
        }

        virtual bool IsVisible(bool bcheckocclusion=true)
        {
            Transform ttarget = _vf->_target->GetTransform();
            TransformMatrix tcamera = ttarget.inverse()*_vf->_psensor->GetTransform();
            if( !InConvexHull(tcamera) ) {
                RAVELOG_WARN("box not in camera vision hull (shouldn't happen due to preprocessing\n");
                return false;
            }
            if( bcheckocclusion && IsOccluded(tcamera) ) {
                return false;
            }
            return true;
        }

        virtual bool Constraint(const PlannerBase::PlannerParameters::CheckPathConstraintFn& oldfn, const vector<dReal>& pSrcConf, const vector<dReal>& pDestConf, IntervalType interval, PlannerBase::ConfigurationListPtr configurations)
        {
            if( !oldfn(pSrcConf,pDestConf,interval,configurations) ) {
                return false;
            }
            return IsVisible();
        }

        /// samples the ik
        /// If camera is attached to robot, assume target is not movable and t is the camera position.
        /// If camera is not attached to robot, assume target is movable and t is the target position.
        bool SampleWithCamera(const TransformMatrix& t, vector<dReal>& pNewSample)
        {
            Transform tCameraInTarget, ttarget;
            if( _vf->_robot != _vf->_sensorrobot ) {
                ttarget = t;
                tCameraInTarget = ttarget.inverse()*_vf->_psensor->GetTransform();
            }
            else {
                ttarget = _vf->_target->GetTransform();
                tCameraInTarget = ttarget.inverse()*t;
            }
            if( !InConvexHull(tCameraInTarget) ) {
                RAVELOG_DEBUG("box not in camera vision hull\n");
                return false;
            }

            // object is inside, find an ik solution
            Transform tgoalee = t*_vf->_ttogripper;
            if( !_vf->_pmanip->FindIKSolution(tgoalee,IKFO_CheckEnvCollisions, _ikreturn) ) {
                RAVELOG_VERBOSE_FORMAT("no valid ik 0x%.8x", _ikreturn->_action);
                return false;
            }
            _vsolution.swap(_ikreturn->_vsolution); // for now swap until this->_vsolution is removed

            // convert the solution into active dofs
            _vf->_robot->GetActiveDOFValues(pNewSample);
            FOREACHC(itarm,_vf->_pmanip->GetArmIndices()) {
                vector<int>::const_iterator itactive = find(_vf->_robot->GetActiveDOFIndices().begin(),_vf->_robot->GetActiveDOFIndices().end(),*itarm);
                if( itactive != _vf->_robot->GetActiveDOFIndices().end() ) {
                    pNewSample.at((int)(itactive-_vf->_robot->GetActiveDOFIndices().begin())) = _vsolution.at((int)(itarm-_vf->_pmanip->GetArmIndices().begin()));
                }
            }
            _vf->_robot->SetActiveDOFValues(pNewSample);

            return !IsOccluded(tCameraInTarget);
        }

        /// \param tCameraInTarget in target coordinate system
        /// \param mindist Minimum distance to keep from the plane (should be non-negative)
        bool InConvexHull(const TransformMatrix& tCameraInTarget, dReal mindist=0)
        {
            _vconvexplanes3d.resize(_vf->_vconvexplanes.size());
            for(size_t i = 0; i < _vf->_vconvexplanes.size(); ++i) {
                _vconvexplanes3d[i] = tCameraInTarget.rotate(_vf->_vconvexplanes[i]);
                _vconvexplanes3d[i].w = -tCameraInTarget.trans.dot3(_vconvexplanes3d[i]) - mindist;
            }
            FOREACH(itobb,_vTargetOBBs) {
                if( !geometry::IsOBBinConvexHull(*itobb,_vconvexplanes3d) ) {
                    return false;
                }
            }
            return true;
        }

        /// check if any part of the environment or robot is in front of the camera blocking the object
        /// sample object's surface and shoot rays
        /// \param tCameraInTarget in target coordinate system
        bool IsOccluded(const TransformMatrix& tCameraInTarget)
        {
            KinBody::KinBodyStateSaver saver1(_ptargetbox), saver2(_vf->_target,KinBody::Save_LinkEnable);
            TransformMatrix tCameraInTargetinv = tCameraInTarget.inverse();
            Transform ttarget = _vf->_target->GetTransform();
            _ptargetbox->SetTransform(ttarget);
            Transform tworldcamera = ttarget*tCameraInTarget;
            _ptargetbox->Enable(true);
            //_vf->_target->Enable(false);
            SampleRaysScope srs(*this);
            FOREACH(itobb,_vTargetOBBs) {
                OBB cameraobb = geometry::TransformOBB(tCameraInTargetinv,*itobb);
                if( !SampleProjectedOBBWithTest(cameraobb, _vf->_fSampleRayDensity, boost::bind(&VisibilityConstraintFunction::_TestRay, this, _1, boost::ref(tworldcamera)),_vf->_fAllowableOcclusion) ) {
                    RAVELOG_VERBOSE("box is occluded\n");
                    return true;
                }
            }
            return false;
        }

        /// check if just the rigidly attached links of the gripper are in the way
        /// this function is not meant to be called during planning (only database generation)
        bool IsOccludedByRigid(const TransformMatrix& tcamera)
        {
            KinBody::KinBodyStateSaver saver1(_ptargetbox), saver2(_vf->_target);
            vector<KinBody::LinkPtr> vattachedlinks;
            _vf->_psensor->GetAttachingLink()->GetRigidlyAttachedLinks(vattachedlinks);
            RobotBase::RobotStateSaver robotsaver(_vf->_robot,RobotBase::Save_LinkTransformation|RobotBase::Save_LinkEnable);
            Transform tsensorinv = _vf->_psensor->GetTransform().inverse();
            FOREACHC(itlink,_vf->_robot->GetLinks()) {
                bool battached = find(vattachedlinks.begin(),vattachedlinks.end(),*itlink)!=vattachedlinks.end();
                (*itlink)->Enable(battached);
                if( battached ) {
                    (*itlink)->SetTransform(tsensorinv*(*itlink)->GetTransform());
                }
            }
            TransformMatrix tcamerainv = tcamera.inverse();
            Transform ttarget = _vf->_target->GetTransform();
            _ptargetbox->SetTransform(ttarget);
            Transform tworldcamera = ttarget*tcamera;
            _ptargetbox->Enable(true);
            //_vf->_target->Enable(false);
            SampleRaysScope srs(*this);
            FOREACH(itobb,_vTargetOBBs) {
                OBB cameraobb = geometry::TransformOBB(tcamerainv,*itobb);
                if( !SampleProjectedOBBWithTest(cameraobb, _vf->_fSampleRayDensity, boost::bind(&VisibilityConstraintFunction::_TestRayRigid, this, _1, boost::ref(tworldcamera),boost::ref(vattachedlinks)), 0.0f) ) {
                    return true;
                }
            }
            return false;
        }

private:
        bool _TestRay(const Vector& v, const TransformMatrix& tcamera)
        {
            RAY r;
            dReal filen = 1/RaveSqrt(v.lengthsqr3());
            r.dir = tcamera.rotate((2.0f*filen)*v);
            r.pos = tcamera.trans + 0.5f*_vf->_fRayMinDist*r.dir;         // move the rays a little forward
            if( !_vf->_robot->GetEnv()->CheckCollision(r,_report) ) {
                return true;         // not supposed to happen, but it is OK
            }

            //            RaveVector<float> vpoints[2];
            //            vpoints[0] = r.pos;
            //            vpoints[1] = _report.contacts[0].pos;
            //            _vf->_robot->GetEnv()->drawlinestrip(vpoints[0],2,16,1.0f,Vector(0,0,1));
            if( !(!!_report->plink1 &&( _report->plink1->GetParent() == _ptargetbox) ) ) {
                if( _report->contacts.size() > 0 ) {
                    Vector v = _report->contacts.at(0).pos;
                    RAVELOG_VERBOSE_FORMAT("bad collision: %s: %f %f %f", _report->__str__()%v.x%v.y%v.z);
                }
                else {
                    RAVELOG_VERBOSE_FORMAT("bad collision: %s", _report->__str__());
                }
            }
            return !!_report->plink1 && _report->plink1->GetParent() == _ptargetbox;
        }

        bool _TestRayRigid(const Vector& v, const TransformMatrix& tcamera, const vector<KinBody::LinkPtr>& vattachedlinks)
        {
            dReal filen = 1/RaveSqrt(v.lengthsqr3());
            RAY r((_vf->_fRayMinDist*filen)*v,(2.0f*filen)*v);
            if( _vf->_robot->GetEnv()->CheckCollision(r,KinBodyConstPtr(_vf->_robot),_report) ) {
                //RAVELOG_INFO(str(boost::format("ray col: %s\n")%_report->__str__()));
                return false;
            }
            return true;
        }

        CollisionAction _IgnoreCollisionCallback(CollisionReportPtr preport, bool IsCalledFromPhysicsEngine)
        {
            if( _bSamplingRays ) {
                if( !!preport->plink1 ) {
                    if(  !preport->plink1->IsVisible() || preport->plink1->GetParent() == _vf->_sensorrobot ) {
                        return CA_Ignore;
                    }
                    // replaced the real target with convex hull
                    if( preport->plink1->GetParent() == _vf->_target ) {
                        return CA_Ignore;
                    }
                }
                if( !!preport->plink2 ) {
                    if( !preport->plink2->IsVisible() || preport->plink2->GetParent() == _vf->_sensorrobot ) {
                        return CA_Ignore;
                    }
                    // replaced the real target with convex hull
                    if( preport->plink2->GetParent() == _vf->_target ) {
                        return CA_Ignore;
                    }
                }
            }
            return CA_DefaultAction;
        }
        bool _bSamplingRays;
        boost::shared_ptr<VisualFeedback> _vf;
        KinBodyPtr _ptargetbox;         ///< box to represent the target for simulating ray collisions
        UserDataPtr _collisionfn;

        vector<OBB> _vTargetOBBs;         // object links local AABBs
        vector<dReal> _vsolution;
        IkReturnPtr _ikreturn;
        CollisionReportPtr _report;
        AABB _abTarget;         // target aabb
        vector<Vector> _vconvexplanes3d;
    };

    class GoalSampleFunction
    {
public:
        GoalSampleFunction(boost::shared_ptr<VisualFeedback> vf, const vector<Transform>& visibilitytransforms) : _vconstraint(vf), _fSampleGoalProb(1.0f), _vf(vf), _visibilitytransforms(visibilitytransforms)
        {
            RAVELOG_DEBUG(str(boost::format("have %d detection extents hypotheses\n")%_visibilitytransforms.size()));
            _ttarget = _vf->_target->GetTransform();
            _sphereperms.PermuteStart(_visibilitytransforms.size());
        }
        virtual ~GoalSampleFunction() {
        }

        virtual bool Sample(vector<dReal>& pNewSample)
        {
            if( RaveRandomFloat() > _fSampleGoalProb ) {
                return false;
            }
            RobotBase::RobotStateSaver state(_vf->_robot);
            _sphereperms._fn = boost::bind(&GoalSampleFunction::SampleWithParameters,this,_1,boost::ref(pNewSample));
            if( _sphereperms.PermuteContinue() >= 0 ) {
                return true;
            }
            //            // start from the beginning, if nothing, throw
            //            _sphereperms.PermuteStart(_visibilitytransforms.size());
            //            if( _sphereperms.PermuteContinue() >= 0 )
            //                return true;

            return false;
        }

        bool SampleWithParameters(int isample, vector<dReal>& pNewSample)
        {
            TransformMatrix tcamera = _ttarget*_visibilitytransforms.at(isample);
            return _vconstraint.SampleWithCamera(tcamera,pNewSample);
        }

        VisibilityConstraintFunction _vconstraint;
        dReal _fSampleGoalProb;
private:
        boost::shared_ptr<VisualFeedback> _vf;
        const vector<Transform>& _visibilitytransforms;


        Transform _ttarget;         ///< transform of target
        Vector _vTargetLocalCenter;
        RandomPermutationExecutor _sphereperms;
        vector<Transform> _vcameras;         ///< camera transformations in local coord systems
    };

    VisualFeedback(EnvironmentBasePtr penv) : ModuleBase(penv), _preport(new CollisionReport())
    {
        __description = ":Interface Author: Rosen Diankov\n\n\
.. image:: ../../../images/interface_visualfeedback.jpg\n\
  :width: 500\n\n\
Adds grasp planning taking into account camera visibility constraints. The relevant paper is:\n\n\
- Rosen Diankov, Takeo Kanade, James Kuffner. Integrating Grasp Planning and Visual Feedback for Reliable Manipulation, IEEE-RAS Intl. Conf. on Humanoid Robots, December 2009.\n\
\n\
Visibility computation checks occlusion with other objects using ray sampling in the image space:\n\n\
.. image:: ../../../images/interface_visualfeedback_occlusions.jpg\n\
  :height: 200\n\
";
        _fMaxVelMult=1;
        _bIgnoreSensorCollision = false;
        _bCameraOnManip = false;
        _fSampleRayDensity = 0.001;
        _fAllowableOcclusion = 0.1;
        _fRayMinDist = 0.02f;

        RegisterCommand("SetCameraAndTarget",boost::bind(&VisualFeedback::SetCameraAndTarget,this,_1,_2),
                        "Sets the camera index from the robot and its convex hull");
        RegisterCommand("ProcessVisibilityExtents",boost::bind(&VisualFeedback::ProcessVisibilityExtents,this,_1,_2),
                        "Processes the visibility extents of the target and initializes the camera transforms.\n\
\n\
:param sphere: Sets the transforms along a sphere density and the distances\n\
:param conedirangle: Prunes the currently set transforms along a cone centered at the local target center and directed towards conedirangle with a half-angle of ``|conedirangle|``. Can specify multiple cones for an OR effect.");
        RegisterCommand("SetCameraTransforms",boost::bind(&VisualFeedback::SetCameraTransforms,this,_1,_2),
                        "Sets new camera transformations. Can optionally choose a minimum distance from all planes of the camera convex hull (includes gripper mask)");
        RegisterCommand("ComputeVisibility",boost::bind(&VisualFeedback::ComputeVisibility,this,_1,_2),
                        "Computes the visibility of the current robot configuration");
        RegisterCommand("ComputeVisibleConfiguration",boost::bind(&VisualFeedback::ComputeVisibleConfiguration,this,_1,_2),
                        "Gives a camera transformation, computes the visibility of the object and returns the robot configuration that takes the camera to its specified position, otherwise returns false");
        RegisterCommand("SampleVisibilityGoal",boost::bind(&VisualFeedback::SampleVisibilityGoal,this,_1,_2),
                        "Samples a goal with the current manipulator maintaining camera visibility constraints");
        RegisterCommand("MoveToObserveTarget",boost::bind(&VisualFeedback::MoveToObserveTarget,this,_1,_2),
                        "Approaches a target object while choosing a goal such that the robot's camera sensor sees the object ");
        RegisterCommand("VisualFeedbackGrasping",boost::bind(&VisualFeedback::VisualFeedbackGrasping,this,_1,_2),
                        "Stochastic greedy grasp planner considering visibility");
        RegisterCommand("SetParameter",boost::bind(&VisualFeedback::SetParameter,this,_1,_2),
                        "Sets internal parameters of visibility computation");
    }

    virtual ~VisualFeedback() {
    }

    void Destroy()
    {
        _robot.reset();
        _sensorrobot.reset();
        _target.reset();
        _psensor.reset();
        _pmanip.reset();
        _pcamerageom.reset();
        _visibilitytransforms.clear();
        _pconstraintfn.reset();
        _preport.reset();
        ModuleBase::Destroy();
    }

    int main(const string& args)
    {
        stringstream ss(args);
        string robotname;
        _fMaxVelMult=1;
        _pconstraintfn.reset();
        ss >> robotname;
        _bIgnoreSensorCollision = false;
        string cmd;
        while(!ss.eof()) {
            ss >> cmd;
            if( !ss )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
            if( cmd == "maxvelmult" ) {
                ss >> _fMaxVelMult;
            }
            else if( cmd == "ignoresensorcollision" ) {
                ss >> _bIgnoreSensorCollision;
            }
            if( ss.fail() || !ss ) {
                break;
            }
        }
        _robot = GetEnv()->GetRobot(robotname);
        return 0;
    }

    virtual bool SendCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        return ModuleBase::SendCommand(sout,sinput);
    }

    bool SetCameraAndTarget(ostream& sout, istream& sinput)
    {
        _bCameraOnManip = false;
        _pmanip.reset();
        _psensor.reset();
        _vconvexplanes.resize(0);
        _pcamerageom.reset();
        _target.reset();
        RobotBase::AttachedSensorPtr psensor;
        RobotBase::ManipulatorPtr pmanip;
        _sensorrobot = _robot;
        bool bHasRayDensity = false;
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "sensorrobot" ) {
                string name;
                sinput >> name;
                _sensorrobot = GetEnv()->GetRobot(name);
            }
            else if( cmd == "sensorindex" ) {
                int sensorindex=-1;
                sinput >> sensorindex;
                psensor = _sensorrobot->GetAttachedSensors().at(sensorindex);
            }
            else if( cmd == "sensorname" ) {
                string sensorname;
                sinput >> sensorname;
                FOREACH(itsensor,_sensorrobot->GetAttachedSensors()) {
                    if( (*itsensor)->GetName() == sensorname ) {
                        psensor = *itsensor;
                        break;
                    }
                }
            }
            else if( cmd == "target" ) {
                string name;
                sinput >> name;
                _target = GetEnv()->GetKinBody(name);
            }
            else if( cmd == "manipname" ) {
                string manipname;
                sinput >> manipname;
                FOREACH(itmanip,_robot->GetManipulators()) {
                    if( (*itmanip)->GetName() == manipname ) {
                        pmanip = *itmanip;
                        break;
                    }
                }
            }
            else if( cmd == "convexdata" ) {
                int numpoints=0;
                sinput >> numpoints;
                vector<dReal> vconvexdata(2*numpoints);
                FOREACH(it, vconvexdata) {
                    sinput >> *it;
                }
                BOOST_ASSERT(vconvexdata.size() > 2 );
                vector<Vector> vpoints;
                Vector vprev,vprev2,v,vdir,vnorm,vcenter;
                for(size_t i = 0; i < vconvexdata.size(); i += 2 ) {
                    vpoints.push_back(Vector((vconvexdata[i]-_pcamerageom->KK.cx)/_pcamerageom->KK.fx,(vconvexdata[i+1]-_pcamerageom->KK.cy)/_pcamerageom->KK.fy,0,0));
                    vcenter += vpoints.back();
                }

                if( vpoints.size() > 2 ) {
                    vcenter *= 1.0f/vpoints.size();

                    // get the planes
                    _vconvexplanes.reserve(vpoints.size());
                    vprev = vpoints.back();
                    FOREACH(itv, vpoints) {
                        vdir = *itv-vprev;
                        vnorm.x = vdir.y;
                        vnorm.y = -vdir.x;
                        // normal has to be facing inside
                        if( vnorm.x*(vcenter.x-vprev.x)+vnorm.y*(vcenter.y-vprev.y) < 0 ) {
                            vnorm = -vnorm;
                        }
                        vnorm.z = -(vnorm.x*vprev.x+vnorm.y*vprev.y);
                        if( vnorm.lengthsqr3() > 1e-10 ) {
                            _vconvexplanes.push_back(vnorm.normalize3());
                        }
                        vprev = *itv;
                    }

                    // get the center of the convex hull
                    dReal totalarea = 0;
                    _vcenterconvex = Vector(0,0,0);
                    for(size_t i = 2; i < vpoints.size(); ++i) {
                        Vector v0 = vpoints[i-1]-vpoints[0];
                        Vector v1 = vpoints[i]-vpoints[0];
                        dReal area = RaveFabs(v0.x*v1.y-v0.y*v1.x);
                        _vcenterconvex += area*(vpoints[0]+vpoints[i-1]+vpoints[i-1]);
                        totalarea += area;
                    }
                    _vcenterconvex /= 3.0f*totalarea; _vcenterconvex.z = 1;
                }
                else {
                    RAVELOG_WARN(str(boost::format("convex data does not have enough points %d\n")%vconvexdata.size()));
                }
            }
            else if( cmd == "raydensity" ) {
                sinput >> _fSampleRayDensity;
                bHasRayDensity = true;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !psensor )
            return false;

        if( !psensor->GetSensor() ) {
            RAVELOG_WARN(str(boost::format("attached sensor %s does not have sensor interface\n")%psensor->GetName()));
            return false;
        }

        if( psensor->GetSensor()->GetSensorGeometry()->GetType() != SensorBase::ST_Camera) {
            RAVELOG_WARN(str(boost::format("attached sensor %s is not a camera")%psensor->GetName()));
            return false;
        }

        // check if there is a manipulator with the same end effector as camera
        if( _sensorrobot == _robot ) {
            std::vector<KinBody::LinkPtr> vattachedlinks;
            psensor->GetAttachingLink()->GetRigidlyAttachedLinks(vattachedlinks);
            if( !!pmanip ) {
                if(( pmanip->GetEndEffector() != psensor->GetAttachingLink()) &&( find(vattachedlinks.begin(),vattachedlinks.end(),pmanip->GetEndEffector()) == vattachedlinks.end()) ) {
                    RAVELOG_DEBUG(str(boost::format("specified manipulator %s end effector not attached to specified sensor %s\n")%pmanip->GetName()%psensor->GetName()));
                }
                else {
                    _bCameraOnManip = true;
                }
            }
            else {
                FOREACHC(itmanip,_robot->GetManipulators()) {
                    if(( (*itmanip)->GetEndEffector() == psensor->GetAttachingLink()) ||( find(vattachedlinks.begin(),vattachedlinks.end(),(*itmanip)->GetEndEffector()) != vattachedlinks.end()) ) {
                        pmanip = *itmanip;
                        _bCameraOnManip = true;
                        break;
                    }
                }

                if( !pmanip ) {
                    RAVELOG_WARN(str(boost::format("failed to find manipulator with end effector rigidly attached to sensor %s.\n")%psensor->GetName()));
                    pmanip = _robot->GetActiveManipulator();
                }
            }

            _ttogripper = psensor->GetTransform().inverse()*pmanip->GetTransform();
        }
        else {
            if( !pmanip ) {
                pmanip = _robot->GetActiveManipulator();
            }

            if( !!_target ) {
                _ttogripper = _target->GetTransform().inverse() * pmanip->GetTransform();
            }
        }

        _pcamerageom = boost::static_pointer_cast<SensorBase::CameraGeomData const>(psensor->GetSensor()->GetSensorGeometry());
        if( !bHasRayDensity ) {
            _fSampleRayDensity = 20.0f/_pcamerageom->KK.fx;
        }

        if( _vconvexplanes.size() == 0 ) {
            // pick the camera boundaries
            _vconvexplanes.push_back(Vector(_pcamerageom->KK.fx,0,_pcamerageom->KK.cx,0).normalize3());     // -x
            _vconvexplanes.push_back(Vector(-_pcamerageom->KK.fx,0,_pcamerageom->width-_pcamerageom->KK.cx,0).normalize3());     // +x
            _vconvexplanes.push_back(Vector(0,_pcamerageom->KK.fy,_pcamerageom->KK.cy,0).normalize3());     // -y
            _vconvexplanes.push_back(Vector(0,-_pcamerageom->KK.fy,_pcamerageom->height-_pcamerageom->KK.cy,0).normalize3());     // +y
            _vcenterconvex = Vector(0,0,1);
        }

        _pmanip = pmanip;
        _psensor = psensor;
        sout << _pmanip->GetName();
        return true;
    }

    TransformMatrix ComputeCameraMatrix(const Vector& vdir,dReal fdist,dReal froll)
    {
        Vector vright, vup = Vector(0,1,0) - vdir * vdir.y;
        dReal uplen = vup.lengthsqr3();
        if( uplen < 0.001 ) {
            vup = Vector(0,0,1) - vdir * vdir.z;
            uplen = vup.lengthsqr3();
        }

        vup *= (dReal)1.0/RaveSqrt(uplen);
        vright = vup.cross(vdir);
        TransformMatrix tcamera;
        tcamera.m[2] = vdir.x; tcamera.m[6] = vdir.y; tcamera.m[10] = vdir.z;

        dReal fcosroll = RaveCos(froll), fsinroll = RaveSin(froll);
        tcamera.m[0] = vright.x*fcosroll+vup.x*fsinroll; tcamera.m[1] = -vright.x*fsinroll+vup.x*fcosroll;
        tcamera.m[4] = vright.y*fcosroll+vup.y*fsinroll; tcamera.m[5] = -vright.y*fsinroll+vup.y*fcosroll;
        tcamera.m[8] = vright.z*fcosroll+vup.z*fsinroll; tcamera.m[9] = -vright.z*fsinroll+vup.z*fcosroll;
        tcamera.trans = -fdist * tcamera.rotate(_vcenterconvex);
        return tcamera;
    }

    bool ProcessVisibilityExtents(ostream& sout, istream& sinput)
    {
        string cmd;
        Vector vTargetLocalCenter;
        bool bSetTargetCenter = false;
        int numrolls=8;
        vector<Vector> vconedirangles;
        vector<Transform> vtransforms;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "localtargetcenter" ) {
                sinput >> vTargetLocalCenter.x >> vTargetLocalCenter.y >> vTargetLocalCenter.z;
                bSetTargetCenter = true;
            }
            else if( cmd == "transforms" ) {
                int numtrans=0;
                sinput >> numtrans;
                vtransforms.resize(numtrans);
                FOREACH(it,vtransforms) {
                    sinput >> *it;
                }
            }
            else if( cmd == "numrolls" )
                sinput >> numrolls;
            else if( cmd == "extents" ) {
                if( !bSetTargetCenter && !!_target ) {
                    KinBody::KinBodyStateSaver saver(_target);
                    _target->SetTransform(Transform());
                    vTargetLocalCenter = _target->ComputeAABB().pos;
                }
                int numtrans=0;
                sinput >> numtrans;
                dReal deltaroll = PI*2.0f/(dReal)numrolls;
                vtransforms.resize(0); vtransforms.reserve(numtrans*numrolls);
                for(int i = 0; i < numtrans; ++i) {
                    Vector v;
                    sinput >> v.x >> v.y >> v.z;
                    dReal fdist = RaveSqrt(v.lengthsqr3());
                    v *= 1/fdist;
                    dReal froll = 0;
                    for(int iroll = 0; iroll < numrolls; ++iroll, froll += deltaroll) {
                        Transform t = ComputeCameraMatrix(v,fdist,froll); t.trans += vTargetLocalCenter;
                        vtransforms.push_back(t);
                    }
                }
            }
            else if( cmd == "sphere" || cmd == "invertsphere" ) {
                if( !bSetTargetCenter && !!_target ) {
                    KinBody::KinBodyStateSaver saver(_target);
                    _target->SetTransform(Transform());
                    vTargetLocalCenter = _target->ComputeAABB().pos;
                }

                TriMesh spheremesh;
                int spherelevel = 3, numdists = 0;
                sinput >> spherelevel >> numdists;
                CM::GenerateSphereTriangulation(spheremesh,spherelevel);
                vector<dReal> vdists(numdists);
                FOREACH(it,vdists) {
                    sinput >> *it;
                }
                bool bInvert = cmd == "invertsphere";
                dReal deltaroll = PI*2.0f/(dReal)numrolls;
                vtransforms.resize(spheremesh.vertices.size()*numdists*numrolls);
                vector<Transform>::iterator itcamera = vtransforms.begin();
                for(size_t j = 0; j < spheremesh.vertices.size(); ++j) {
                    Vector v = spheremesh.vertices[j];
                    for(int i = 0; i < numdists; ++i) {
                        dReal froll = 0;
                        for(int iroll = 0; iroll < numrolls; ++iroll, froll += deltaroll) {
                            *itcamera = ComputeCameraMatrix(spheremesh.vertices[j],vdists[i],froll);
                            itcamera->trans += vTargetLocalCenter;
                            if( bInvert ) {
                                *itcamera = itcamera->inverse();
                            }
                            ++itcamera;
                        }
                    }
                }
            }
            else if( cmd == "conedirangle" ) {
                Vector vconedir; dReal fangle;
                sinput >> vconedir.x >> vconedir.y >> vconedir.z;
                fangle = RaveSqrt(vconedir.lengthsqr3());
                if( fangle == 0 ) {
                    return false;
                }

                vconedir /= fangle;
                vconedir.w = RaveCos(fangle);
                vconedirangles.push_back(vconedir);
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( vconedirangles.size() > 0 ) {
            vector<Transform> voldtransforms;
            voldtransforms.swap(vtransforms);
            vtransforms.reserve(voldtransforms.size());
            FOREACH(itt,voldtransforms) {
                Vector v = itt->trans-vTargetLocalCenter;
                bool bInCone = false;
                FOREACH(itcone, vconedirangles) {
                    if( itcone->dot3(v) >= itcone->w*RaveSqrt(v.lengthsqr3()) ) {
                        bInCone = true;
                        break;
                    }
                }
                if( bInCone ) {
                    vtransforms.push_back(*itt);
                }
            }
        }

        KinBody::KinBodyStateSaver saver(_target,KinBody::Save_LinkTransformation);
        _target->SetTransform(Transform());
        boost::shared_ptr<VisibilityConstraintFunction> pconstraintfn(new VisibilityConstraintFunction(shared_problem()));

        // get all the camera positions and test them
        FOREACHC(itcamera, vtransforms) {
            Transform tCameraInTarget = *itcamera;
            Transform tTargetInWorld;
            if( _sensorrobot == _robot ) {
                //BOOST_ASSERT(0);
                tTargetInWorld = _sensorrobot->GetTransform() * tCameraInTarget.inverse();
            }
            else {
                tTargetInWorld = _sensorrobot->GetTransform() * tCameraInTarget.inverse();
            }

            if( pconstraintfn->InConvexHull(*itcamera) ) {
                if( !_pmanip->CheckEndEffectorCollision(tTargetInWorld*_ttogripper, _preport) ) {
                    if( !pconstraintfn->IsOccludedByRigid(*itcamera) ) {
                        sout << *itcamera << " ";
                    }
                    else {
                        RAVELOG_VERBOSE("in convex hull and effector is free, but not occluded by rigid\n");
                    }
                }
                else {
                    RAVELOG_VERBOSE_FORMAT("in convex hull, but end effector collision: %s", _preport->__str__());
                }
            }
        }

        return true;
    }

    bool SetCameraTransforms(ostream& sout, istream& sinput)
    {
        string cmd;
        _visibilitytransforms.resize(0);
        dReal mindist = 0;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "transforms" ) {
                size_t numtrans=0;
                sinput >> numtrans;
                _visibilitytransforms.resize(numtrans);
                Transform t;
                for(size_t i =0; i < numtrans; ++i) {
                    sinput >> _visibilitytransforms[i];
                }
            }
            else if( cmd == "mindist" ) {
                sinput >> mindist;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( mindist != 0 ) {
            KinBody::KinBodyStateSaver saver(_target);
            _target->SetTransform(Transform());
            boost::shared_ptr<VisibilityConstraintFunction> pconstraintfn(new VisibilityConstraintFunction(shared_problem()));
            vector<Transform> visibilitytransforms; visibilitytransforms.swap(_visibilitytransforms);
            _visibilitytransforms.reserve(visibilitytransforms.size());
            FOREACH(it,visibilitytransforms) {
                if( pconstraintfn->InConvexHull(*it,mindist) ) {
                    _visibilitytransforms.push_back(*it);
                }
            }
        }

        return true;
    }

    bool ComputeVisibility(ostream& sout, istream& sinput)
    {
        bool bcheckocclusion = true;
        sinput >> bcheckocclusion;

        RobotBase::RobotStateSaver saver(_robot);
        _robot->SetActiveManipulator(_pmanip);
        _robot->SetActiveDOFs(_pmanip->GetArmIndices());

        if( !_pconstraintfn ) {
            _pconstraintfn.reset(new VisibilityConstraintFunction(shared_problem()));
        }
        sout << _pconstraintfn->IsVisible(bcheckocclusion);
        return true;
    }

    bool SetParameter(ostream& sout, istream& sinput)
    {
        string cmd;
        Transform t;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "raydensity" ) {
                sinput >> _fSampleRayDensity;
            }
            else if( cmd == "raymindist") {
                sinput >> _fRayMinDist;
            }
            else if( cmd == "allowableocclusion" ) {
                sinput >> _fAllowableOcclusion;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }
        return true;
    }

    bool ComputeVisibleConfiguration(ostream& sout, istream& sinput)
    {
        string cmd;
        Transform t;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "pose" ) {
                sinput >> t;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        vector<dReal> vsample;
        RobotBase::RobotStateSaver saver(_robot);
        _robot->SetActiveManipulator(_pmanip);
        _robot->SetActiveDOFs(_pmanip->GetArmIndices());
        boost::shared_ptr<VisibilityConstraintFunction> pconstraintfn(new VisibilityConstraintFunction(shared_problem()));
        if( _pmanip->CheckEndEffectorCollision(t*_ttogripper, _preport) ) {
            RAVELOG_VERBOSE_FORMAT("endeffector is in collision, %s\n",_preport->__str__());
            return false;
        }
        if( !pconstraintfn->SampleWithCamera(t,vsample) ) {
            return false;
        }
        FOREACH(it,vsample) {
            sout << *it << " ";
        }
        return true;
    }

    bool SampleVisibilityGoal(ostream& sout, istream& sinput)
    {
        string cmd;

        int numsamples=1;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "numsamples" ) {
                sinput >> numsamples;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver(_robot);
        _robot->SetActiveManipulator(_pmanip);
        _robot->SetActiveDOFs(_pmanip->GetArmIndices());

        if( _pmanip->CheckIndependentCollision(_preport) ) {
            RAVELOG_WARN(str(boost::format("robot independent links in collision: %s\n")%_preport->__str__()));
            return false;
        }

        boost::shared_ptr<GoalSampleFunction> pgoalsampler(new GoalSampleFunction(shared_problem(),_visibilitytransforms));

        uint64_t starttime = utils::GetMicroTime();
        vector<dReal> vsample;
        vector<dReal> vsamples(_robot->GetActiveDOF()*numsamples);
        int numsampled = 0;
        for(int i = 0; i < numsamples; ++i) {
            if( pgoalsampler->Sample(vsample) ) {
                std::copy(vsample.begin(), vsample.end(),vsamples.begin()+i*_robot->GetActiveDOF());
                numsampled++;
            }
        }

        if( numsampled == 0 ) {
            return false;
        }
        float felapsed = (utils::GetMicroTime()-starttime)*1e-6f;
        RAVELOG_INFO("total time for %d samples is %fs, %f avg\n", numsamples,felapsed,felapsed/numsamples);
        sout << numsampled << " ";
        for(int i = 0; i < numsampled*_robot->GetActiveDOF(); ++i) {
            sout << vsamples[i] << " ";
        }
        return true;
    }

    bool MoveToObserveTarget(ostream& sout, istream& sinput)
    {
        string strtrajfilename;
        bool bExecute = true;
        boost::shared_ptr<ostream> pOutputTrajStream;

        PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
        params->_nMaxIterations = 4000;
        int affinedofs=0;
        string cmd, plannername="BiRRT";
        dReal fSampleGoalProb = 0.001f;

        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "outputtraj" ) {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,utils::null_deleter());
            }
            else if( cmd == "affinedofs" ) {
                sinput >> affinedofs;
            }
            else if( cmd == "maxiter" ) {
                sinput >> params->_nMaxIterations;
            }
            else if( cmd == "execute" ) {
                sinput >> bExecute;
            }
            else if( cmd == "writetraj" ) {
                sinput >> strtrajfilename;
            }
            else if( cmd == "smoothpath" ) {
                sinput >> params->_sPostProcessingPlanner;
            }
            else if( cmd == "planner" ) {
                sinput >> plannername;
            }
            else if( cmd == "sampleprob" ) {
                sinput >> fSampleGoalProb;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver(_robot);
        _robot->SetActiveManipulator(_pmanip);
        _robot->SetActiveDOFs(_pmanip->GetArmIndices(), affinedofs);

        boost::shared_ptr<GoalSampleFunction> pgoalsampler(new GoalSampleFunction(shared_problem(),_visibilitytransforms));
        pgoalsampler->_fSampleGoalProb = fSampleGoalProb;
        _robot->RegrabAll();

        params->SetRobotActiveJoints(_robot);
        _robot->GetActiveDOFValues(params->vinitialconfig);

        params->_samplegoalfn = boost::bind(&GoalSampleFunction::Sample,pgoalsampler,_1);
        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
        ptraj->Init(_robot->GetActiveConfigurationSpecification());
        ptraj->Insert(0,params->vinitialconfig);

        // jitter for initial collision
        if( !planningutils::JitterActiveDOF(_robot) ) {
            RAVELOG_WARN("jitter failed for initial\n");
            return false;
        }
        _robot->GetActiveDOFValues(params->vinitialconfig);

        PlannerBasePtr planner = RaveCreatePlanner(GetEnv(),plannername);
        if( !planner ) {
            RAVELOG_ERROR("failed to create BiRRTs\n");
            return false;
        }

        bool bSuccess = false;
        RAVELOG_INFOA("starting planning\n");
        uint64_t starttime = utils::GetMicroTime();
        for(int iter = 0; iter < 1; ++iter) {
            if( !planner->InitPlan(_robot, params) ) {
                RAVELOG_ERROR("InitPlan failed\n");
                return false;
            }
            if( planner->PlanPath(ptraj) ) {
                bSuccess = true;
                RAVELOG_INFOA("finished planning\n");
                break;
            }
            else {
                RAVELOG_WARN("PlanPath failed\n");
            }
        }

        float felapsed = (utils::GetMicroTime()-starttime)*0.000001f;
        RAVELOG_INFOA("total planning time: %fs\n", felapsed);
        if( !bSuccess ) {
            return false;
        }
        if( RaveGetDebugLevel() & Level_VerifyPlans ) {
            planningutils::VerifyTrajectory(params,ptraj);
        }
        CM::SetActiveTrajectory(_robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool VisualFeedbackGrasping(ostream& sout, istream& sinput)
    {
        string strtrajfilename;
        bool bExecute = true;
        boost::shared_ptr<ostream> pOutputTrajStream;

        boost::shared_ptr<GraspSetParameters> params(new GraspSetParameters(GetEnv()));
        params->_nMaxIterations = 4000;
        string cmd, plannername="GraspGradient";
        params->_fVisibiltyGraspThresh = 0.05f;
        bool bUseVisibility = false;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "outputtraj" ) {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,utils::null_deleter());
            }
            else if( cmd == "maxiter" ) {
                sinput >> params->_nMaxIterations;
            }
            else if( cmd == "visgraspthresh" ) {
                sinput >> params->_fVisibiltyGraspThresh;
            }
            else if( cmd == "execute" ) {
                sinput >> bExecute;
            }
            else if( cmd == "writetraj" ) {
                sinput >> strtrajfilename;
            }
            else if( cmd == "usevisibility" ) {
                sinput >> bUseVisibility;
            }
            else if( cmd == "planner" ) {
                sinput >> plannername;
            }
            else if( cmd == "graspdistthresh" ) {
                sinput >> params->_fGraspDistThresh;
            }
            else if( cmd == "graspset" ) {
                int numgrasps=0;
                sinput >> numgrasps;
                params->_vgrasps.resize(numgrasps);
                TransformMatrix t;
                FOREACH(itgrasp,params->_vgrasps) {
                    sinput >> t;
                    *itgrasp = t;
                }
                RAVELOG_DEBUG(str(boost::format("grasp set size = %d\n")%params->_vgrasps.size()));
            }
            else if( cmd == "numgradientsamples" ) {
                sinput >> params->_nGradientSamples;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                return false;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver(_robot);
        _robot->SetActiveManipulator(_pmanip);
        _robot->SetActiveDOFs(_pmanip->GetArmIndices());
        params->SetRobotActiveJoints(_robot);

        if( bUseVisibility ) {
            RAVELOG_DEBUG("using visibility constraints\n");
            boost::shared_ptr<VisibilityConstraintFunction> pconstraint(new VisibilityConstraintFunction(shared_problem()));
            params->_checkpathconstraintsfn = boost::bind(&VisibilityConstraintFunction::Constraint,pconstraint,params->_checkpathconstraintsfn,_1,_2,_3,_4);
        }

        params->_ptarget = _target;
        _robot->GetActiveDOFValues(params->vinitialconfig);

        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
        ptraj->Init(_robot->GetActiveConfigurationSpecification());
        PlannerBasePtr planner = RaveCreatePlanner(GetEnv(),plannername);
        if( !planner ) {
            RAVELOG_ERROR("failed to create BiRRTs\n");
            return false;
        }

        bool bSuccess = false;
        RAVELOG_INFOA("starting planning\n");
        uint64_t starttime = utils::GetMicroTime();
        if( !planner->InitPlan(_robot, params) ) {
            RAVELOG_ERROR("InitPlan failed\n");
            return false;
        }

        if( planner->PlanPath(ptraj) ) {
            bSuccess = true;
        }
        else {
            RAVELOG_WARN("PlanPath failed\n");
        }

        float felapsed = (utils::GetMicroTime()-starttime)*0.000001f;
        RAVELOG_INFOA("total planning time: %fs\n", felapsed);
        if( !bSuccess ) {
            return false;
        }
        if( RaveGetDebugLevel() & Level_VerifyPlans ) {
            planningutils::VerifyTrajectory(params,ptraj);
        }
        CM::SetActiveTrajectory(_robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool SimulationStep(dReal fElapsedTime)
    {
        return false;
    }

protected:
    RobotBasePtr _robot, _sensorrobot;
    bool _bIgnoreSensorCollision; ///< if true will ignore any collisions with vf->_sensorrobot
    KinBodyPtr _target; ///< the target to check visibility for. Only links that are visible are checked
    dReal _fMaxVelMult;
    RobotBase::AttachedSensorPtr _psensor;
    RobotBase::ManipulatorPtr _pmanip;
    bool _bCameraOnManip;     ///< true if camera is attached to manipulator
    SensorBase::CameraGeomDataConstPtr _pcamerageom;
    Transform _ttogripper;     ///< transforms a coord system to the gripper coordsystem
    vector<Transform> _visibilitytransforms;
    dReal _fRayMinDist, _fAllowableOcclusion, _fSampleRayDensity;

    CollisionReportPtr _preport;

    boost::shared_ptr<VisibilityConstraintFunction> _pconstraintfn;

    vector<Vector> _vconvexplanes;     ///< the planes defining the bounding visibility region (posive is inside)
    Vector _vcenterconvex;     ///< center point on the z=1 plane of the convex region
};

ModuleBasePtr CreateVisualFeedback(EnvironmentBasePtr penv) {
    return ModuleBasePtr(new VisualFeedback(penv));
}
