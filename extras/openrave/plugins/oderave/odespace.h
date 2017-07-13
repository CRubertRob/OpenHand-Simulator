// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVE_ODE_SPACE
#define OPENRAVE_ODE_SPACE

//#include <boost/thread/tss.hpp>

// manages a space of ODE objects
class ODESpace : public boost::enable_shared_from_this<ODESpace>
{
    static void DummySetParam(dJointID id, int param, dReal)
    {
        RAVELOG_WARN(str(boost::format("failed to set param to dummy %d\n")%dJointGetType(id)));
    }

    //    static void AllocateODEResources()
    //    {
    //#ifdef ODE_HAVE_ALLOCATE_DATA_THREAD
    //        static boost::thread_specific_ptr<bool> isalloc;
    //        if( !isalloc.get() ) {
    //            RAVELOG_INFO("allocating ode resources\n");
    //            isalloc.reset(new bool(true));
    //            dAllocateODEDataForThread(dAllocateMaskAll);
    //        }
    //#endif
    //    }
    inline boost::weak_ptr<ODESpace> weak_space() {
        return shared_from_this();
    }

    class ODEResources
    {
public:
        ODEResources() {
#ifdef ODE_HAVE_ALLOCATE_DATA_THREAD
            dAllocateODEDataForThread(dAllocateMaskAll);
#endif
            world = dWorldCreate();
            space = dHashSpaceCreate(0);
            contactgroup = dJointGroupCreate(0);
        }
        virtual ~ODEResources() {
            if( contactgroup ) {
                dJointGroupDestroy(contactgroup);
            }
            if( space ) {
                dSpaceDestroy(space);
            }
            if( world ) {
                dWorldDestroy(world);
            }
#ifdef ODE_HAVE_ALLOCATE_DATA_THREAD
            dCleanupODEAllDataForThread();
#endif
        }
        dWorldID world;          ///< the dynamics world
        dSpaceID space;          ///< the collision world
        dJointGroupID contactgroup;
        boost::mutex _mutex;
    };

public:
    // information about the kinematics of the body
    class KinBodyInfo : public boost::enable_shared_from_this<KinBodyInfo>, public OpenRAVE::UserData
    {
public:
        struct LINK
        {
            LINK() : body(NULL), geom(NULL), _bEnabled(true) {
            }
            virtual ~LINK() {
                BOOST_ASSERT(listtrimeshinds.size()==0&&listvertices.size()==0&&body==NULL&&geom==NULL);
            }

            dBodyID body;
            dGeomID geom;

            void Enable(bool bEnable)
            {
                if( GetLink()->IsStatic() || _bEnabled == bEnable ) {
                    return;
                }
                _bEnabled = bEnable;
                if( body ) {
                    if( bEnable) {
                        dBodyEnable(body);
                    }
                    else {
                        dBodyDisable(body);
                    }
                }

                dGeomID curgeom = geom;
                while(curgeom != NULL ) {
                    if( bEnable ) {
                        dGeomEnable(curgeom);
                    }
                    else {
                        dGeomDisable(curgeom);
                    }
                    if( dGeomGetClass(curgeom) == dGeomTransformClass ) {
                        if( bEnable ) {
                            dGeomEnable(dGeomTransformGetGeom(curgeom));
                        }
                        else {
                            dGeomDisable(dGeomTransformGetGeom(curgeom));
                        }
                    }
                    curgeom = dBodyGetNextGeom(curgeom);
                }
            }

            KinBody::LinkPtr GetLink() {
                return _plink.lock();
            }

            list<dTriIndex*> listtrimeshinds;
            list<dReal*> listvertices;
            KinBody::LinkWeakPtr _plink;
            bool _bEnabled;
            Transform tlinkmass, tlinkmassinv; // the local mass frame ODE was initialized with
            std::string bodylinkname; // for debugging purposes
        };

        KinBodyInfo(boost::shared_ptr<ODEResources> ode) : _ode(ode)
        {
            jointgroup = dJointGroupCreate(0);
            space = dHashSpaceCreate(_ode->space);
            nLastStamp = 0;
        }

        virtual ~KinBodyInfo() {
            boost::mutex::scoped_lock lock(_ode->_mutex);
            Reset();
            dSpaceClean(space);
            dJointGroupEmpty(jointgroup);

            dSpaceDestroy(space);
            dJointGroupDestroy(jointgroup);
        }

        void Reset()
        {
            FOREACH(itlink, vlinks) {
                dGeomID curgeom = (*itlink)->geom;
                while(curgeom) {
                    dGeomID pnextgeom = dBodyGetNextGeom(curgeom);
                    if( dGeomGetClass(curgeom) == dGeomTransformClass ) {
                        dGeomID childgeom = dGeomTransformGetGeom(curgeom);
                        if(( childgeom != NULL) &&( dGeomGetClass(childgeom) == dTriMeshClass) ) {
                            if( dGeomTriMeshGetData(childgeom) != 0 ) {
                                dGeomTriMeshDataDestroy(dGeomTriMeshGetData(childgeom));
                            }
                        }
                    }
                    else if( dGeomGetClass(curgeom) == dTriMeshClass ) {
                        if( dGeomTriMeshGetData(curgeom) != 0 ) {
                            dGeomTriMeshDataDestroy(dGeomTriMeshGetData(curgeom));
                        }
                    }
                    dGeomDestroy(curgeom);
                    curgeom = pnextgeom;
                }
                (*itlink)->geom = NULL;
                if( (*itlink)->body ) {
                    dBodyDestroy((*itlink)->body);
                    (*itlink)->body = NULL;
                }
                FOREACH(itind, (*itlink)->listtrimeshinds) {
                    delete[] *itind;
                }
                (*itlink)->listtrimeshinds.clear();
                FOREACH(itind, (*itlink)->listvertices) {
                    delete[] *itind;
                }
                (*itlink)->listvertices.clear();
                (*itlink)->_bEnabled = false;
            }
            vlinks.resize(0);

            FOREACH(itjoint, vjoints) {
                if( *itjoint ) {
                    dJointDestroy(*itjoint);
                }
            }
            vjoints.resize(0);

            _geometrycallback.reset();
            _staticcallback.reset();
        }

        KinBodyPtr GetBody() {
            return _pbody.lock();
        }

        KinBodyWeakPtr _pbody;         ///< body associated with this structure
        int nLastStamp;

        vector<boost::shared_ptr<LINK> > vlinks;         ///< if body is disabled, then geom is static (it can't be connected to a joint!)
        vector<OpenRAVE::dReal> _vdofbranches;

        ///< the pointer to this Link is the userdata
        vector<dJointID> vjoints;
        vector<dJointFeedback> vjointfeedback;
        OpenRAVE::UserDataPtr _geometrycallback, _staticcallback;
        boost::weak_ptr<ODESpace> _odespace;

        dSpaceID space;                             ///< space that contanis all the collision objects of this chain
        dJointGroupID jointgroup;

private:
        boost::shared_ptr<ODEResources> _ode;
    };

    typedef boost::shared_ptr<KinBodyInfo> KinBodyInfoPtr;
    typedef boost::shared_ptr<KinBodyInfo const> KinBodyInfoConstPtr;
    typedef boost::function<void (KinBodyInfoPtr)> SynchronizeCallbackFn;

    ODESpace(EnvironmentBasePtr penv, const std::string& userdatakey, bool bUsingPhysics) : _penv(penv), _userdatakey(userdatakey), _bUsingPhysics(bUsingPhysics)
    {
        static bool s_bIsODEInitialized = false;
        if( !s_bIsODEInitialized ) {
            s_bIsODEInitialized = true;
            dInitODE();
        }

        memset(_jointset, 0, sizeof(_jointset));
        _jointset[dJointTypeBall] = DummySetParam;
        _jointset[dJointTypeHinge] = dJointSetHingeParam;
        _jointset[dJointTypeSlider] = dJointSetSliderParam;
        _jointset[dJointTypeUniversal] = dJointSetUniversalParam;
        _jointset[dJointTypeHinge2] = dJointSetHinge2Param;
    }

    virtual ~ODESpace() {
        DestroyEnvironment();
        Destroy();
    }

    bool Init()
    {
#ifdef ODE_HAVE_ALLOCATE_DATA_THREAD
        dAllocateODEDataForThread(dAllocateMaskAll);
#endif

        RAVELOG_VERBOSE("init ode collision environment\n");
        _ode.reset(new ODEResources());
        return true;
    }

    void Destroy()
    {
        DestroyEnvironment();
        _ode.reset();
    }

    void DestroyEnvironment()
    {
        RAVELOG_VERBOSE("destroying ode collision environment\n");
        // go through all the initialized KinBodies
        FOREACH(itbody, _setInitializedBodies) {
            (*itbody)->RemoveUserData(_userdatakey);
        }
        _setInitializedBodies.clear();
    }

    bool IsInitialized() {
        return !!_ode;
    }

    KinBodyInfoPtr InitKinBody(KinBodyConstPtr pbody, KinBodyInfoPtr pinfo = KinBodyInfoPtr(), bool blockode=true)
    {
        EnvironmentMutex::scoped_lock lock(pbody->GetEnv()->GetMutex());
        boost::shared_ptr<boost::mutex::scoped_lock> lockode;
        if( blockode ) {
            lockode.reset(new boost::mutex::scoped_lock(_ode->_mutex));
        }
#ifdef ODE_HAVE_ALLOCATE_DATA_THREAD
        dAllocateODEDataForThread(dAllocateMaskAll);
#endif

        // create all ode bodies and joints
        if( !pinfo ) {
            pinfo.reset(new KinBodyInfo(_ode));
        }
        pinfo->Reset();
        pinfo->_pbody = boost::const_pointer_cast<KinBody>(pbody);
        pinfo->_odespace = weak_space();
        pinfo->vlinks.reserve(pbody->GetLinks().size());
        pinfo->vjoints.reserve(pbody->GetJoints().size()+pbody->GetPassiveJoints().size());
        pinfo->vjointfeedback.resize(pinfo->vjoints.capacity());
        dGeomSetData((dGeomID)pinfo->space, pinfo.get());     // so that the kinbody can be retreived from the space

        pinfo->vlinks.reserve(pbody->GetLinks().size());
        FOREACHC(itlink, pbody->GetLinks()) {
            boost::shared_ptr<KinBodyInfo::LINK> link(new KinBodyInfo::LINK());
            pinfo->vlinks.push_back(link);
            link->body = dBodyCreate(GetWorld());

            if( (*itlink)->IsStatic() ) {
                dBodyDisable(link->body);
                link->_bEnabled = false;
            }

            link->tlinkmass = (*itlink)->GetLocalMassFrame();
            link->tlinkmassinv = link->tlinkmass.inverse();

            if( _geometrygroup.size() > 0 && (*itlink)->GetGroupNumGeometries(_geometrygroup) >= 0 ) {
                // add a geometry object group
                const std::vector<KinBody::GeometryInfoPtr>& vgeometryinfos = (*itlink)->GetGeometriesFromGroup(_geometrygroup);
                FOREACHC(itgeominfo, vgeometryinfos) {
                    dGeomID odegeomtrans = _CreateODEGeomFromGeometryInfo(pinfo->space, link, **itgeominfo);
                    if( !odegeomtrans ) {
                        continue;
                    }
                    // finally set the geom to the ode body
                    dGeomSetBody(odegeomtrans, link->body);
                    link->geom = odegeomtrans; // is it ok that we're overwriting this?
                }
            }
            else {
                // add all the current geometry objects
                FOREACHC(itgeom, (*itlink)->GetGeometries()) {
                    dGeomID odegeomtrans = _CreateODEGeomFromGeometryInfo(pinfo->space, link, (*itgeom)->GetInfo());
                    if( !odegeomtrans ) {
                        continue;
                    }
                    // finally set the geom to the ode body
                    dGeomSetBody(odegeomtrans, link->body);
                    link->geom = odegeomtrans; // is it ok that we're overwriting this?
                }
            }

            if( !(*itlink)->IsStatic() && _bUsingPhysics ) {
                // set the mass
                dMass mass;
                dMassSetZero(&mass);
                mass.mass = (*itlink)->GetMass();
                if( mass.mass <= 0 ) {
                    RAVELOG_WARN(str(boost::format("body %s:%s mass is %f. filling dummy values")%pbody->GetName()%(*itlink)->GetName()%mass.mass));
                    mass.mass = 1e-7;
                    mass.I[0] = 1; mass.I[1] = 0; mass.I[2] = 0;
                    mass.I[4] = 0; mass.I[5] = 1; mass.I[6] = 0;
                    mass.I[8] = 0; mass.I[9] = 0; mass.I[10] = 1;
                }
                else {
                    Vector vprincipalmoments = (*itlink)->GetPrincipalMomentsOfInertia();
                    if( vprincipalmoments.lengthsqr3() > 0 ) {
                        mass.I[0] = vprincipalmoments.x; mass.I[1] = 0; mass.I[2] = 0;
                        mass.I[4] = 0; mass.I[5] = vprincipalmoments.y; mass.I[6] = 0;
                        mass.I[8] = 0; mass.I[9] = 0; mass.I[10] = vprincipalmoments.z;
                    }
                    else {
                        mass.I[0] = 1; mass.I[1] = 0; mass.I[2] = 0;
                        mass.I[4] = 0; mass.I[5] = 1; mass.I[6] = 0;
                        mass.I[8] = 0; mass.I[9] = 0; mass.I[10] = 1;
                    }
                }
                // ODE link coordinate system coincides with center of mass
                dBodySetMass(link->body, &mass);
            }

            link->_plink = *itlink;
            link->bodylinkname = pbody->GetName() + "/" + (*itlink)->GetName();
            // Calculate ODE transform consisting of link origin + center of mass offset
            RaveTransform<dReal> t = (*itlink)->GetTransform() * link->tlinkmass;
            dBodySetPosition(link->body,t.trans.x, t.trans.y, t.trans.z);
            BOOST_ASSERT( RaveFabs(t.rot.lengthsqr4()-1) < 0.0001f );
            dBodySetQuaternion(link->body,&t.rot[0]);
            dBodySetData(link->body, link.get());     // so that the link can be retreived from the body
        }

        if( _bUsingPhysics ) {
            vector<KinBody::JointPtr> vbodyjoints = pbody->GetJoints();
            vbodyjoints.insert(vbodyjoints.end(),pbody->GetPassiveJoints().begin(),pbody->GetPassiveJoints().end());
            FOREACHC(itjoint, vbodyjoints) {
                //bool bPassive = (*itjoint)->GetJointIndex()<0;
                RaveVector<dReal> anchor = (*itjoint)->GetAnchor();
                RaveVector<dReal> axis0 = -(*itjoint)->GetAxis(0);
                RaveVector<dReal> axis1 = -(*itjoint)->GetAxis(1);
                dBodyID body0 = (!!(*itjoint)->GetFirstAttached() && !(*itjoint)->GetFirstAttached()->IsStatic()) ? pinfo->vlinks[(*itjoint)->GetFirstAttached()->GetIndex()]->body : NULL;
                dBodyID body1 = (!!(*itjoint)->GetSecondAttached() && !(*itjoint)->GetSecondAttached()->IsStatic()) ? pinfo->vlinks[(*itjoint)->GetSecondAttached()->GetIndex()]->body : NULL;

                dJointID joint = NULL;
                switch((*itjoint)->GetType()) {
                case KinBody::JointHinge:
                    joint = dJointCreateHinge(GetWorld(), pinfo->jointgroup);
                    break;
                case KinBody::JointSlider:
                    joint = dJointCreateSlider(GetWorld(), pinfo->jointgroup);
                    break;
                case KinBody::JointUniversal:
                    joint = dJointCreateUniversal(GetWorld(), pinfo->jointgroup);
                    break;
                case KinBody::JointHinge2:
                    joint = dJointCreateHinge2(GetWorld(), pinfo->jointgroup);
                    break;
                case KinBody::JointSpherical:
                    joint = dJointCreateBall(GetWorld(),pinfo->jointgroup);
                    break;
                default:
                    break;
                }

                if( joint == NULL ) {
                    RAVELOG_WARN("ode failed to create joint type %d\n", (*itjoint)->GetType());
                    continue;
                }

                dJointAttach(joint, body0, body1);
                if( (*itjoint)->GetSecondAttached()->IsStatic() ) {
                    axis0 = -axis0;
                    axis1 = -axis1;
                }

                // Joint anchors are specified here, Don't need an offset here since the anchors are global
                KinBody::LinkPtr parent = (*itjoint)->GetHierarchyParentLink();
                switch((*itjoint)->GetType()) {
                case KinBody::JointHinge:
                    dJointSetHingeAnchor(joint,anchor.x,anchor.y,anchor.z);
                    dJointSetHingeAxis(joint,axis0.x,axis0.y,axis0.z);
                    break;
                case KinBody::JointSlider:
                    dJointSetSliderAxis(joint,axis0.x,axis0.y,axis0.z);
                    break;
                case KinBody::JointUniversal:
                    dJointSetUniversalAnchor(joint,anchor.x,anchor.y,anchor.z);
                    dJointSetUniversalAxis1(joint,axis0.x,axis0.y,axis0.z);
                    dJointSetUniversalAxis2(joint,axis1.x,axis1.y,axis1.z);
                    break;
                case KinBody::JointHinge2:
                    dJointSetHinge2Anchor(joint,anchor.x,anchor.y,anchor.z);
                    dJointSetHinge2Axis1(joint,axis0.x,axis0.y,axis0.z);
                    dJointSetHinge2Axis2(joint,axis1.x,axis1.y,axis1.z);
                    break;
                case KinBody::JointSpherical:
                    dJointSetBallAnchor(joint,anchor.x,anchor.y,anchor.z);
                    break;
                default:
                    break;
                }

                // set the joint limits
                vector<OpenRAVE::dReal> vlower, vupper;
                (*itjoint)->GetLimits(vlower,vupper);
                for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
                    //                    if( (*itjoint)->GetMimicJointIndex() < 0 && !bPassive )
                    //                        // setting this makes every joint add like a motor, which is not desired
                    //                        _jointset[dJointGetType(joint)](joint,dParamFMax+dParamGroup*i,(*itjoint)->GetMaxTorque());
                    if( (*itjoint)->GetType() == KinBody::JointSpherical ) {
                        continue;
                    }
                    if( (*itjoint)->IsCircular(i) ) {
                        _jointset[dJointGetType(joint)](joint,dParamLoStop+dParamGroup*i,-dInfinity);
                        _jointset[dJointGetType(joint)](joint,dParamHiStop+dParamGroup*i,dInfinity);
                    }
                    else {
                        _jointset[dJointGetType(joint)](joint,dParamLoStop+dParamGroup*i,vlower[i]);
                        _jointset[dJointGetType(joint)](joint,dParamHiStop+dParamGroup*i,vupper[i]);
                    }
                }

                dJointSetFeedback(joint,&pinfo->vjointfeedback.at(pinfo->vjoints.size()));
                pinfo->vjoints.push_back(joint);
            }
        }

        pinfo->_geometrycallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometry, boost::bind(&ODESpace::_ResetKinBodyCallback,boost::bind(&OpenRAVE::utils::sptr_from<ODESpace>, weak_space()),boost::weak_ptr<KinBody const>(pbody)));
        if( _bUsingPhysics ) {
            pinfo->_staticcallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkStatic|KinBody::Prop_LinkDynamics, boost::bind(&ODESpace::_ResetKinBodyCallback,boost::bind(&OpenRAVE::utils::sptr_from<ODESpace>, weak_space()),boost::weak_ptr<KinBody const>(pbody)));
        }

        pbody->SetUserData(_userdatakey, pinfo);
        _setInitializedBodies.insert(pbody);
        _Synchronize(pinfo, false);
        return pinfo;
    }

    void SetGeometryGroup(const std::string& groupname)
    {
        if( groupname != _geometrygroup ) {
            _geometrygroup = groupname;
            // reset all the existing bodies
            std::vector<KinBodyPtr> vbodies;
            _penv->GetBodies(vbodies);
            FOREACH(itbody, vbodies) {
                KinBodyConstPtr pbody(*itbody);
                KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<KinBodyInfo>(pbody->GetUserData(_userdatakey));
                if( !!pinfo ) {
                    InitKinBody(pbody,pinfo);
                }
            }
        }
    }

    const std::string& GetGeometryGroup()
    {
        return _geometrygroup;
    }

    void RemoveUserData(KinBodyPtr pbody)
    {
        if( !!pbody ) {
            bool bremoved = pbody->RemoveUserData(_userdatakey);
            size_t numerased = _setInitializedBodies.erase(pbody);
            if( (size_t)bremoved != numerased ) {
                RAVELOG_WARN("inconsistency detected with odespace user data\n");
            }
        }
    }

    void Synchronize()
    {
#ifdef ODE_HAVE_ALLOCATE_DATA_THREAD
        dAllocateODEDataForThread(dAllocateMaskAll);
#endif
        boost::mutex::scoped_lock lockode(_ode->_mutex);
        vector<KinBodyPtr> vbodies;
        _penv->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            KinBodyInfoPtr pinfo = GetCreateInfo(*itbody, false).first;
            BOOST_ASSERT( pinfo->GetBody() == *itbody );
            _Synchronize(pinfo,false);
        }
    }

    void Synchronize(KinBodyConstPtr pbody)
    {
        KinBodyInfoPtr pinfo = GetCreateInfo(pbody).first;
        BOOST_ASSERT( pinfo->GetBody() == pbody );
        _Synchronize(pinfo);
    }

    dSpaceID GetBodySpace(KinBodyConstPtr pbody)
    {
        KinBodyInfoPtr pinfo = GetInfo(pbody);
        BOOST_ASSERT(pinfo->GetBody() == pbody );
        return pinfo->space;
    }

    dBodyID GetLinkBody(KinBody::LinkConstPtr plink)
    {
        KinBodyInfoPtr pinfo = GetInfo(plink->GetParent());
        BOOST_ASSERT( pinfo->GetBody() == plink->GetParent() );
        BOOST_ASSERT( plink->GetIndex() >= 0 && plink->GetIndex() < (int)pinfo->vlinks.size());
        return pinfo->vlinks[plink->GetIndex()]->body;
    }

    dGeomID GetLinkGeom(KinBody::LinkConstPtr plink)
    {
        KinBodyInfoPtr pinfo = GetInfo(plink->GetParent());
        BOOST_ASSERT( pinfo->GetBody() == plink->GetParent() );
        BOOST_ASSERT( plink->GetIndex() >= 0 && plink->GetIndex() < (int)pinfo->vlinks.size());
        return pinfo->vlinks[plink->GetIndex()]->geom;
    }

    dJointID GetJoint(KinBody::JointConstPtr pjoint)
    {
        KinBodyInfoPtr pinfo = GetInfo(pjoint->GetParent());
        OPENRAVE_ASSERT_FORMAT(!!pinfo, "info does not exist for joint %s, key %s", pjoint->GetName()%_userdatakey, OpenRAVE::ORE_Assert);
        BOOST_ASSERT( pinfo->GetBody() == pjoint->GetParent() );
        if( pjoint->GetJointIndex() >= 0 ) {
            BOOST_ASSERT( pjoint->GetParent()->GetJointFromDOFIndex(pjoint->GetDOFIndex()) == pjoint );
            BOOST_ASSERT( pjoint->GetJointIndex() >= 0);
            return pinfo->vjoints.at(pjoint->GetJointIndex());
        }
        else {
            // passive joint
            KinBodyPtr pbody = pinfo->GetBody();
            size_t index = pbody->GetJoints().size();
            FOREACHC(itjoint,pbody->GetPassiveJoints()) {
                if( *itjoint == pjoint ) {
                    return pinfo->vjoints.at(index);
                }
                index++;
            }
        }
        return NULL;
    }

    dWorldID GetWorld() const {
        return _ode->world;
    }
    dSpaceID GetSpace() const {
        return _ode->space;
    }
    dJointGroupID GetContactGroup() const {
        return _ode->contactgroup;
    }

    void SetSynchronizationCallback(const SynchronizeCallbackFn& synccallback) {
        _synccallback = synccallback;
    }

    typedef void (*JointSetFn)(dJointID, int param, dReal val);
    JointSetFn _jointset[12];

    KinBodyInfoPtr GetInfo(KinBodyConstPtr pbody) {
        return boost::dynamic_pointer_cast<KinBodyInfo>(pbody->GetUserData(_userdatakey));
    }

    std::pair<KinBodyInfoPtr, bool> GetCreateInfo(KinBodyConstPtr pbody, bool blockode=true) {
        KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<KinBodyInfo>(pbody->GetUserData(_userdatakey));
        bool bcreated = false;
        if( !pinfo ) {
            pinfo = InitKinBody(pbody, KinBodyInfoPtr(), blockode);
            pbody->SetUserData(_userdatakey, pinfo);
            _setInitializedBodies.insert(pbody);
            bcreated = true;
        }
        return std::make_pair(pinfo,bcreated);
    }

private:
    dGeomID _CreateODEGeomFromGeometryInfo(dSpaceID space, boost::shared_ptr<KinBodyInfo::LINK> link, const KinBody::GeometryInfo& info)
    {
        dGeomID odegeom = NULL;
        switch(info._type) {
        case OpenRAVE::GT_None:
            break;
        case OpenRAVE::GT_Box:
            odegeom = dCreateBox(0,info._vGeomData.x*2.0f,info._vGeomData.y*2.0f,info._vGeomData.z*2.0f);
            break;
        case OpenRAVE::GT_Sphere:
            odegeom = dCreateSphere(0,info._vGeomData.x);
            break;
        case OpenRAVE::GT_Cylinder:
            odegeom = dCreateCylinder(0,info._vGeomData.x,info._vGeomData.y);
            break;
        case OpenRAVE::GT_Container:
        case OpenRAVE::GT_TriMesh:
            if( info._meshcollision.indices.size() > 0 ) {
                dTriIndex* pindices = new dTriIndex[info._meshcollision.indices.size()];
                for(size_t i = 0; i < info._meshcollision.indices.size(); ++i) {
                    pindices[i] = info._meshcollision.indices[i];
                }
                dReal* pvertices = new dReal[4*info._meshcollision.vertices.size()];
                for(size_t i = 0; i < info._meshcollision.vertices.size(); ++i) {
                    Vector v = info._meshcollision.vertices[i];
                    pvertices[4*i+0] = v.x; pvertices[4*i+1] = v.y; pvertices[4*i+2] = v.z;
                }
                dTriMeshDataID id = dGeomTriMeshDataCreate();
                dGeomTriMeshDataBuildSimple(id, pvertices, info._meshcollision.vertices.size(), pindices, info._meshcollision.indices.size());
                odegeom = dCreateTriMesh(0, id, NULL, NULL, NULL);
                link->listtrimeshinds.push_back(pindices);
                link->listvertices.push_back(pvertices);
            }
            break;
        default:
            RAVELOG_WARN(str(boost::format("ode doesn't support geom type %d")%info._type));
            break;
        }

        if( !odegeom ) {
            return NULL;
        }

        dGeomID odegeomtrans = dCreateGeomTransform(space);
        dGeomTransformSetCleanup(odegeomtrans, 1);
        dGeomTransformSetGeom(odegeomtrans, odegeom);
        //dGeomSetData(odegeom, (void*)geom.get());

        // set the transformation
        RaveTransform<dReal> t = link->tlinkmassinv * info._t;
        dGeomSetQuaternion(odegeom,&t.rot[0]);
        dGeomSetPosition(odegeom,t.trans.x, t.trans.y, t.trans.z);

        return odegeomtrans;
    }

    /// \param block if true, then will lock _ode->_mutex. Set to false when mutex is known to be already locked.
    void _Synchronize(KinBodyInfoPtr pinfo, bool block=true)
    {
        if( pinfo->nLastStamp != pinfo->GetBody()->GetUpdateStamp() ) {
            boost::shared_ptr<boost::mutex::scoped_lock> lockode;
            if( block ) {
                lockode.reset(new boost::mutex::scoped_lock(_ode->_mutex));
            }
            vector<Transform> vtrans;
            KinBodyPtr pbody = pinfo->GetBody();
            pbody->GetLinkTransformations(vtrans, pinfo->_vdofbranches);
            pinfo->nLastStamp = pbody->GetUpdateStamp();
            BOOST_ASSERT( vtrans.size() == pinfo->vlinks.size() );
            for(size_t i = 0; i < vtrans.size(); ++i) {
                RaveTransform<dReal> t = vtrans[i] * pinfo->vlinks[i]->tlinkmass;
                BOOST_ASSERT( RaveFabs(t.rot.lengthsqr4()-1) < 0.0001f );
                dBodySetQuaternion(pinfo->vlinks[i]->body, &t.rot[0]);
                dBodySetPosition(pinfo->vlinks[i]->body, t.trans.x, t.trans.y, t.trans.z);
            }

            // update stamps also reflect enable links
            FOREACH(it, pinfo->vlinks) {
                (*it)->Enable((*it)->GetLink()->IsEnabled());
            }
            if( !!_synccallback ) {
                _synccallback(pinfo);
            }
        }
    }

    void _ResetKinBodyCallback(boost::weak_ptr<KinBody const> _pbody)
    {
        KinBodyConstPtr pbody(_pbody);
        std::pair<KinBodyInfoPtr, bool> infocreated = GetCreateInfo(pbody);
        if( !infocreated.second ) {
            // only init if the body was not just created
            BOOST_ASSERT(boost::shared_ptr<ODESpace>(infocreated.first->_odespace) == shared_from_this());
            BOOST_ASSERT(infocreated.first->GetBody()==pbody);
            InitKinBody(pbody,infocreated.first);
        }
    }

    EnvironmentBasePtr _penv;
    boost::shared_ptr<ODEResources> _ode;
    std::string _userdatakey;
    std::string _geometrygroup;
    SynchronizeCallbackFn _synccallback;
    std::set<KinBodyConstPtr> _setInitializedBodies; ///< set of bodies that have been initialized and user data is set
    bool _bUsingPhysics;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(ODESpace)
BOOST_TYPEOF_REGISTER_TYPE(ODESpace::KinBodyInfo)
BOOST_TYPEOF_REGISTER_TYPE(ODESpace::KinBodyInfo::LINK)
BOOST_TYPEOF_REGISTER_TYPE(dJointID)
#endif

#endif
