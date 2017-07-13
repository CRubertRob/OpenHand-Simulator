// -*- coding: utf-8 --*
// Copyright (C) 2014 Alejandro Perez & Rosen Diankov <rosen.diankov@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <openraveplugindefs.h>

#ifdef OPENRAVE_HAS_LAPACK
// for jacobians
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/lapack/gesdd.hpp>
#endif

#include "configurationcachetree.h"

namespace configurationcache {

/// \brief holds parameters for threshing the direction. if dot(manipdir, tooldir) > cosanglethresh, then ok
class ManipDirectionThresh
{
public:
    ManipDirectionThresh() : vManipDir(0,0,1), vGlobalDir(0,0,1), fCosAngleThresh(0.9999999) {
    }
    ManipDirectionThresh(const ManipDirectionThresh &r) : vManipDir(r.vManipDir), vGlobalDir(r.vGlobalDir), fCosAngleThresh(r.fCosAngleThresh) {
    }

    inline bool IsInConstraints(const Transform& tmanip) const
    {
        return tmanip.rotate(vManipDir).dot3(vGlobalDir) >= fCosAngleThresh;
    }

    Vector vManipDir; ///< direction on the manipulator
    Vector vGlobalDir; ///< direction in world coordinates
    dReal fCosAngleThresh; ///< the cos angle threshold
};

typedef boost::shared_ptr<ManipDirectionThresh> ManipDirectionThreshPtr;

class ConfigurationJitterer : public SpaceSamplerBase
{
public:
    /// \param parameters The planner parameters used to define the configuration space to jitter. The following fields are required: _getstatefn, _setstatefn, _vConfigUpperLimit, _vConfigLowerLimit, _checkpathvelocityconstraintsfn, _diffstatefn, _nRandomGeneratorSeed, _samplefn. The following are used and optional : _neighstatefn (used for constraining on manifolds)
    ConfigurationJitterer(EnvironmentBasePtr penv, std::istream& is) : SpaceSamplerBase(penv)
    {
        __description = ":Interface Author: Alejandro Perez and Rosen Diankov\n\n\
If the current robot configuration is in collision, then jitters the robot until it is out of collision.\n\
By default will sample the robot's active DOFs. Parameters part of the interface name::\n\
\n\
  [robotname] [samplername]\n\
\n\
";
        RegisterCommand("SetMaxJitter",boost::bind(&ConfigurationJitterer::SetMaxJitterCommand,this,_1,_2),
                        "set a new max jitter");
        RegisterCommand("SetMaxIterations",boost::bind(&ConfigurationJitterer::SetMaxIterationsCommand,this,_1,_2),
                        "set a new max iterations");
        RegisterCommand("SetMaxLinkDistThresh",boost::bind(&ConfigurationJitterer::SetMaxLinkDistThreshCommand,this,_1,_2),
                        "set a new max link dist threshold");
        RegisterCommand("SetPerturbation",boost::bind(&ConfigurationJitterer::SetPerturbationCommand,this,_1,_2),
                        "set a new perturbation");
        RegisterCommand("SetResultOnRobot",boost::bind(&ConfigurationJitterer::SetResultOnRobotCommand,this,_1,_2),
                        "set a new result on a robot");
        RegisterCommand("SetNeighDistThresh",boost::bind(&ConfigurationJitterer::SetNeighDistThreshCommand,this,_1,_2),
                        "sets the minimum distance that nodes can be with respect to each other for the cache");
        RegisterCommand("SetConstraintToolDirection", boost::bind(&ConfigurationJitterer::SetConstraintToolDirectionCommand,this,_1,_2),
                        "constrains an axis of the manipulator around a cone. manipname + 7 values: vManipDir, vGlobalDir, fCosAngleThresh.");
        RegisterCommand("SetManipulatorBias",boost::bind(&ConfigurationJitterer::SetManipulatorBiasCommand,this,_1,_2),
                        "Sets a bias on the sampling so that the manipulator has a tendency to move along vbias direction::\n\n\
  [manipname] bias_dir_x bias_dir_y bias_dir_z [nullsampleprob] [nullbiassampleprob] [deltasampleprob]\n\
 //\n\
    bias_dir is the workspace direction to bias the sampling in.\n\
    nullsampleprob, nullbiassampleprob, and deltasampleprob are in [0,1]\n\
 //");

        bool bUseCache = false;
        std::string robotname, samplername = "MT19937";
        is >> robotname >> samplername >> bUseCache;
        _probot = GetEnv()->GetRobot(robotname);
        OPENRAVE_ASSERT_FORMAT(!!_probot, "could not find robot %s", robotname, ORE_InvalidArguments);

        _vActiveIndices = _probot->GetActiveDOFIndices();
        _nActiveAffineDOFs = _probot->GetAffineDOF();
        _vActiveAffineAxis = _probot->GetAffineRotationAxis();
        _vLinks = _probot->GetLinks();
        _vLinkAABBs.resize(_vLinks.size());
        for(size_t i = 0; i < _vLinks.size(); ++i) {
            _vLinkAABBs[i] = _vLinks[i]->ComputeLocalAABB();
        }

        std::vector<dReal> vweights;
        _probot->GetActiveDOFResolutions(vweights);

        // if weights are zero, used a default value
        FOREACH(itweight, vweights) {
            if( *itweight > 0 ) {
                *itweight = 1 / *itweight;
            }
            else {
                *itweight = 100;
            }
        }

        if( bUseCache ) {
            _cache.reset(new CacheTree(_probot->GetActiveDOF()));
            _cache->Init(vweights, 1);
        }

        _bSetResultOnRobot = true;
        _busebiasing = false;

        // for selecting sampling modes
        if( samplername.size() == 0 ) {
            samplername = "mt19937";
        }
        _ssampler = RaveCreateSpaceSampler(penv,samplername);
        OPENRAVE_ASSERT_FORMAT(!!_ssampler, "sampler %s not found", samplername, ORE_InvalidArguments);
        _ssampler->SetSpaceDOF(1);

        size_t dof = _probot->GetActiveDOF();

        // use for sampling, perturbations
        _curdof.resize(dof,0);
        _newdof2.resize(dof);
        _deltadof.resize(dof);
        _deltadof2.resize(dof);
        _nRandomGeneratorSeed = 0;

        _report.reset(new CollisionReport());
        _maxiterations=5000;
        _maxjitter=0.02;
        _perturbation=1e-5;
        _linkdistthresh=0.02;
        _linkdistthresh2 = _linkdistthresh*_linkdistthresh;
        _neighdistthresh = 1;

        _UpdateLimits();
        _limitscallback = _probot->RegisterChangeCallback(RobotBase::Prop_JointLimits, boost::bind(&ConfigurationJitterer::_UpdateLimits,this));
        _UpdateGrabbed();
        _grabbedcallback = _probot->RegisterChangeCallback(RobotBase::Prop_RobotGrabbed, boost::bind(&ConfigurationJitterer::_UpdateGrabbed,this));

        if( !!_cache ) {
            _SetCacheMaxDistance();
        }
    }

    virtual ~ConfigurationJitterer(){
    }

    virtual void SetSeed(uint32_t seed) {
        _nRandomGeneratorSeed = seed;
        _ssampler->SetSeed(seed);
    }

    virtual int GetDOF() const {
        return (int)_lower.size();
    }
    virtual int GetNumberOfValues() const {
        return (int)_lower.size();
    }

    virtual bool Supports(SampleDataType type) const {
        return type == SDT_Real;
    }

    virtual void GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const {
        vLowerLimit = _lower;
        vUpperLimit = _upper;
    }

    virtual void GetLimits(std::vector<uint32_t>& vLowerLimit, std::vector<uint32_t>& vUpperLimit) const {
        BOOST_ASSERT(0);
    }

    virtual bool SetMaxJitterCommand(std::ostream& sout, std::istream& sinput)
    {
        dReal maxjitter=0;
        sinput >> maxjitter;
        if( !sinput || maxjitter < 0 ) {
            return false;
        }
        _maxjitter = maxjitter;
        if( !!_cache ) {
            _SetCacheMaxDistance();
        }
        return true;
    }

    bool SetMaxIterationsCommand(std::ostream& sout, std::istream& sinput)
    {
        int maxiterations=0;
        sinput >> maxiterations;
        if( !sinput || maxiterations < 0 ) {
            return false;
        }
        _maxiterations = maxiterations;
        return true;
    }

    bool SetMaxLinkDistThreshCommand(std::ostream& sout, std::istream& sinput)
    {
        dReal linkdistthresh=0;
        sinput >> linkdistthresh;
        if( linkdistthresh < 0 || !sinput ) {
            return false;
        }
        _linkdistthresh = linkdistthresh;
        _linkdistthresh2 = _linkdistthresh*_linkdistthresh;
        return true;
    }

    bool SetPerturbationCommand(std::ostream& sout, std::istream& sinput)
    {
        int perturbation=0;
        sinput >> perturbation;
        if( perturbation < 0 ) {
            return false;
        }
        _perturbation = perturbation;
        return true;
    }

    bool SetResultOnRobotCommand(std::ostream& sout, std::istream& sinput)
    {
        int bSetResultOnRobot = 0;
        sinput >> bSetResultOnRobot;
        if( bSetResultOnRobot < 0 ) {
            return false;
        }
        _bSetResultOnRobot = bSetResultOnRobot;
        return true;
    }

    bool SetNeighDistThreshCommand(std::ostream& sout, std::istream& sinput)
    {
        dReal neighdistthresh = 0;
        sinput >> neighdistthresh;
        if( neighdistthresh <= 0 ) {
            return false;
        }
        _neighdistthresh = neighdistthresh;
        return true;
    }

    bool SetConstraintToolDirectionCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string manipname;
        ManipDirectionThreshPtr thresh(new ManipDirectionThresh());
        sinput >> manipname;
        if( manipname.size() == 0 ) {
            // reset the tool direction
            if( !!_pConstraintToolDirection ) {
                if( !!_cache ) {
                    _cache->Reset(); // need this here in order to invalidate cache.
                }
            }
            _pConstraintToolDirection.reset();
            return true;
        }
        sinput >> thresh->vManipDir.x >> thresh->vManipDir.y >> thresh->vManipDir.z >> thresh->vGlobalDir.x >> thresh->vGlobalDir.y >> thresh->vGlobalDir.z >> thresh->fCosAngleThresh;
        if( !sinput ) {
            return false;
        }
        RobotBase::ManipulatorConstPtr pmanip = _probot->GetManipulator(manipname);
        if( !pmanip ) {
            return false;
        }
        _pmanip = pmanip;
        _pConstraintToolDirection = thresh;
        if( !!_cache ) {
            _cache->Reset(); // need this here in order to invalidate cache.
        }
        return true;
    }

    virtual int SampleSequence(std::vector<dReal>& samples, size_t num=1,IntervalType interval=IT_Closed)
    {
        samples.resize(0);
        for(size_t i = 0; i < num; ++i) {
            int ret = Sample(_vonesample, interval);
            if( ret == 1 ) {
                samples.insert(samples.end(), _vonesample.begin(), _vonesample.end());
            }
            else {
                return ret;
            }
        }
        return (int)num;
    }

    virtual dReal SampleSequenceOneReal(IntervalType interval) {
        BOOST_ASSERT(0);
        return 0;
    }
    virtual uint32_t SampleSequenceOneUInt32()
    {
        BOOST_ASSERT(0);
        return 0;
    }

    virtual int SampleComplete(std::vector<dReal>& samples, size_t num, IntervalType interval=IT_Closed) {
        // have to reset the seed
        _ssampler->SetSeed(_nRandomGeneratorSeed);
        return SampleSequence(samples, num, interval);
    }

    virtual int SampleComplete(std::vector<uint32_t>& samples, size_t num) {
        BOOST_ASSERT(0);
        return 0;
    }

    virtual bool SetManipulatorBiasCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string manipname;
        Vector vbiasdirection(0,0,0.1);
        dReal nullsampleprob = 0.60, nullbiassampleprob = 0.50, deltasampleprob = 0.50;
        sinput >> manipname >> vbiasdirection.x >> vbiasdirection.y >> vbiasdirection.z >> nullsampleprob >> nullbiassampleprob >> deltasampleprob;
        RobotBase::ManipulatorConstPtr pmanip = _probot->GetManipulator(manipname);
        if( !pmanip ) {
            return false;
        }
        if( vbiasdirection.lengthsqr3() <= g_fEpsilon ) {
            return false;
        }
        //vbiasdirection.normalize3();
        SetManipulatorBias(pmanip, vbiasdirection, nullsampleprob, nullbiassampleprob, deltasampleprob);
        return true;
    }

    void SetManipulatorBias(RobotBase::ManipulatorConstPtr pmanip, const Vector& vbiasdirection, dReal nullsampleprob, dReal nullbiassampleprob, dReal deltasampleprob)
    {
#ifdef OPENRAVE_HAS_LAPACK
        _pmanip = pmanip;
        _vbiasdirection = vbiasdirection;
        _vbiasdofdirection.resize(0);
        _vbiasnullspace.resize(0);
        _nullsampleprob = nullsampleprob;
        _nullbiassampleprob = nullbiassampleprob;
        _deltasampleprob = deltasampleprob;
        _busebiasing = true;
        RAVELOG_VERBOSE_FORMAT("set bias nullsampleprob %f nullbiassampleprob %f deltasampleprob %f", _nullsampleprob%_nullbiassampleprob%_deltasampleprob);
#else
        throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot set manipulator bias since lapack is not supported"), ORE_CommandNotSupported);
#endif
    }

    /// \brief jitters the current configuration and sets a new configuration on the environment
    ///
    int Sample(std::vector<dReal>& vnewdof, IntervalType interval=IT_Closed)
    {
        RobotBase::RobotStateSaver robotsaver(_probot, KinBody::Save_LinkTransformation|KinBody::Save_ActiveDOF);
        _InitRobotState();
        const dReal linkdistthresh = _linkdistthresh;
        const dReal linkdistthresh2 = _linkdistthresh2;

        vector<AABB> newLinkAABBs;
        bool bCollision = false;
        bool bConstraintFailed = false;
        bool bConstraint = !!_neighstatefn;

        // have to test with perturbations since very small changes in angles can produce collision inconsistencies
        std::vector<dReal> perturbations;
        if( _perturbation > 0 ) {
            perturbations.resize(3,0);
            perturbations[0] = _perturbation;
            perturbations[1] = -_perturbation;
        }
        else {
            perturbations.resize(1,0);
        }
        vnewdof.resize(GetDOF());
        FOREACH(itperturbation,perturbations) {
            if( bConstraint ) {
                FOREACH(it,_deltadof) {
                    *it = *itperturbation;
                }
                vnewdof = _curdof;
                if( !_neighstatefn(vnewdof,_deltadof,0) ) {
                    _probot->SetActiveDOFValues(_curdof);
//                    if( setret != 0 ) {
//                        // state failed to set, this could mean the initial state is just really bad, so resume jittering
//                        bCollision = true;
//                        break;
//                    }
                    return -1;
                }
            }
            else {
                for(size_t i = 0; i < vnewdof.size(); ++i) {
                    vnewdof[i] = _curdof[i]+*itperturbation;
                    if( vnewdof[i] > _upper.at(i) ) {
                        vnewdof[i] = _upper.at(i);
                    }
                    else if( vnewdof[i] < _lower.at(i) ) {
                        vnewdof[i] = _lower.at(i);
                    }
                }
            }

            // don't need to set state since CheckPathAllConstraints does it
            _probot->SetActiveDOFValues(vnewdof);

            if( !!_pConstraintToolDirection && !!_pmanip ) {
                if( !_pConstraintToolDirection->IsInConstraints(_pmanip->GetTransform()) ) {
                    bConstraintFailed = true;
                    break;

                }
            }
            if( GetEnv()->CheckCollision(_probot, _report) || _probot->CheckSelfCollision(_report) ) {
                bCollision = true;
                break;
            }
        }

        if( (!bCollision && !bConstraintFailed) || _maxjitter <= 0 ) {
            return -1;
        }

        if( !!_cache ) {
            _cache->InsertNode(_curdof, CollisionReportPtr(), _neighdistthresh);
            _cachehit = 0;
        }

        BOOST_ASSERT(!_busebiasing || _vbiasdofdirection.size() > 0);
        const boost::array<dReal, 3> rayincs = {{0.5, 0.9, 0.2}};

        bool busebiasing = _busebiasing;
        const int nMaxIterRadiusThresh=_maxiterations/2;
        const dReal imaxiterations = 2.0/dReal(_maxiterations);
        const dReal fJitterLowerThresh=0.2, fJitterHigherThresh=0.8;
        dReal fBias = _vbiasdirection.lengthsqr3();
        if( fBias > g_fEpsilon ) {
            fBias = RaveSqrt(fBias);
        }

        uint64_t starttime = utils::GetNanoPerformanceTime();
        for(int iter = 0; iter < _maxiterations; ++iter) {
            if( (iter%10) == 0 ) { // not sure what a good rate is...
                _CallStatusFunctions(iter);
            }
            if( busebiasing && iter < (int)rayincs.size() ) {
                // start by checking samples directly above the current configuration
                for (size_t j = 0; j < vnewdof.size(); ++j) {
                    vnewdof[j] = _curdof[j] + (rayincs[iter] * _vbiasdofdirection.at(j));
                }
            }
            else {
                // ramp of the jitter as iterations increase
                dReal jitter = _maxjitter;
                if( iter < nMaxIterRadiusThresh ) {
                    jitter = _maxjitter*dReal(iter+1)*imaxiterations;
                }

                bool samplebiasdir = false;
                bool samplenull = false;
                bool sampledelta = false;
                if (busebiasing && _ssampler->SampleSequenceOneReal() < _nullsampleprob)
                {
                    samplenull = true;
                }
                if (busebiasing && _ssampler->SampleSequenceOneReal() < _nullbiassampleprob) {
                    samplebiasdir = true;
                }
                if( (!samplenull && !samplebiasdir) || _ssampler->SampleSequenceOneReal() < _deltasampleprob ) {
                    sampledelta = true;
                }

                bool deltasuccess = false;
                if( sampledelta ) {
                    // check which third the sampled dof is in
                    for(size_t j = 0; j < vnewdof.size(); ++j) {
                        dReal f = 2*_ssampler->SampleSequenceOneReal(interval)-1; // f in [-1,1]
                        if( RaveFabs(f) < fJitterLowerThresh ) {
                            _deltadof[j] = 0;
                        }
                        else if( f < -fJitterHigherThresh ) {
                            _deltadof[j] = -jitter;
                        }
                        else if( f > fJitterHigherThresh ) {
                            _deltadof[j] = jitter;
                        }
                        else {
                            _deltadof[j] = jitter*f;
                        }
                    }
                    deltasuccess = true;
                }

                if (!samplebiasdir && !samplenull && !deltasuccess) {
                    continue;
                }
                // (lambda * biasdir) + (Nx) + delta + _curdofs
                dReal fNullspaceMultiplier = linkdistthresh*2;
                if( fNullspaceMultiplier <= 0 ) {
                    fNullspaceMultiplier = fBias;
                }
                for (size_t k = 0; k < vnewdof.size(); ++k) {
                    vnewdof[k] = _curdof[k];
                    if (samplebiasdir) {
                        vnewdof[k] += _ssampler->SampleSequenceOneReal() * _vbiasdofdirection[k];
                    }
                    if (samplenull) {
                        for (size_t j = 0; j < _vbiasnullspace.size(); ++j) {
                            dReal nullx = (_ssampler->SampleSequenceOneReal()*2-1)*fNullspaceMultiplier;
                            vnewdof[k] += nullx * _vbiasnullspace[j][k];
                        }
                    }
                    if (sampledelta) {
                        vnewdof[k] += _deltadof[k];
                    }
                }
            }

            // get new state
            for(size_t j = 0; j < _deltadof.size(); ++j) {
                if( vnewdof[j] > _upper.at(j) ) {
                    vnewdof[j] = _upper.at(j);
                }
                else if( vnewdof[j] < _lower.at(j) ) {
                    vnewdof[j] = _lower.at(j);
                }
            }

            if( !!_cache ) {
                if( !!_cache->FindNearestNode(vnewdof, _neighdistthresh).first ) {
                    _cachehit++;
                    continue;
                }
            }

            //int ret = cache.InsertNode(vnewdof, CollisionReportPtr(), _neighdistthresh);
            //BOOST_ASSERT(ret==1);

            _probot->SetActiveDOFValues(vnewdof);
#ifdef _DEBUG
            dReal fmaxtransdist = 0;
#endif
            bool bSuccess = true;
            if( linkdistthresh > 0 ) {
                for (size_t ilink = 0; ilink < _vLinkAABBs.size(); ++ilink) {
                    // check for an elipse
                    // L^2 (b*v)^2 + |v|^2|b|^4 - (b*v)^2 |b|^2 <= |b|^4 * L^2
                    Transform tnewlink = _vLinks[ilink]->GetTransform();
                    TransformMatrix projdelta = _vOriginalInvTransforms[ilink] * tnewlink;
                    projdelta.m[0] -= 1;
                    projdelta.m[5] -= 1;
                    projdelta.m[10] -= 1;
                    Vector projextents = _vLinkAABBs[ilink].extents;
                    Vector projboxright(projdelta.m[0]*projextents.x, projdelta.m[4]*projextents.x, projdelta.m[8]*projextents.x);
                    Vector projboxup(projdelta.m[1]*projextents.y, projdelta.m[5]*projextents.y, projdelta.m[9]*projextents.y);
                    Vector projboxdir(projdelta.m[2]*projextents.z, projdelta.m[6]*projextents.z, projdelta.m[10]*projextents.z);
                    Vector projboxpos = projdelta * _vLinkAABBs[ilink].pos;

                    Vector b;
                    if( busebiasing ) {
                        b = _vOriginalInvTransforms[ilink].rotate(_vbiasdirection); // inside link coordinate system
                    }
                    else {
                        // doesn't matter which vector we pick since it is just a sphere.
                        b = Vector(0,0,linkdistthresh);
                    }

                    dReal blength2 = b.lengthsqr3();
                    dReal blength4 = blength2*blength2;
                    dReal rhs = blength4 * linkdistthresh2;
                    //dReal rhs = (b.lengthsqr3()) * linkdistthresh;
                    dReal ellipdist = 0;
                    // now figure out what is the max distance
                    for(int ix = 0; ix < 2; ++ix) {
                        Vector projvx = ix > 0 ? projboxpos + projboxright : projboxpos - projboxright;
                        for(int iy = 0; iy < 2; ++iy) {
                            Vector projvy = iy > 0 ? projvx + projboxup : projvx - projboxup;
                            for(int iz = 0; iz < 2; ++iz) {
                                Vector projvz = iz > 0 ? projvy + projboxdir : projvy - projboxdir;
                                Vector v = projvz; // inside link coordinate system
                                dReal bv = (v.dot3(b));
                                dReal bv2 = bv*bv;
                                dReal flen2 = (linkdistthresh2 - blength2) * bv2 + v.lengthsqr3()*blength4;

                                if( ellipdist < flen2 ) {
                                    ellipdist = flen2;
#ifdef _DEBUG
                                    fmaxtransdist = flen2;
#endif
                                    if (ellipdist > rhs) {
                                        bSuccess = false;
                                        break;
                                    }
                                }
                            }

                            if (ellipdist > rhs) {
                                bSuccess = false;
                                break;
                            }
                        }
                        if (ellipdist > rhs) {
                            bSuccess = false;
                            break;
                        }
                    }
                }

                if (!bSuccess) {
                    continue;
                }
            }

            // check perturbation
            bCollision = false;
            bConstraintFailed = false;
            FOREACH(itperturbation,perturbations) {
                for(size_t j = 0; j < _deltadof.size(); ++j) {
                    _deltadof2[j] = *itperturbation;
                }
                if( bConstraint ) {
                    _newdof2 = vnewdof;
                    _probot->SetActiveDOFValues(_newdof2);
                    if( !_neighstatefn(_newdof2,_deltadof2,0) ) {
                        if( *itperturbation != 0 ) {
                            RAVELOG_DEBUG(str(boost::format("constraint function failed, pert=%e\n")%*itperturbation));
                        }
                        bConstraintFailed = true;
                        break;
                    }
                }
                else {
                    for(size_t j = 0; j < _deltadof.size(); ++j) {
                        _newdof2[j] = vnewdof[j] + _deltadof2[j];
                        if( _newdof2[j] > _upper.at(j) ) {
                            _newdof2[j] = _upper.at(j);
                        }
                        else if( _newdof2[j] < _lower.at(j) ) {
                            _newdof2[j] = _lower.at(j);
                        }
                    }
                }

                _probot->SetActiveDOFValues(_newdof2);
                if( !!_pConstraintToolDirection ) {
                    if( !_pConstraintToolDirection->IsInConstraints(_pmanip->GetTransform()) ) {
                        bConstraintFailed = true;
                        break;
                    }
                }

                if( GetEnv()->CheckCollision(_probot, _report) || _probot->CheckSelfCollision(_report)) {
                    bCollision = true;
                    if( IS_DEBUGLEVEL(Level_Verbose) ) {
                        stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                        ss << "constraints failed, ";
                        for(size_t i = 0; i < _newdof2.size(); ++i ) {
                            if( i > 0 ) {
                                ss << "," << _newdof2[i];
                            }
                            else {
                                ss << "colvalues=[" << _newdof2[i];
                            }
                        }
                        ss << "], report=" << _report->__str__();
                        RAVELOG_VERBOSE(ss.str());
                    }
                    break;
                }
            }

            if( !bCollision && !bConstraintFailed ) {
                // the last perturbation is 0, so state is already set to the correct jittered value
                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                    _probot->GetActiveDOFValues(vnewdof);
                    stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                    ss << "jitter iter=" << iter << " ";
#ifdef _DEBUG
                    ss << "maxtrans=" << fmaxtransdist << " ";
#endif
                    for(size_t i = 0; i < vnewdof.size(); ++i ) {
                        if( i > 0 ) {
                            ss << "," << vnewdof[i];
                        }
                        else {
                            ss << "jitteredvalues=[" << vnewdof[i];
                        }
                    }
                    ss << "]";
                    RAVELOG_VERBOSE(ss.str());
                }

                if( _bSetResultOnRobot ) {
                    // have to release the saver so it does not restore the old configuration
                    robotsaver.Release();
                }

                RAVELOG_DEBUG_FORMAT("succeed iterations=%d, computation=%fs\n",iter%(1e-9*(utils::GetNanoPerformanceTime() - starttime)));
                //RAVELOG_VERBOSE_FORMAT("succeed iterations=%d, cachehits=%d, cache size=%d, originaldist=%f, computation=%fs\n",iter%_cachehit%cache.GetNumNodes()%cache.ComputeDistance(_curdof, vnewdof)%(1e-9*(utils::GetNanoPerformanceTime() - starttime)));
                return 1;
            }
        }

        RAVELOG_INFO_FORMAT("failed iterations=%d, computation=%fs\n",_maxiterations%(1e-9*(utils::GetNanoPerformanceTime() - starttime)));
        //RAVELOG_WARN_FORMAT("failed iterations=%d, cachehits=%d, cache size=%d, jitter time=%fs", _maxiterations%_cachehit%cache.GetNumNodes()%(1e-9*(utils::GetNanoPerformanceTime() - starttime)));
        return 0;
    }

protected:

    /// \brief extracts all used bodies from the configurationspecification and computes AABBs, transforms, and limits for links
    void _InitRobotState()
    {
        _probot->SetActiveDOFs(_vActiveIndices, _nActiveAffineDOFs, _vActiveAffineAxis);
        _probot->GetActiveDOFValues(_curdof);

        _vOriginalTransforms.resize(_vLinks.size());
        _vOriginalInvTransforms.resize(_vLinks.size());
        for(size_t i = 0; i < _vLinks.size(); ++i) {
            _vOriginalTransforms[i] = _vLinks[i]->GetTransform();
            _vOriginalInvTransforms[i] = _vOriginalTransforms[i].inverse();
        }
#ifdef OPENRAVE_HAS_LAPACK
        if( _busebiasing ) {
            using namespace boost::numeric::ublas;
            _pmanip->CalculateJacobian(_mjacobian);
            boost::numeric::ublas::matrix<double, boost::numeric::ublas::column_major> J(3,_pmanip->GetArmIndices().size());
            boost::numeric::ublas::vector<double> P(3);
            for(size_t i = 0; i < 3; ++i) {
                P(i) = _vbiasdirection[i];
                for(size_t j = 0; j < _pmanip->GetArmIndices().size(); ++j) {
                    J(i,j) = _mjacobian[i][j]; // *_viweights.at(j), will have to scale output also?, needs testing
                }
            }

            dReal zerothresh = 1e-7;
            boost::numeric::ublas::vector<double> S(P.size());
            size_t numdof = _pmanip->GetArmIndices().size();
            boost::numeric::ublas::matrix<double, column_major> U(P.size(),P.size()), V(numdof,numdof);
            // J * dofvelocities = P
            // compute single value decomposition: Jacobian = U*diag(S)*transpose(V)
            int ret = boost::numeric::bindings::lapack::gesdd('O','A',J,S,U,V);
            if( ret != 0 ) {
                RAVELOG_WARN("failed to compute SVD for jacobian, disabling bias\n");
                return;
            }
            // diag(S) * transpose(V) * dofvelocities = transpose(U) * P = P2
            // transpose(V) * dofvelocities = diag(1/S) * P2 = P3
            boost::numeric::ublas::vector<double> P2 = prod(trans(U),P);
            boost::numeric::ublas::vector<double> P3(numdof);
            for(size_t i = 0; i < numdof; ++i) {
                if( i < S.size() ) {
                    if( RaveFabs(S(i)) < zerothresh ) {
                        P3(i) = 0;
                    }
                    else {
                        P3(i) = P2(i)/S(i);
                    }
                }
                else {
                    P3(i) = 0;
                }
            }
            // dofvelocities = P3
            P3 = prod(trans(V),P3);
            _vbiasdofdirection.resize(numdof);
            for(size_t i = 0; i < numdof; ++i) {
                _vbiasdofdirection[i] = P3(i);
            }

            size_t istart = 0;
            for(istart = 0; istart < S.size(); ++istart ) {
                if( RaveFabs(S(istart)) < zerothresh ) {
                    break;
                }
            }

            if( istart >= numdof ) {
                // no _vbiasnullspace, so extract _vbiasdofdirection
                _vbiasnullspace.resize(0);
            }
            else {
                _vbiasnullspace.resize(numdof - istart);
                for(size_t i = istart; i < numdof; ++i) {
                    _vbiasnullspace[i-istart].resize(numdof);
                    for(size_t j = 0; j < numdof; ++j) {
                        _vbiasnullspace[i-istart][j] = V(i,j);
                    }
                }
            }
        }
#endif

        // update all the links (since geometry could have changed)
        _vLinkAABBs.resize(_vLinks.size());
        for(size_t i = 0; i < _vLinks.size(); ++i) {
            _vLinkAABBs[i] = _vLinks[i]->ComputeLocalAABB();
        }


        if( !!_cache ) {
            _cache->Reset(); // need this here in order to invalidate cache.
        }
    }

    void _UpdateGrabbed()
    {
        vector<KinBodyPtr> vgrabbedbodies;
        _probot->GetGrabbed(vgrabbedbodies);
        _vLinks.resize(_probot->GetLinks().size());
        FOREACHC(itgrabbed, vgrabbedbodies) {
            FOREACHC(itlink2, (*itgrabbed)->GetLinks()) {
                _vLinks.push_back(*itlink2);
            }
        }

        // update all the grabbed links
        _vLinkAABBs.resize(_vLinks.size());
        for(size_t i = _probot->GetLinks().size(); i < _vLinks.size(); ++i) {
            _vLinkAABBs[i] = _vLinks[i]->ComputeLocalAABB();
        }

        //_SetCacheMaxDistance();
    }

    void _UpdateLimits()
    {
        _probot->SetActiveDOFs(_vActiveIndices, _nActiveAffineDOFs, _vActiveAffineAxis);
        _probot->GetActiveDOFLimits(_lower, _upper);
        _range.resize(_lower.size());
        // even though jitter is limited, if bias is enabled
        for(size_t i = 0; i < _range.size(); ++i) {
            _range[i] = _upper[i] - _lower[i];
        }

        //_SetCacheMaxDistance();
    }

    /// sets the cache's max configuration distance
    void _SetCacheMaxDistance()
    {
        dReal maxdistance=0;
        for(size_t i = 0; i < _cache->GetWeights().size(); ++i) {
            dReal f = _range[i] * _cache->GetWeights()[i];
            maxdistance += f*f;
        }
        maxdistance = RaveSqrt(maxdistance);
        if( maxdistance > _cache->GetMaxDistance()+g_fEpsilonLinear ) {
            _cache->SetMaxDistance(maxdistance);
        }
    }

    RobotBasePtr _probot;
    std::vector<dReal> _lower, _upper, _range, _rangescaled;
    std::vector<int> _vActiveIndices;
    int _nActiveAffineDOFs;
    Vector _vActiveAffineAxis;
    std::vector<KinBody::LinkPtr> _vLinks; ///< links tracking for linkdistthresh
    std::vector<AABB> _vLinkAABBs; ///< indexed according to _vLinks
    std::vector<Transform> _vOriginalTransforms, _vOriginalInvTransforms; ///< indexed according to _vLinks
    CollisionReportPtr _report;

    boost::function<bool (std::vector<dReal>&,const std::vector<dReal>&, int)> _neighstatefn; ///< if initialized, then use this function to get nearest neighbor
    ///< Advantage of using neightstatefn is that user constraints can be met like maintaining a certain orientation of the gripper.

    UserDataPtr _limitscallback, _grabbedcallback; ///< limits,grabbed change handles

    /// \return Return 0 if jitter failed and constraints are not satisfied. -1 if constraints are originally satisfied. 1 if jitter succeeded, configuration is different, and constraints are satisfied.

    uint32_t _nRandomGeneratorSeed;
    int _maxiterations; ///< number of different configurations to test
    dReal _maxjitter; ///< The max deviation of a dof value to jitter. value +- maxjitter
    dReal _perturbation; ///< Test with perturbations since very small changes in angles can produce collision inconsistencies
    dReal _linkdistthresh, _linkdistthresh2; ///< the maximum distance to allow a link to move. If 0, then will disable checking

    std::vector<dReal> _curdof, _newdof2, _deltadof, _deltadof2, _vonesample;

    CacheTreePtr _cache; ///< caches the visisted configurations
    int _cachehit;
    dReal _neighdistthresh; ///< the minimum distance that nodes can be with respect to each other for the cache

    // for biasing
    SpaceSamplerBasePtr _ssampler;
    dReal _nullsampleprob;
    dReal _nullbiassampleprob;
    dReal _deltasampleprob;
    RobotBase::ManipulatorConstPtr _pmanip;
    boost::multi_array<OpenRAVE::dReal,2> _mjacobian;
    Vector _vbiasdirection; // direction to bias in workspace. magnitude is the max bias distance
    std::vector<dReal> _vbiasdofdirection; // direction to bias in configuration space (from jacobian)
    std::vector< std::vector<dReal> > _vbiasnullspace; // configuration nullspace that does not constraint rotation. vectors are unit

    ManipDirectionThreshPtr _pConstraintToolDirection;

    bool _bSetResultOnRobot; ///< if true, will set the final result on the robot DOF values
    bool _busebiasing; ///< if true will bias the end effector along a certain direction using the jacobian and nullspace.
};

SpaceSamplerBasePtr CreateConfigurationJitterer(EnvironmentBasePtr penv, std::istream& sinput)
{
    return SpaceSamplerBasePtr(new ConfigurationJitterer(penv, sinput));
}

}
