// -*- coding: utf-8 -*-
// Copyright (C) 2012-2015 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "openraveplugindefs.h"
#include <fstream>

#include <openrave/planningutils.h>

#include "manipconstraints.h"
#include "ParabolicPathSmooth/DynamicPath.h"

namespace rplanners {

namespace ParabolicRamp = ParabolicRampInternal;

class ParabolicSmoother : public PlannerBase, public ParabolicRamp::FeasibilityCheckerBase, public ParabolicRamp::RandomNumberGeneratorBase
{
    class MyRampFeasibilityChecker : public ParabolicRamp::RampFeasibilityChecker
    {
public:
        MyRampFeasibilityChecker(ParabolicRamp::FeasibilityCheckerBase* feas) : ParabolicRamp::RampFeasibilityChecker(feas) {
        }

        /// \brief checks a ramp for collisions.
        ParabolicRamp::CheckReturn Check2(const ParabolicRamp::ParabolicRampND& rampnd, int options, std::vector<ParabolicRamp::ParabolicRampND>& outramps)
        {
            // only set constraintchecked if all necessary constraints are checked
            if( (options & constraintsmask) == constraintsmask ) {
                rampnd.constraintchecked = 1;
            }
            OPENRAVE_ASSERT_OP(tol.size(), ==, rampnd.ramps.size());
            for(size_t i = 0; i < tol.size(); ++i) {
                OPENRAVE_ASSERT_OP(tol[i], >, 0);
            }

            _ExtractSwitchTimes(rampnd, vswitchtimes, true);
            ParabolicRamp::CheckReturn ret0 = feas->ConfigFeasible2(rampnd.x0, rampnd.dx0, options);
            if( ret0.retcode != 0 ) {
                return ret0;
            }
            ParabolicRamp::CheckReturn ret1 = feas->ConfigFeasible2(rampnd.x1, rampnd.dx1, options);
            if( ret1.retcode != 0 ) {
                return ret1;
            }

            // check if configurations are feasible for all the switch times.
            _vsearchsegments.resize(vswitchtimes.size(), 0);
            for(size_t i = 0; i < _vsearchsegments.size(); ++i) {
                _vsearchsegments[i] = i;
            }
            int midindex = _vsearchsegments.size()/2;
            std::swap(_vsearchsegments[0], _vsearchsegments[midindex]); // put the mid point as the first point to be considered
            for(size_t i = 0; i < vswitchtimes.size(); ++i) {
                dReal switchtime = vswitchtimes[_vsearchsegments[i]];
                rampnd.Evaluate(switchtime,q0);
                if( feas->NeedDerivativeForFeasibility() ) {
                    rampnd.Derivative(switchtime,dq0);
                }
                ParabolicRamp::CheckReturn retconf = feas->ConfigFeasible2(q0, dq0, options);
                if( retconf.retcode != 0 ) {
                    return retconf;
                }
            }

            outramps.resize(0);

            // check each of the ramps sequentially
            q0 = rampnd.x0;
            dq0 = rampnd.dx0;
            q1.resize(q0.size());
            dq1.resize(dq0.size());
            for(size_t iswitch = 1; iswitch < vswitchtimes.size(); ++iswitch) {
                rampnd.Evaluate(vswitchtimes.at(iswitch),q1);
                dReal elapsedtime = vswitchtimes.at(iswitch)-vswitchtimes.at(iswitch-1);

                // unfortunately due to constraints, rampnd.Derivative(vswitchtimes.at(iswitch),dq1); might not be consistent with q0, q1, dq0, and elapsedtime, so recompute it here
                if( feas->NeedDerivativeForFeasibility() ) {
                    rampnd.Derivative(vswitchtimes.at(iswitch),dq1);
                    dReal expectedelapsedtime = 0;
                    dReal totalweight = 0;
                    for(size_t idof = 0; idof < dq0.size(); ++idof) {
                        dReal avgvel = 0.5*(dq0[idof] + dq1[idof]);
                        if( RaveFabs(avgvel) > g_fEpsilon ) {
                            // need to weigh appropriately or else small differences in q1-q0 can really affect the result.
                            dReal fweight = RaveFabs(q1[idof] - q0[idof]);
                            expectedelapsedtime += fweight*(q1[idof] - q0[idof])/avgvel;
                            totalweight += fweight;
                        }
                    }
                    if( totalweight > g_fEpsilon ) {
                        // find a better elapsed time
                        dReal newelapsedtime = expectedelapsedtime/totalweight;
                        if( RaveFabs(newelapsedtime) > ParabolicRamp::EpsilonT ) {
                            RAVELOG_VERBOSE_FORMAT("changing ramp elapsed time %.15e -> %.15e", elapsedtime%newelapsedtime);
                            elapsedtime = newelapsedtime;
                            if( elapsedtime > g_fEpsilon ) {
                                dReal ielapsedtime = 1/elapsedtime;
                                for(size_t idof = 0; idof < dq0.size(); ++idof) {
                                    dq1[idof] = 2*ielapsedtime*(q1[idof] - q0[idof]) - dq0[idof];
                                }
                            }
                            else {
                                // elapsed time is non-existent, so have the same velocity?
                                dq1 = dq0;
                            }
                        }
                    }
                }

                // have to recompute a new
                ParabolicRamp::CheckReturn retseg = feas->SegmentFeasible2(q0,q1, dq0, dq1, elapsedtime, options, segmentoutramps);
                if( retseg.retcode != 0 ) {
                    return retseg;
                }

                if( segmentoutramps.size() > 0 ) {
                    if( IS_DEBUGLEVEL(Level_Verbose) ) {
                        for(size_t idof = 0; idof < q0.size(); ++idof) {
                            if( RaveFabs(q1[idof]-segmentoutramps.back().x1[idof]) > ParabolicRamp::EpsilonX ) {
                                // RAVELOG_VERBOSE_FORMAT("ramp end point does not finish at desired position values %f, so rejecting", (q0[idof]-rampnd.x1[idof]));
                                RAVELOG_VERBOSE_FORMAT("ramp end point does not finish at desired position values %f, so rejecting", (q1[idof]-rampnd.x1[idof]));
                            }
                            if( RaveFabs(dq1[idof]-segmentoutramps.back().dx1[idof]) > ParabolicRamp::EpsilonV ) {
                                // RAVELOG_VERBOSE_FORMAT("ramp end point does not finish at desired velocity values %e, so reforming ramp", (dq0[idof]-rampnd.dx1[idof]));
                                RAVELOG_VERBOSE_FORMAT("ramp end point does not finish at desired velocity values %e, so reforming ramp", (dq1[idof]-segmentoutramps.back().dx1[idof]));
                            }
                        }
                    }

                    outramps.insert(outramps.end(), segmentoutramps.begin(), segmentoutramps.end());
                    // the last ramp in segmentoutramps might not be exactly equal to q1/dq1!
                    q0 = segmentoutramps.back().x1;
                    dq0 = segmentoutramps.back().dx1;
                }
            }

            // have to make sure that the last ramp's ending velocity is equal to db
            //bool bDifferentPosition = false;
            bool bDifferentVelocity = false;
            for(size_t idof = 0; idof < q0.size(); ++idof) {
                if( RaveFabs(q0[idof]-rampnd.x1[idof]) > ParabolicRamp::EpsilonX ) {
                    RAVELOG_DEBUG_FORMAT("ramp end point does not finish at desired position values %f, so rejecting", (q0[idof]-rampnd.x1[idof]));
                    return ParabolicRamp::CheckReturn(CFO_FinalValuesNotReached);
                }
                if( RaveFabs(dq0[idof]-rampnd.dx1[idof]) > ParabolicRamp::EpsilonV ) {
                    RAVELOG_VERBOSE_FORMAT("ramp end point does not finish at desired velocity values %e, so reforming ramp", (dq0[idof]-rampnd.dx1[idof]));
                    bDifferentVelocity = true;
                }
            }

            ParabolicRamp::CheckReturn finalret(0);
            finalret.bDifferentVelocity = bDifferentVelocity;
            return finalret;
        }

private:
        std::vector<dReal> vswitchtimes;
        std::vector<dReal> q0, q1, dq0, dq1;
        std::vector<uint8_t> _vsearchsegments; ///< 1 if searched
        std::vector<ParabolicRamp::ParabolicRampND> segmentoutramps;
    };

public:
    ParabolicSmoother(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv), _feasibilitychecker(this)
    {
        __description = ":Interface Author: Rosen Diankov\n\nInterface to `Indiana University Intelligent Motion Laboratory <http://www.iu.edu/~motion/software.html>`_ parabolic smoothing library (Kris Hauser).\n\n**Note:** The original trajectory will not be preserved at all, don't use this if the robot has to hit all points of the trajectory.\n";
        _bmanipconstraints = false;
        _constraintreturn.reset(new ConstraintFilterReturn());
        _logginguniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
        if( !!_logginguniformsampler ) {
            _logginguniformsampler->SetSeed(utils::GetMicroTime());
        }
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        _parameters->copy(params);
        return _InitPlan();
    }

    virtual bool InitPlan(RobotBasePtr pbase, std::istream& isParameters)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        isParameters >> *_parameters;
        return _InitPlan();
    }

    bool _InitPlan()
    {
        if( _parameters->_nMaxIterations <= 0 ) {
            _parameters->_nMaxIterations = 100;
        }
        _bUsePerturbation = true;

        _bmanipconstraints = _parameters->manipname.size() > 0 && (_parameters->maxmanipspeed>0 || _parameters->maxmanipaccel>0);

        // initialize workspace constraints on manipulators
        if(_bmanipconstraints ) {
            if( !_manipconstraintchecker ) {
                _manipconstraintchecker.reset(new ManipConstraintChecker(GetEnv()));
            }
            _manipconstraintchecker->Init(_parameters->manipname, _parameters->_configurationspecification, _parameters->maxmanipspeed, _parameters->maxmanipaccel);
        }

        if( !_uniformsampler ) {
            _uniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
        }
        _uniformsampler->SetSeed(_parameters->_nRandomGeneratorSeed);
        return !!_uniformsampler;
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        BOOST_ASSERT(!!_parameters && !!ptraj);
        if( ptraj->GetNumWaypoints() < 2 ) {
            return PS_Failed;
        }

        // should always set the seed since smoother can be called with different trajectories even though InitPlan was only called once
        if( !!_uniformsampler ) {
            _uniformsampler->SetSeed(_parameters->_nRandomGeneratorSeed);
        }

        if( IS_DEBUGLEVEL(Level_Verbose) ) {
            // store the trajectory
            uint32_t randnum;
            if( !!_logginguniformsampler ) {
                randnum = _logginguniformsampler->SampleSequenceOneUInt32();
            }
            else {
                randnum = RaveRandomInt();
            }
            string filename = str(boost::format("%s/parabolicsmoother%d.parameters.xml")%RaveGetHomeDirectory()%(randnum%1000));
            ofstream f(filename.c_str());
            f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
            f << *_parameters;
            RAVELOG_VERBOSE_FORMAT("saved parabolic parameters to %s", filename);
        }
        _DumpTrajectory(ptraj, Level_Verbose);

        // save velocities
        std::vector<KinBody::KinBodyStateSaverPtr> vstatesavers;
        std::vector<KinBodyPtr> vusedbodies;
        _parameters->_configurationspecification.ExtractUsedBodies(GetEnv(), vusedbodies);
        if( vusedbodies.size() == 0 ) {
            RAVELOG_WARN("there are no used bodies in this configuration\n");
        }

        FOREACH(itbody, vusedbodies) {
            KinBody::KinBodyStateSaverPtr statesaver;
            if( (*itbody)->IsRobot() ) {
                statesaver.reset(new RobotBase::RobotStateSaver(RaveInterfaceCast<RobotBase>(*itbody), KinBody::Save_LinkTransformation|KinBody::Save_LinkEnable|KinBody::Save_ActiveDOF|KinBody::Save_ActiveManipulator|KinBody::Save_LinkVelocities));
            }
            else {
                statesaver.reset(new KinBody::KinBodyStateSaver(*itbody, KinBody::Save_LinkTransformation|KinBody::Save_LinkEnable|KinBody::Save_ActiveDOF|KinBody::Save_ActiveManipulator|KinBody::Save_LinkVelocities));
            }
            vstatesavers.push_back(statesaver);
        }

        uint32_t basetime = utils::GetMilliTime();
        ConfigurationSpecification posspec = _parameters->_configurationspecification;
        ConfigurationSpecification velspec = posspec.ConvertToVelocitySpecification();
        ConfigurationSpecification timespec;
        timespec.AddDeltaTimeGroup();

        std::vector<ConfigurationSpecification::Group>::const_iterator itcompatposgroup = ptraj->GetConfigurationSpecification().FindCompatibleGroup(posspec._vgroups.at(0), false);
        OPENRAVE_ASSERT_FORMAT(itcompatposgroup != ptraj->GetConfigurationSpecification()._vgroups.end(), "failed to find group %s in passed in trajectory", posspec._vgroups.at(0).name, ORE_InvalidArguments);

        ConstraintTrajectoryTimingParametersConstPtr parameters = boost::dynamic_pointer_cast<ConstraintTrajectoryTimingParameters const>(GetParameters());

        ParabolicRamp::DynamicPath &dynamicpath=_cachedynamicpath;
        dynamicpath.ramps.resize(0); // clear
        OPENRAVE_ASSERT_OP(parameters->_vConfigVelocityLimit.size(),==,parameters->_vConfigAccelerationLimit.size());
        OPENRAVE_ASSERT_OP((int)parameters->_vConfigVelocityLimit.size(),==,parameters->GetDOF());
        dynamicpath.Init(parameters->_vConfigVelocityLimit,parameters->_vConfigAccelerationLimit);
        dynamicpath._multidofinterp = _parameters->_multidofinterp;
        dynamicpath.SetJointLimits(parameters->_vConfigLowerLimit,parameters->_vConfigUpperLimit);

        bool bRampIsPerfectlyModeled = false;
        ParabolicRamp::Vector q(_parameters->GetDOF());
        vector<dReal> &vtrajpoints=_cachetrajpoints;
        if (_parameters->_hastimestamps && itcompatposgroup->interpolation == "quadratic" ) {
            RAVELOG_VERBOSE("Initial traj is piecewise quadratic\n");
            // assumes that the traj has velocity data and is consistent, so convert the original trajectory in a sequence of ramps, and preserve velocity
            vector<dReal> x0, x1, dx0, dx1, ramptime;
            ptraj->GetWaypoint(0,x0,posspec);
            ptraj->GetWaypoint(0,dx0,velspec);
            dynamicpath.ramps.resize(ptraj->GetNumWaypoints()-1);
            size_t iramp = 0;
            for(size_t i=0; i+1<ptraj->GetNumWaypoints(); i++) {
                ptraj->GetWaypoint(i+1,ramptime,timespec);
                if (ramptime.at(0) > g_fEpsilonLinear) {
                    ptraj->GetWaypoint(i+1,x1,posspec);
                    ptraj->GetWaypoint(i+1,dx1,velspec);
                    dynamicpath.ramps[iramp].SetPosVelTime(x0,dx0,x1,dx1,ramptime.at(0));
                    x0.swap(x1);
                    dx0.swap(dx1);
                    iramp += 1;
                }
            }
            dynamicpath.ramps.resize(iramp);
            bRampIsPerfectlyModeled = true;
        }
        else if (_parameters->_hastimestamps && itcompatposgroup->interpolation == "cubic" ) {
            RAVELOG_VERBOSE("Initial traj is piecewise cubic\n");
            // assumes that the traj has velocity data and is consistent, so convert the original trajectory in a sequence of ramps, and preserve velocity
            vector<dReal> x0, x1, dx0, dx1, ramptime;
            ptraj->GetWaypoint(0,x0,posspec);
            ptraj->GetWaypoint(0,dx0,velspec);
            dynamicpath.ramps.resize(ptraj->GetNumWaypoints()-1);
            size_t iramp = 0;
            for(size_t i=0; i+1<ptraj->GetNumWaypoints(); i++) {
                ptraj->GetWaypoint(i+1,ramptime,timespec);
                if (ramptime.at(0) > g_fEpsilonLinear) {
                    ptraj->GetWaypoint(i+1,x1,posspec);
                    ptraj->GetWaypoint(i+1,dx1,velspec);

                    // if the 3rd cubic coeff is 0, treat as tested parabolic ramp, otherwise have to re-compute it.
                    dReal ideltatime = 1.0/ramptime.at(0);
                    dReal ideltatime2 = ideltatime*ideltatime;
                    bool bIsParabolic = true;
                    for(size_t j = 0; j < x0.size(); ++j) {
                        dReal coeff = (-2.0*ideltatime*(x1[j] - x0[j]) + (dx1[j]+dx0[j]))*ideltatime2;
                        if( RaveFabs(coeff) > 1e-5 ) {
                            RAVELOG_VERBOSE_FORMAT("env=%d, ramp %d/%d dof %d cubic coeff=%15e is non-zero", GetEnv()->GetId()%iramp%dynamicpath.ramps.size()%j%coeff);
                            bIsParabolic = false;
                        }
                    }

                    dynamicpath.ramps[iramp].SetPosVelTime(x0,dx0,x1,dx1,ramptime.at(0));
                    if( bIsParabolic ) {
                        if( !_parameters->verifyinitialpath ) {
                            dynamicpath.ramps[iramp].constraintchecked = 1;
                        }
                    }
                    else {
                        // only check time based constraints since most of the collision checks here will change due to a different path. however it's important to have the ramp start with reasonable velocities/accelerations.
                        if( !_ValidateRamp(dynamicpath.ramps[iramp], CFO_CheckTimeBasedConstraints, iramp, dynamicpath.ramps.size()) ) {
                            RAVELOG_WARN("failed to initialize from cubic ramps\n");
                            _DumpTrajectory(ptraj, Level_Debug);
                            return PS_Failed;

                        }
                    }

                    x0.swap(x1);
                    dx0.swap(dx1);
                    iramp += 1;
                }
            }
            dynamicpath.ramps.resize(iramp);
        }
        else {
            if( itcompatposgroup->interpolation.size() == 0 || itcompatposgroup->interpolation == "linear" ) {
                bRampIsPerfectlyModeled = true;
            }

            // linear ramp or unknown
            vector<ParabolicRamp::Vector> &path=_cachepath; path.resize(0);
            if( path.capacity() < ptraj->GetNumWaypoints() ) {
                path.reserve(ptraj->GetNumWaypoints());
            }
            // linear piecewise trajectory
            ptraj->GetWaypoints(0,ptraj->GetNumWaypoints(),vtrajpoints,_parameters->_configurationspecification);
            for(size_t i = 0; i < ptraj->GetNumWaypoints(); ++i) {
                std::copy(vtrajpoints.begin()+i*_parameters->GetDOF(),vtrajpoints.begin()+(i+1)*_parameters->GetDOF(),q.begin());
                if( path.size() >= 2 ) {
                    // check if collinear by taking angle
                    const ParabolicRamp::Vector& x0 = path[path.size()-2];
                    const ParabolicRamp::Vector& x1 = path[path.size()-1];
                    dReal dotproduct=0,x0length2=0,x1length2=0;
                    for(size_t i = 0; i < q.size(); ++i) {
                        dReal dx0=x0[i]-q[i];
                        dReal dx1=x1[i]-q[i];
                        dotproduct += dx0*dx1;
                        x0length2 += dx0*dx0;
                        x1length2 += dx1*dx1;
                    }
                    if( RaveFabs(dotproduct * dotproduct - x0length2*x1length2) < 100*ParabolicRamp::EpsilonX*ParabolicRamp::EpsilonX ) {
                        path.back() = q;
                        continue;
                    }
                }
                // check if the point is not the same as the previous point
                if( path.size() > 0 ) {
                    dReal d = 0;
                    for(size_t i = 0; i < q.size(); ++i) {
                        d += RaveFabs(q[i]-path.back().at(i));
                    }
                    if( d <= q.size()*std::numeric_limits<dReal>::epsilon() ) {
                        continue;
                    }
                }
                path.push_back(q);
            }
            //dynamicpath.SetMilestones(path);   //now the trajectory starts and stops at every milestone
            if( !_SetMilestones(dynamicpath.ramps, path) ) {
                RAVELOG_WARN("failed to initialize ramps\n");
                _DumpTrajectory(ptraj, Level_Debug);
                return PS_Failed;
            }
            RAVELOG_DEBUG("Finish initializing the trajectory (via _SetMilestones)");
        }

        // if ramp is not perfectly modeled, then have to verify!
        if( !_parameters->verifyinitialpath && bRampIsPerfectlyModeled ) {
            // disable verification
            FOREACH(itramp, dynamicpath.ramps) {
                itramp->constraintchecked = 1;
            }
        }

        try {
            _bUsePerturbation = true;
            RAVELOG_DEBUG_FORMAT("env=%d, initial path size=%d, duration=%f, pointtolerance=%f, multidof=%d, manipname=%s, maxmanipspeed=%f, maxmanipaccel=%f", GetEnv()->GetId()%dynamicpath.ramps.size()%dynamicpath.GetTotalTime()%parameters->_pointtolerance%parameters->_multidofinterp%parameters->manipname%parameters->maxmanipspeed%parameters->maxmanipaccel);
            _feasibilitychecker.tol = parameters->_vConfigResolution;
            FOREACH(it, _feasibilitychecker.tol) {
                *it *= parameters->_pointtolerance;
            }

            _progress._iteration = 0;
            if( _CallCallbacks(_progress) == PA_Interrupt ) {
                return PS_Interrupted;
            }

            int numshortcuts=0;
            if( !!parameters->_setstatevaluesfn || !!parameters->_setstatefn ) {
                // no idea what a good mintimestep is... _parameters->_fStepLength*0.5?
                //numshortcuts = dynamicpath.Shortcut(parameters->_nMaxIterations,_feasibilitychecker,this, parameters->_fStepLength*0.99);
                if (IS_DEBUGLEVEL(Level_Verbose)) {
                    DumpDynamicPath(dynamicpath);
                }
                numshortcuts = _Shortcut(dynamicpath, parameters->_nMaxIterations,this, parameters->_fStepLength*0.99);
                if( numshortcuts < 0 ) {
                    return PS_Interrupted;
                }
                if (IS_DEBUGLEVEL(Level_Verbose)) {
                    DumpDynamicPath(dynamicpath);
                }
            }

            ++_progress._iteration;
            if( _CallCallbacks(_progress) == PA_Interrupt ) {
                return PS_Interrupted;
            }

            ConfigurationSpecification newspec = posspec;
            newspec.AddDerivativeGroups(1,true);
            int waypointoffset = newspec.AddGroup("iswaypoint", 1, "next");

            int timeoffset=-1;
            FOREACH(itgroup,newspec._vgroups) {
                if( itgroup->name == "deltatime" ) {
                    timeoffset = itgroup->offset;
                }
                else if( velspec.FindCompatibleGroup(*itgroup) != velspec._vgroups.end() ) {
                    itgroup->interpolation = "linear";
                }
                else if( posspec.FindCompatibleGroup(*itgroup) != posspec._vgroups.end() ) {
                    itgroup->interpolation = "quadratic";
                }
            }

            // have to write to another trajectory
            if( !_dummytraj || _dummytraj->GetXMLId() != ptraj->GetXMLId() ) {
                _dummytraj = RaveCreateTrajectory(GetEnv(), ptraj->GetXMLId());
            }
            _dummytraj->Init(newspec);

            // separate all the acceleration switches into individual points
            vtrajpoints.resize(newspec.GetDOF());
            OPENRAVE_ASSERT_OP((int)dynamicpath.ramps.at(0).x0.size(), ==, _parameters->GetDOF());
            ConfigurationSpecification::ConvertData(vtrajpoints.begin(),newspec,dynamicpath.ramps.at(0).x0.begin(), posspec,1,GetEnv(),true);
            ConfigurationSpecification::ConvertData(vtrajpoints.begin(),newspec,dynamicpath.ramps.at(0).dx0.begin(),velspec,1,GetEnv(),false);
            vtrajpoints.at(waypointoffset) = 1;
            vtrajpoints.at(timeoffset) = 0;
            _dummytraj->Insert(_dummytraj->GetNumWaypoints(),vtrajpoints);
            vector<dReal> &vswitchtimes=_cacheswitchtimes;
            ParabolicRamp::Vector vconfig;
            std::vector<ParabolicRamp::ParabolicRampND>& temprampsnd=_cacheoutramps;
            ParabolicRamp::ParabolicRampND rampndtrimmed;
            dReal fTrimEdgesTime = parameters->_fStepLength*2; // 2 controller timesteps is enough?
            dReal fExpectedDuration = 0;
            for(size_t irampindex = 0; irampindex < dynamicpath.ramps.size(); ++irampindex) {
                const ParabolicRamp::ParabolicRampND& rampnd = dynamicpath.ramps[irampindex];
                temprampsnd.resize(1);
                temprampsnd[0] = rampnd;
                // double-check the current ramps, ignore first and last ramps since they connect to the initial and goal positions, and those most likely they cannot be fixed .
                if(!rampnd.constraintchecked ) {
                    //(irampindex > 0 && irampindex+1 < dynamicpath.ramps.size())
                    rampndtrimmed = rampnd;
                    bool bTrimmed = false;
                    bool bCheck = true;
                    if( irampindex == 0 ) {
                        if( rampnd.endTime <= fTrimEdgesTime+g_fEpsilonLinear ) {
                            // ramp is too short so ignore checking
                            bCheck = false;
                        }
                        else {
                            // don't check points close to the initial configuration because of jittering
                            rampndtrimmed.TrimFront(fTrimEdgesTime);
                            bTrimmed = true;
                        }
                    }
                    else if( irampindex+1 == dynamicpath.ramps.size() ) {
                        if( rampnd.endTime <= fTrimEdgesTime+g_fEpsilonLinear ) {
                            // ramp is too short so ignore checking
                            bCheck = false;
                        }
                        else {
                            // don't check points close to the final configuration because of jittering
                            rampndtrimmed.TrimBack(fTrimEdgesTime);
                            bTrimmed = true;
                        }
                    }
                    // part of original trajectory which might not have been processed with perturbations, so ignore perturbations
                    _bUsePerturbation = false;
                    std::vector<ParabolicRamp::ParabolicRampND> outramps;
                    if( bCheck ) {
                        ParabolicRamp::CheckReturn checkret = _feasibilitychecker.Check2(rampndtrimmed, 0xffff, outramps);

                        if( checkret.retcode != 0 ) { // probably don't need to check bDifferentVelocity
                            std::vector<std::vector<ParabolicRamp::ParabolicRamp1D> > tempramps1d;
                            // try to time scale, perhaps collision and dynamics will change
                            // go all the way up to 2.0 multiplier: 1.05*1.1*1.15*1.2*1.25 ~= 2
                            bool bSuccess = false;
                            dReal endTime = rampndtrimmed.endTime;
                            dReal incr = 0.05*endTime;
                            // (Puttichai) now go up to 3.0 multiplier
                            // (Puttichai) try using an increment instead of a multiplier
                            size_t maxIncrement = 30;
                            for(size_t idilate = 0; idilate < maxIncrement; ++idilate ) {
                                RAVELOG_VERBOSE_FORMAT("env=%d, ramp %d, idilate=%d/%d", GetEnv()->GetId()%irampindex%idilate%maxIncrement);
                                tempramps1d.resize(0);
                                if( ParabolicRamp::SolveAccelBounded(rampndtrimmed.x0, rampndtrimmed.dx0, rampndtrimmed.x1, rampndtrimmed.dx1, endTime,  parameters->_vConfigAccelerationLimit, parameters->_vConfigVelocityLimit, parameters->_vConfigLowerLimit, parameters->_vConfigUpperLimit, tempramps1d, _parameters->_multidofinterp, rampndtrimmed.dx0.size()) ) {

                                    temprampsnd.resize(0);
                                    CombineRamps(tempramps1d, temprampsnd);

                                    if( temprampsnd[0].endTime > endTime*10 ) {
                                        RAVELOG_WARN_FORMAT("new end time=%fs is really big compared to old %fs!", temprampsnd[0].endTime%endTime);
                                        break;
                                    }

                                    endTime = temprampsnd[0].endTime;

                                    // not necessary to trim again!?
                                    //                                if( irampindex == 0 ) {
                                    //                                    temprampsnd[0].TrimFront(fTrimEdgesTime);
                                    //                                }
                                    //                                else if( irampindex+1 == dynamicpath.ramps.size() ) {
                                    //                                    temprampsnd[0].TrimBack(fTrimEdgesTime);
                                    //                                }
                                    bool bHasBadRamp=false;
                                    RAVELOG_VERBOSE_FORMAT("old endtime=%.15e, new endtime=%.15e", rampndtrimmed.endTime%endTime);
                                    FOREACH(itnewrampnd, temprampsnd) {
                                        ParabolicRamp::CheckReturn newrampret = _feasibilitychecker.Check2(*itnewrampnd, 0xffff, outramps);
                                        if( newrampret.retcode != 0 ) { // probably don't need to check bDifferentVelocity
                                            RAVELOG_VERBOSE_FORMAT("env=%d, has bad ramp retcode=0x%x", GetEnv()->GetId()%newrampret.retcode);

                                            bHasBadRamp = true;
                                            break;
                                        }
                                    }
                                    if( !bHasBadRamp ) {
                                        if( bTrimmed ) {
                                            // have to retime the original ramp without trimming
                                            // keep in mind that new ramps might be slower than endTime
                                            if( !ParabolicRamp::SolveAccelBounded(rampnd.x0, rampnd.dx0, rampnd.x1, rampnd.dx1, endTime,  parameters->_vConfigAccelerationLimit, parameters->_vConfigVelocityLimit, parameters->_vConfigLowerLimit, parameters->_vConfigUpperLimit, tempramps1d, _parameters->_multidofinterp, rampndtrimmed.dx0.size()) ) {
                                                break;
                                            }
                                            temprampsnd.resize(0);
                                            CombineRamps(tempramps1d, temprampsnd);
                                            endTime = temprampsnd[0].endTime;
                                        }
                                        bSuccess = true;
                                        break;
                                    }
                                }

                                // have to be very careful when incrementing endTime, sometimes it's small epsilons that preven the ramps from being computed and incrementing too much might make the trajectory duration jump by 100x!
                                if( idilate > 2 ) {
                                    endTime += incr;
                                }
                                else {
                                    endTime += ParabolicRamp::EpsilonT;
                                }
                            }
                            if( !bSuccess ) {
                                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                                    std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                                    ss << "x0=[";
                                    FOREACHC(itvalue, rampndtrimmed.x0) {
                                        ss << *itvalue << ",";
                                    }
                                    ss << "]; x1=[";
                                    FOREACHC(itvalue, rampndtrimmed.x1) {
                                        ss << *itvalue << ",";
                                    }
                                    ss << "]; dx0=[";
                                    FOREACHC(itvalue, rampndtrimmed.dx0) {
                                        ss << *itvalue << ",";
                                    }
                                    ss << "]; dx1=[";
                                    FOREACHC(itvalue, rampndtrimmed.dx1) {
                                        ss << *itvalue << ",";
                                    }
                                    ss << "]; deltatime=" << rampndtrimmed.endTime;
                                    ss << "; xmin=[";
                                    FOREACHC(itvalue, parameters->_vConfigLowerLimit) {
                                        ss << *itvalue << ",";
                                    }
                                    ss << "]; xmax=[";
                                    FOREACHC(itvalue, parameters->_vConfigUpperLimit) {
                                        ss << *itvalue << ",";
                                    }
                                    ss << "]; vellimits=[";
                                    FOREACHC(itvalue, parameters->_vConfigVelocityLimit) {
                                        ss << *itvalue << ",";
                                    }
                                    ss << "]; accellimits=[";
                                    FOREACHC(itvalue, parameters->_vConfigAccelerationLimit) {
                                        ss << *itvalue << ",";
                                    }
                                    ss << "];";
                                    RAVELOG_WARN_FORMAT("env=%d, original ramp %d/%d does not satisfy contraints. check retcode=0x%x! %s", GetEnv()->GetId()%irampindex%dynamicpath.ramps.size()%checkret.retcode%ss.str());
                                }
                                else {
                                    RAVELOG_WARN_FORMAT("env=%d, original ramp %d/%d does not satisfy contraints. check retcode=0x%x!", GetEnv()->GetId()%irampindex%dynamicpath.ramps.size()%checkret.retcode);
                                }
                                _DumpTrajectory(ptraj, Level_Verbose);
                                return PS_Failed;
                            }
                        }
                    }
                    _bUsePerturbation = true; // re-enable
                    ++_progress._iteration;
                    if( _CallCallbacks(_progress) == PA_Interrupt ) {
                        return PS_Interrupted;
                    }
                }

                FOREACH(itrampnd2, temprampsnd) {
                    fExpectedDuration += itrampnd2->endTime;
                    vswitchtimes.resize(0);
                    vswitchtimes.push_back(itrampnd2->endTime);
                    if( _parameters->_outputaccelchanges ) {
                        FOREACHC(itramp,itrampnd2->ramps) {
                            vector<dReal>::iterator it;
                            if( itramp->tswitch1 != 0 ) {
                                it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch1);
                                if( it != vswitchtimes.end() && *it != itramp->tswitch1) {
                                    vswitchtimes.insert(it,itramp->tswitch1);
                                }
                            }
                            if( itramp->tswitch1 != itramp->tswitch2 && itramp->tswitch2 != 0 ) {
                                it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch2);
                                if( it != vswitchtimes.end() && *it != itramp->tswitch2 ) {
                                    vswitchtimes.insert(it,itramp->tswitch2);
                                }
                            }
                            if( itramp->ttotal != itramp->tswitch2 && itramp->ttotal != 0 ) {
                                it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->ttotal);
                                if( it != vswitchtimes.end() && *it != itramp->ttotal ) {
                                    vswitchtimes.insert(it,itramp->ttotal);
                                }
                            }
                        }
                    }
                    vtrajpoints.resize(newspec.GetDOF()*vswitchtimes.size());
                    vector<dReal>::iterator ittargetdata = vtrajpoints.begin();
                    dReal prevtime = 0;
                    for(size_t i = 0; i < vswitchtimes.size(); ++i) {
                        itrampnd2->Evaluate(vswitchtimes[i],vconfig);
                        ConfigurationSpecification::ConvertData(ittargetdata,newspec,vconfig.begin(),posspec,1,GetEnv(),true);
                        itrampnd2->Derivative(vswitchtimes[i],vconfig);
                        ConfigurationSpecification::ConvertData(ittargetdata,newspec,vconfig.begin(),velspec,1,GetEnv(),false);
                        *(ittargetdata+timeoffset) = vswitchtimes[i]-prevtime;
                        *(ittargetdata+waypointoffset) = dReal(i+1==vswitchtimes.size());
                        ittargetdata += newspec.GetDOF();
                        prevtime = vswitchtimes[i];
                    }
                    _dummytraj->Insert(_dummytraj->GetNumWaypoints(),vtrajpoints);
                }

                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                    // if verbose, do tighter bound checking
                    OPENRAVE_ASSERT_OP(RaveFabs(fExpectedDuration-_dummytraj->GetDuration()),<,0.001);
                }
            }

            // dynamic path dynamicpath.GetTotalTime() could change if timing constraints get in the way, so use fExpectedDuration
            OPENRAVE_ASSERT_OP(RaveFabs(fExpectedDuration-_dummytraj->GetDuration()),<,0.01); // maybe because of trimming, will be a little different
            RAVELOG_DEBUG_FORMAT("env=%d, after shortcutting %d times: path waypoints=%d, traj waypoints=%d, traj time=%fs", GetEnv()->GetId()%numshortcuts%dynamicpath.ramps.size()%_dummytraj->GetNumWaypoints()%_dummytraj->GetDuration());
            //DumpDynamicPath(dynamicpath);
            ptraj->Swap(_dummytraj);
        }
        catch (const std::exception& ex) {
            _DumpTrajectory(ptraj, Level_Debug);
            RAVELOG_WARN_FORMAT("env=%d, parabolic planner failed, iter=%d: %s", GetEnv()->GetId()%_progress._iteration%ex.what());
            return PS_Failed;
        }
        RAVELOG_DEBUG_FORMAT("env=%d, path optimizing - computation time=%fs", GetEnv()->GetId()%(0.001f*(float)(utils::GetMilliTime()-basetime)));
        //====================================================================================================
        if (IS_DEBUGLEVEL(Level_Debug)) {
            RAVELOG_DEBUG("start sampling the trajectory (verification purpose) after shortcutting");
            // Actually _VerifySampling() gets called every time we sample a trajectory. The function
            // already checks _Validate* at every traj point. Therefore, in order to just verify, we
            // need to call ptraj->Sample just once.
            // RAVELOG_DEBUG_FORMAT("_timoffset = %d", ptraj->_timeoffset);
            std::vector<dReal> dummy;
            try {
                ptraj->Sample(dummy, 0);
                RAVELOG_DEBUG("sampling for verification successful");
            }
            catch (const std::exception& ex) {
                RAVELOG_WARN_FORMAT("sampling for verification failed: %s", ex.what());
                _DumpTrajectory(ptraj, Level_Debug);
            }
        }
        //====================================================================================================
        return _ProcessPostPlanners(RobotBasePtr(),ptraj);
    }

    virtual int ConfigFeasible(const ParabolicRamp::Vector& a, const ParabolicRamp::Vector& da, int options)
    {
        if( _bUsePerturbation ) {
            options |= CFO_CheckWithPerturbation;
        }
        try {
            return _parameters->CheckPathAllConstraints(a,a, da, da, 0, IT_OpenStart, options);
        }
        catch(const std::exception& ex) {
            // some constraints assume initial conditions for a and b are followed, however at this point a and b are sa
            RAVELOG_WARN_FORMAT("env=%d, rrtparams path constraints threw an exception: %s", GetEnv()->GetId()%ex.what());
            return 0xffff; // could be anything
        }
    }

    virtual ParabolicRamp::CheckReturn ConfigFeasible2(const ParabolicRamp::Vector& a, const ParabolicRamp::Vector& da, int options)
    {
        if( _bUsePerturbation ) {
            options |= CFO_CheckWithPerturbation;
        }
        try {
            int ret = _parameters->CheckPathAllConstraints(a,a, da, da, 0, IT_OpenStart, options);
            ParabolicRamp::CheckReturn checkret(ret);
            if( ret == CFO_CheckTimeBasedConstraints ) {
                checkret.fTimeBasedSurpassMult = 0.8; // don't have any other info, so just pick a multiple
            }
            return checkret;
        }
        catch(const std::exception& ex) {
            // some constraints assume initial conditions for a and b are followed, however at this point a and b are sa
            RAVELOG_WARN_FORMAT("env=%d, rrtparams path constraints threw an exception: %s", GetEnv()->GetId()%ex.what());
            return 0xffff; // could be anything...
        }
    }

    /// \brief checks a parabolic ramp and outputs smaller set of ramps. Because of manipulator constraints, the outramps's ending values might not be equal to b/db!
    virtual ParabolicRamp::CheckReturn SegmentFeasible2(const ParabolicRamp::Vector& a,const ParabolicRamp::Vector& b, const ParabolicRamp::Vector& da,const ParabolicRamp::Vector& db, dReal timeelapsed, int options, std::vector<ParabolicRamp::ParabolicRampND>& outramps)
    {
        outramps.resize(0);
        if( timeelapsed <= ParabolicRamp::EpsilonT ) {
            return ConfigFeasible2(a, da, options);
        }

        if( _bUsePerturbation ) {
            options |= CFO_CheckWithPerturbation;
        }
        bool bExpectModifiedConfigurations = _parameters->fCosManipAngleThresh > -1+g_fEpsilonLinear;
        if(bExpectModifiedConfigurations || _bmanipconstraints) {
            options |= CFO_FillCheckedConfiguration;
            _constraintreturn->Clear();
        }
        try {
            int ret = _parameters->CheckPathAllConstraints(a,b,da, db, timeelapsed, IT_OpenStart, options, _constraintreturn);
            if( ret != 0 ) {
                ParabolicRamp::CheckReturn checkret(ret);
                if( ret == CFO_CheckTimeBasedConstraints ) {
                    checkret.fTimeBasedSurpassMult = 0.8; // don't have any other info, so just pick a multiple
                }
                return checkret;
            }
        }
        catch(const std::exception& ex) {
            // some constraints assume initial conditions for a and b are followed, however at this point a and b are sa
            RAVELOG_WARN_FORMAT("env=%d, rrtparams path constraints threw an exception: %s", GetEnv()->GetId()%ex.what());
            return ParabolicRamp::CheckReturn(0xffff); // could be anything
        }
        // Test for collision and/or dynamics has succeeded, now test for manip constraint
        if( bExpectModifiedConfigurations ) {
            // the configurations are getting constrained, therefore the path checked is not equal to the simply interpolated path by (a,b, da, db).
            if( _constraintreturn->_configurationtimes.size() > 0 ) {
                OPENRAVE_ASSERT_OP(_constraintreturn->_configurations.size(),==,_constraintreturn->_configurationtimes.size()*a.size());
                outramps.resize(0);
                if( outramps.capacity() < _constraintreturn->_configurationtimes.size() ) {
                    outramps.reserve(_constraintreturn->_configurationtimes.size());
                }

                std::vector<dReal> curvel = da, newvel(a.size());
                std::vector<dReal> curpos = a, newpos(a.size());
                // _constraintreturn->_configurationtimes[0] is actually the end of the first segment since interval is IT_OpenStart
                std::vector<dReal>::const_iterator it = _constraintreturn->_configurations.begin();
                dReal curtime = 0;
                for(size_t itime = 0; itime < _constraintreturn->_configurationtimes.size(); ++itime, it += a.size()) {
                    std::copy(it, it+a.size(), newpos.begin());
                    dReal deltatime = _constraintreturn->_configurationtimes[itime]-curtime;
                    if( deltatime > g_fEpsilon ) {
                        dReal ideltatime = 1.0/deltatime;
                        for(size_t idof = 0; idof < newvel.size(); ++idof) {
                            newvel[idof] = 2*(newpos[idof] - curpos[idof])*ideltatime - curvel[idof];
                            if( RaveFabs(newvel[idof]) > _parameters->_vConfigVelocityLimit.at(idof)+ParabolicRamp::EpsilonV ) {
                                if( 0.9*_parameters->_vConfigVelocityLimit.at(idof) < 0.1*RaveFabs(newvel[idof]) ) {
                                    RAVELOG_WARN_FORMAT("new velocity for dof %d is too high %f > %f", idof%newvel[idof]%_parameters->_vConfigVelocityLimit.at(idof));
                                }
                                RAVELOG_VERBOSE_FORMAT("env=%d, retcode = 0x4; idof = %d; newvel[idof] = %.15e; vellimit = %.15e; g_fEpsilon = %.15e", GetEnv()->GetId()%idof%newvel[idof]%_parameters->_vConfigVelocityLimit.at(idof)%ParabolicRamp::EpsilonV);
                                return ParabolicRamp::CheckReturn(CFO_CheckTimeBasedConstraints, 0.9*_parameters->_vConfigVelocityLimit.at(idof)/RaveFabs(newvel[idof]));
                            }
                        }
                        ParabolicRamp::ParabolicRampND outramp;
                        outramp.SetPosVelTime(curpos, curvel, newpos, newvel, deltatime);
                        bool bAccelChanged = false;
                        // due to epsilon inaccuracies, have to clamp the max accel depending on _vConfigAccelerationLimit
                        for(size_t idof = 0; idof < outramp.ramps.size(); ++idof) {
                            if( outramp.ramps[idof].a1 < -_parameters->_vConfigAccelerationLimit[idof] ) {
                                RAVELOG_VERBOSE_FORMAT("env=%d, idof=%d, a1 changed: %15e -> %.15e, diff = %.15e", GetEnv()->GetId()%idof%outramp.ramps[idof].a1%(-_parameters->_vConfigAccelerationLimit[idof])%(outramp.ramps[idof].a1 + _parameters->_vConfigAccelerationLimit[idof]));
                                outramp.ramps[idof].a1 = -_parameters->_vConfigAccelerationLimit[idof];
                                bAccelChanged = true;
                            }
                            else if( outramp.ramps[idof].a1 > _parameters->_vConfigAccelerationLimit[idof] ) {
                                RAVELOG_VERBOSE_FORMAT("env=%d, idof=%d, a1 changed: %15e -> %.15e, diff = %.15e", GetEnv()->GetId()%idof%outramp.ramps[idof].a1%(_parameters->_vConfigAccelerationLimit[idof])%(outramp.ramps[idof].a1 - _parameters->_vConfigAccelerationLimit[idof]));
                                outramp.ramps[idof].a1 = _parameters->_vConfigAccelerationLimit[idof];
                                bAccelChanged = true;
                            }
                            if( outramp.ramps[idof].a2 < -_parameters->_vConfigAccelerationLimit[idof] ) {
                                RAVELOG_VERBOSE_FORMAT("env=%d, idof=%d, a2 changed: %15e -> %.15e, diff = %.15e", GetEnv()->GetId()%idof%outramp.ramps[idof].a2%(-_parameters->_vConfigAccelerationLimit[idof])%(outramp.ramps[idof].a2 + _parameters->_vConfigAccelerationLimit[idof]));
                                outramp.ramps[idof].a2 = -_parameters->_vConfigAccelerationLimit[idof];
                                bAccelChanged = true;
                            }
                            else if( outramp.ramps[idof].a2 > _parameters->_vConfigAccelerationLimit[idof] ) {
                                RAVELOG_VERBOSE_FORMAT("env=%d, idof=%d, a2 changed: %15e -> %.15e, diff = %.15e", GetEnv()->GetId()%idof%outramp.ramps[idof].a2%(_parameters->_vConfigAccelerationLimit[idof])%(outramp.ramps[idof].a2 - _parameters->_vConfigAccelerationLimit[idof]));
                                outramp.ramps[idof].a2 = _parameters->_vConfigAccelerationLimit[idof];
                                bAccelChanged = true;
                            }
                        }
                        if( bAccelChanged ) {
                            RAVELOG_VERBOSE_FORMAT("env=%d, deltatime = %.15e", GetEnv()->GetId()%deltatime);
                            if( !outramp.IsValid() ) {
                                RAVELOG_WARN_FORMAT("env=%d, ramp becomes invalid after changing acceleration limits", GetEnv()->GetId());
                                return ParabolicRamp::CheckReturn(CFO_CheckTimeBasedConstraints, 0.9); //?
                            }
                        }

                        outramp.constraintchecked = 1;
                        outramps.push_back(outramp);
                        curtime = _constraintreturn->_configurationtimes[itime];
                        curpos.swap(newpos);
                        curvel.swap(newvel);
                    }
                }

                // have to make sure that the last ramp's ending velocity is equal to db
                //bool bDifferentVelocity = false;
                for(size_t idof = 0; idof < curpos.size(); ++idof) {
                    if( RaveFabs(curpos[idof]-b[idof])+g_fEpsilon > ParabolicRamp::EpsilonX ) {
                        RAVELOG_WARN_FORMAT("env=%d, curpos[%d] (%.15e) != b[%d] (%.15e)", GetEnv()->GetId()%idof%curpos[idof]%idof%b[idof]);
                        return ParabolicRamp::CheckReturn(CFO_StateSettingError);
                    }
                }

//                // have to make sure that the last ramp's ending velocity is equal to db
//                bool bDifferentVelocity = false;
//                for(size_t idof = 0; idof < curvel.size(); ++idof) {
//                    if( RaveFabs(curvel[idof]-db[idof])+g_fEpsilon > ParabolicRamp::EpsilonV ) {
//                        bDifferentVelocity = true;
//                        break;
//                    }
//                }
//                if( bDifferentVelocity ) {
//                    // see if last ramp's time can be modified to accomodate the velocity
//                    ParabolicRamp::ParabolicRampND& outramp = outramps.at(outramps.size()-1);
//                    outramp.SetPosVelTime(outramp.x0, outramp.dx0, b, db, deltatime);
//                    outramp.constraintchecked = 1;
//                    outramps.push_back(outramp);
//                    curtime = _constraintreturn->_configurationtimes[itime];
//                    curpos.swap(newpos);
//                    curvel.swap(newvel);
//
//                }
            }
        }

        if( outramps.size() == 0 ) {
            ParabolicRamp::ParabolicRampND newramp;
            newramp.SetPosVelTime(a, da, b, db, timeelapsed);
            newramp.constraintchecked = 1;
            outramps.push_back(newramp);
        }

        if( _bmanipconstraints && (options & CFO_CheckTimeBasedConstraints) ) {
            try {
                ParabolicRamp::CheckReturn retmanip = _manipconstraintchecker->CheckManipConstraints2(outramps);
                if( retmanip.retcode != 0 ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, from CheckManipConstraints2: retcode = %d", GetEnv()->GetId()%retmanip.retcode);
                    return retmanip;
                }
            }
            catch(const std::exception& ex) {
                RAVELOG_WARN_FORMAT("CheckManipConstraints2 (modified=%d) threw an exception: %s", ((int)bExpectModifiedConfigurations)%ex.what());
                return 0xffff; // could be anything
            }
        }

        return ParabolicRamp::CheckReturn(0);
    }

    virtual ParabolicRamp::Real Rand()
    {
        return _uniformsampler->SampleSequenceOneReal(IT_OpenEnd);
    }

    virtual bool NeedDerivativeForFeasibility()
    {
        // always enable since CheckPathAllConstraints needs to interpolate quadratically
        return true;
    }

protected:
    /// \brief converts a path of linear points to a ramp that initially satisfies the constraints
    bool _SetMilestones(std::vector<ParabolicRamp::ParabolicRampND>& ramps, const vector<ParabolicRamp::Vector>& vpath)
    {
        size_t numdof = _parameters->GetDOF();
        ramps.resize(0);
        if(vpath.size()==1) {
            ramps.push_back(ParabolicRamp::ParabolicRampND());
            ramps.front().SetConstant(vpath[0]);
        }
        else if( vpath.size() > 1 ) {
            // only check time based constraints since most of the collision checks here will change due to a different path. however it's important to have the ramp start with reasonable velocities/accelerations.
            int options = CFO_CheckTimeBasedConstraints;
            if(!_parameters->verifyinitialpath) {
                options = options & (~CFO_CheckEnvCollisions) & (~CFO_CheckSelfCollisions); // no collision checking
                RAVELOG_VERBOSE_FORMAT("env=%d, Initial path verification is disabled using options=0x%x", GetEnv()->GetId()%options);
            }
            std::vector<dReal> vzero(numdof, 0.0);

            // in several cases when there are manipulator constraints, 0.5*(x0+x1) will not follow the constraints, instead of failing the plan, try to recompute a better midpoint
            std::vector<ParabolicRamp::Vector> vnewpath;
            std::vector<uint8_t> vforceinitialchecking(vpath.size(), 0);

            if( !!_parameters->_neighstatefn ) {
                std::vector<dReal> xmid(numdof), xmiddelta(numdof);
                vnewpath = vpath;
                int nConsecutiveExpansions = 0;
                size_t iwaypoint = 0;
                while(iwaypoint+1 < vnewpath.size() ) {
                    for(size_t idof = 0; idof < numdof; ++idof) {
                        xmiddelta.at(idof) = 0.5*(vnewpath[iwaypoint+1].at(idof) - vnewpath[iwaypoint].at(idof));
                    }
                    xmid = vnewpath[iwaypoint];
                    if( _parameters->SetStateValues(xmid) != 0 ) {
                        RAVELOG_WARN_FORMAT("env=%d, could not set values of path %d/%d", GetEnv()->GetId()%iwaypoint%vnewpath.size());
                        return false;
                    }
                    if( !_parameters->_neighstatefn(xmid, xmiddelta, NSO_OnlyHardConstraints) ) {
                        RAVELOG_WARN_FORMAT("env=%d, failed to get the neighbor of the midpoint of path %d/%d", GetEnv()->GetId()%iwaypoint%vnewpath.size());
                        return false;
                    }
                    // if the distance between xmid and the real midpoint is big, then have to add another point in vnewpath
                    dReal dist = 0;
                    for(size_t idof = 0; idof < numdof; ++idof) {
                        dReal fexpected = 0.5*(vnewpath[iwaypoint+1].at(idof) + vnewpath[iwaypoint].at(idof));
                        dReal ferror = fexpected - xmid[idof];
                        dist += ferror*ferror;
                    }
                    if( dist > 0.00001 ) {
                        RAVELOG_DEBUG_FORMAT("env=%d, adding extra midpoint at %d/%d since dist^2=%f", GetEnv()->GetId()%iwaypoint%vnewpath.size()%dist);
                        OPENRAVE_ASSERT_OP(xmid.size(),==,numdof);
                        vnewpath.insert(vnewpath.begin()+iwaypoint+1, xmid);
                        vforceinitialchecking[iwaypoint+1] = 1; // next point
                        vforceinitialchecking.insert(vforceinitialchecking.begin()+iwaypoint+1, 1); // just inserted point
                        nConsecutiveExpansions += 2;
                        if( nConsecutiveExpansions > 10 ) {
                            RAVELOG_WARN_FORMAT("env=%d, too many consecutive expansions, %d/%d is bad", GetEnv()->GetId()%iwaypoint%vnewpath.size());
                            return false;
                        }
                        continue;
                    }
                    if( nConsecutiveExpansions > 0 ) {
                        nConsecutiveExpansions--;
                    }
                    iwaypoint += 1;
                }
            }
            else {
                vnewpath=vpath;
            }

            ramps.resize(vnewpath.size()-1);
            for(size_t i=0; i+1<vnewpath.size(); i++) {
                ParabolicRamp::ParabolicRampND& ramp = ramps[i];
                OPENRAVE_ASSERT_OP(vnewpath[i].size(),==,numdof);
                ramp.x0 = vnewpath[i];
                ramp.x1 = vnewpath[i+1];
                ramp.dx0 = vzero;
                ramp.dx1 = vzero;
                if( !_ValidateRamp(ramp, options, i, vnewpath.size()) ) {
                    return false;
                }
                if( !_parameters->verifyinitialpath && !vforceinitialchecking.at(i) ) {
                    // disable future verification
                    ramp.constraintchecked = 1;
                }
            }
        }
        return true;
    }

    /// \brief validates a ramp with options
    bool _ValidateRamp(ParabolicRamp::ParabolicRampND& ramp, int options, int iramp=-1, int numramps=-1)
    {
        std::vector<dReal> &vswitchtimes=_cacheswitchtimes;
        std::vector<dReal> &x0=_x0cache, &x1=_x1cache, &dx0=_dx0cache, &dx1=_dx1cache;
        std::vector<dReal>& vellimits=_cachevellimits, &accellimits=_cacheaccellimits;
        vellimits = _parameters->_vConfigVelocityLimit;
        accellimits = _parameters->_vConfigAccelerationLimit;
        std::vector<ParabolicRamp::ParabolicRampND> &outramps=_cacheoutramps;
        //dReal fmult = 0.9;
        ParabolicRamp::CheckReturn retseg(0);
        for(size_t itry = 0; itry < 30; ++itry) {
            bool res=ramp.SolveMinTimeLinear(accellimits, vellimits);
            _ExtractSwitchTimes(ramp, vswitchtimes);
            ramp.Evaluate(0, x0);
            ramp.Derivative(0, dx0);
            dReal fprevtime = 0;
            size_t iswitch = 0;
            for(iswitch = 0; iswitch < vswitchtimes.size(); ++iswitch) {
                ramp.Evaluate(vswitchtimes.at(iswitch), x1);
                ramp.Derivative(vswitchtimes.at(iswitch), dx1);
                retseg = SegmentFeasible2(x0, x1, dx0, dx1, vswitchtimes.at(iswitch) - fprevtime, options, outramps);
//                        if( retseg.retcode == CFO_StateSettingError ) {
//                            // there's a bug with the checker function. given that this ramp has been validated to be ok and we're just checking time based constraints, can pass it
//                            std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
//                            ss << "x0=[";
//                            SerializeValues(ss, x0);
//                            ss << "]; x1=[";
//                            SerializeValues(ss, x1);
//                            ss << "]; dx0=[";
//                            SerializeValues(ss, dx0);
//                            ss << "]; dx1=[";
//                            SerializeValues(ss, dx1);
//                            ss << "]; deltatime=" << (vswitchtimes.at(iswitch) - fprevtime);
//                            RAVELOG_WARN_FORMAT("env=%d, initial ramp starting at %d/%d, switchtime=%f (%d/%d), returned a state error 0x%x; %s ignoring since we only care about time based constraints....", GetEnv()->GetId()%i%vnewpath.size()%vswitchtimes.at(iswitch)%iswitch%vswitchtimes.size()%retseg.retcode%ss.str());
//                            //retseg.retcode = 0;
//                        }
                if( retseg.retcode != 0 ) {
                    break;
                }
                x0.swap(x1);
                dx0.swap(dx1);
                fprevtime = vswitchtimes[iswitch];
            }
            if( retseg.retcode == 0 ) {
                break;
            }
            else if( retseg.retcode == CFO_CheckTimeBasedConstraints ) {
                // slow the ramp down and try again
                RAVELOG_VERBOSE_FORMAT("env=%d, slowing down ramp %d/%d by %.15e since too fast, try %d", GetEnv()->GetId()%iramp%numramps%retseg.fTimeBasedSurpassMult%itry);
                for(size_t j = 0; j < vellimits.size(); ++j) {
                    vellimits.at(j) *= retseg.fTimeBasedSurpassMult;
                    accellimits.at(j) *= retseg.fTimeBasedSurpassMult;
                }
            }
            else {
                std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                ss << "x0=[";
                SerializeValues(ss, x0);
                ss << "]; x1=[";
                SerializeValues(ss, x1);
                ss << "]; dx0=[";
                SerializeValues(ss, dx0);
                ss << "]; dx1=[";
                SerializeValues(ss, dx1);
                ss << "]; deltatime=" << (vswitchtimes.at(iswitch) - fprevtime);
                RAVELOG_WARN_FORMAT("initial ramp starting at %d/%d, switchtime=%f (%d/%d), returned error 0x%x; %s giving up....", iramp%numramps%vswitchtimes.at(iswitch)%iswitch%vswitchtimes.size()%retseg.retcode%ss.str());
                return false;
            }
        }
        if( retseg.retcode != 0 ) {
            // couldn't find anything...
            return false;
        }
        return true;
    }

    int _Shortcut(ParabolicRamp::DynamicPath& dynamicpath, int numIters, ParabolicRamp::RandomNumberGeneratorBase* rng, dReal mintimestep)
    {
        std::vector<ParabolicRamp::ParabolicRampND>& ramps = dynamicpath.ramps;
        int shortcuts = 0;
        vector<dReal> rampStartTime(ramps.size());
        dReal endTime=0;
        for(size_t i=0; i<ramps.size(); i++) {
            rampStartTime[i] = endTime;
            endTime += ramps[i].endTime;
        }
        dReal originalEndTime = endTime;
        dReal dummyEndTime;
        ParabolicRamp::Vector x0, x1, dx0, dx1;
        ParabolicRamp::DynamicPath &intermediate=_cacheintermediate, &intermediate2=_cacheintermediate2;
        std::vector<dReal>& vellimits=_cachevellimits, &accellimits=_cacheaccellimits;
        vellimits.resize(_parameters->_vConfigVelocityLimit.size());
        accellimits.resize(_parameters->_vConfigAccelerationLimit.size());
        std::vector<ParabolicRamp::ParabolicRampND>& accumoutramps=_cacheaccumoutramps, &outramps=_cacheoutramps, &outramps2=_cacheoutramps2;

        int numslowdowns = 0; // total number of times a ramp has been slowed down.
        bool bExpectModifiedConfigurations = _parameters->fCosManipAngleThresh > -1+g_fEpsilonLinear;

        dReal fiSearchVelAccelMult = 1.0/_parameters->fSearchVelAccelMult; // for slowing down when timing constraints
        dReal fstarttimemult = 1.0; // the start velocity/accel multiplier for the velocity and acceleration computations. If manip speed/accel or dynamics constraints are used, then this will track the last successful multipler. Basically if the last successful one is 0.1, it's very unlikely than a muliplier of 0.8 will meet the constraints the next time.
        int iters=0;
        for(iters=0; iters<numIters; iters++) {
            dReal t1=rng->Rand()*endTime,t2=rng->Rand()*endTime;
            if( iters == 0 ) {
                t1 = 0;
                t2 = endTime;
            }
            if(t1 > t2) {
                ParabolicRamp::Swap(t1,t2);
            }
            RAVELOG_VERBOSE_FORMAT("env = %d, shortcut iter = %d/%d, shortcutting from t1 = %.15e to t2 = %.15e", GetEnv()->GetId()%iters%numIters%t1%t2);
            if( t2 - t1 < mintimestep ) {
                continue;
            }
            int i1 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t1)-rampStartTime.begin()-1;
            int i2 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t2)-rampStartTime.begin()-1;
            // i1 can be equal to i2 and that is valid and should be rechecked again

            uint32_t iIterProgress = 0; // used for debug purposes
            try {
                //same ramp
                dReal u1 = t1-rampStartTime.at(i1); // at the same time check for boundaries
                dReal u2 = t2-rampStartTime.at(i2); // at the same time check for boundaries
                OPENRAVE_ASSERT_OP(u1, >=, 0);
                OPENRAVE_ASSERT_OP(u1, <=, ramps[i1].endTime+ParabolicRamp::EpsilonT);
                OPENRAVE_ASSERT_OP(u2, >=, 0);
                OPENRAVE_ASSERT_OP(u2, <=, ramps[i2].endTime+ParabolicRamp::EpsilonT);
                u1 = ParabolicRamp::Min(u1,ramps[i1].endTime);
                u2 = ParabolicRamp::Min(u2,ramps[i2].endTime);
                ramps[i1].Evaluate(u1,x0);
                if( _parameters->SetStateValues(x0) != 0 ) {
                    continue;
                }
                iIterProgress += 0x10000000;
                _parameters->_getstatefn(x0);
                iIterProgress += 0x10000000;
                ramps[i2].Evaluate(u2,x1);
                iIterProgress += 0x10000000;
                if( _parameters->SetStateValues(x1) != 0 ) {
                    continue;
                }
                iIterProgress += 0x10000000;
                _parameters->_getstatefn(x1);
                ramps[i1].Derivative(u1,dx0);
                ramps[i2].Derivative(u2,dx1);
                ++_progress._iteration;

                bool bsuccess = false;

                vellimits = _parameters->_vConfigVelocityLimit;
                accellimits = _parameters->_vConfigAccelerationLimit;
                if( _bmanipconstraints && !!_manipconstraintchecker ) {
                    if( _parameters->SetStateValues(x0) != 0 ) {
                        RAVELOG_VERBOSE("state set error\n");
                        continue;
                    }
                    _manipconstraintchecker->GetMaxVelocitiesAccelerations(dx0, vellimits, accellimits);
                    if( _parameters->SetStateValues(x1) != 0 ) {
                        RAVELOG_VERBOSE("state set error\n");
                        continue;
                    }
                    _manipconstraintchecker->GetMaxVelocitiesAccelerations(dx1, vellimits, accellimits);
                }
                for(size_t j = 0; j < _parameters->_vConfigVelocityLimit.size(); ++j) {
                    // have to watch out that velocities don't drop under dx0 & dx1!
                    dReal fminvel = max(RaveFabs(dx0[j]), RaveFabs(dx1[j]));
                    if( vellimits[j] < fminvel ) {
                        vellimits[j] = fminvel;
                    }
                    else {
                        dReal f = max(fminvel, _parameters->_vConfigVelocityLimit[j]*fstarttimemult);
                        if( vellimits[j] > f ) {
                            vellimits[j] = f;
                        }
                    }
                    {
                        dReal f = _parameters->_vConfigAccelerationLimit[j]*fstarttimemult;
                        if( accellimits[j] > f ) {
                            accellimits[j] = f;
                        }
                    }
                }

                dReal fcurmult = fstarttimemult;

                for(size_t islowdowntry = 0; islowdowntry < 4; ++islowdowntry ) {
                    bool res=ParabolicRamp::SolveMinTime(x0, dx0, x1, dx1, accellimits, vellimits, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, intermediate, _parameters->_multidofinterp);
                    iIterProgress += 0x1000;
                    if(!res) {
                        break;
                    }
                    // check the new ramp time makes significant steps
                    dReal newramptime = intermediate.GetTotalTime();
                    if( newramptime+mintimestep > t2-t1 ) {
                        // reject since it didn't make significant improvement
                        RAVELOG_VERBOSE_FORMAT("env=%d, shortcut iter=%d rejected times [%f, %f]. final trajtime=%fs", GetEnv()->GetId()%iters%t1%t2%(endTime-(t2-t1)+newramptime));
                        break;
                    }

                    if( _CallCallbacks(_progress) == PA_Interrupt ) {
                        return -1;
                    }

                    iIterProgress += 0x1000;
                    accumoutramps.resize(0);
                    ParabolicRamp::CheckReturn retcheck(0);
                    for(size_t iramp=0; iramp<intermediate.ramps.size(); iramp++) {
                        iIterProgress += 0x10;
                        if( iramp > 0 ) {
                            intermediate.ramps[iramp].x0 = intermediate.ramps[iramp-1].x1; // to remove noise?
                            intermediate.ramps[iramp].dx0 = intermediate.ramps[iramp-1].dx1; // to remove noise?
                        }
                        if( _parameters->SetStateValues(intermediate.ramps[iramp].x1) != 0 ) {
                            retcheck.retcode = CFO_StateSettingError;
                            break;
                        }
                        _parameters->_getstatefn(intermediate.ramps[iramp].x1);
                        // have to resolve for the ramp since the positions might have changed?
                        //                for(size_t j = 0; j < intermediate.rams[iramp].x1.size(); ++j) {
                        //                    intermediate.ramps[iramp].SolveFixedSwitchTime();
                        //                }

                        iIterProgress += 0x10;
                        retcheck = _feasibilitychecker.Check2(intermediate.ramps[iramp], 0xffff, outramps);
                        iIterProgress += 0x10;
                        if( retcheck.retcode != 0) {
                            break;
                        }

                        // if SegmentFeasible2 is modifying the original ramps due to jacobian project constraints inside of CheckPathAllConstraints, then have to reset the velocity and accel limits so that they are above the waypoints of the intermediate ramps.
                        if( bExpectModifiedConfigurations ) {
                            for(size_t i=0; i+1<outramps.size(); i++) {
                                for(size_t j = 0; j < outramps[i].x1.size(); ++j) {
                                    // have to watch out that velocities don't drop under dx0 & dx1!
                                    dReal fminvel = max(RaveFabs(outramps[i].dx0[j]), RaveFabs(outramps[i].dx1[j]));
                                    if( vellimits[j] < fminvel ) {
                                        vellimits[j] = fminvel;
                                    }

                                    // maybe do accel limits depending on (dx1-dx0)/elapsedtime?
                                }
                            }
                        }

                        if( IS_DEBUGLEVEL(Level_Verbose) ) {
                            for(size_t i=0; i+1<outramps.size(); i++) {
                                for(size_t j = 0; j < outramps[i].x1.size(); ++j) {
                                    OPENRAVE_ASSERT_OP(RaveFabs(outramps[i].x1[j]-outramps[i+1].x0[j]), <=, ParabolicRamp::EpsilonX);
                                    OPENRAVE_ASSERT_OP(RaveFabs(outramps[i].dx1[j]-outramps[i+1].dx0[j]), <=, ParabolicRamp::EpsilonV);
                                }
                            }
                        }

                        if( retcheck.bDifferentVelocity && outramps.size() > 0 ) {
                            ParabolicRamp::ParabolicRampND& outramp = outramps.at(outramps.size()-1);
                            dReal allowedstretchtime = (t2 - t1) - (newramptime + mintimestep);

                            bool res=ParabolicRamp::SolveMinTime(outramp.x0, outramp.dx0, intermediate.ramps[iramp].x1, intermediate.ramps[iramp].dx1, accellimits, vellimits, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, intermediate2, _parameters->_multidofinterp);
                            if( !res ) {
                                RAVELOG_WARN("failed to SolveMinTime for different vel ramp\n");
                                retcheck.retcode = CFO_FinalValuesNotReached;
                                break;
                            }
                            if( RaveFabs(intermediate2.GetTotalTime()-outramp.endTime) > allowedstretchtime) {
                                RAVELOG_DEBUG_FORMAT("env=%d, intermediate2 ramp duration is too long %fs", GetEnv()->GetId()%intermediate2.GetTotalTime());
                                retcheck.retcode = CFO_FinalValuesNotReached;
                                break;
                            }

                            // Check the newly interpolated segment. Note that intermediate2 should have ramps.size() == 1.
                            OPENRAVE_ASSERT_OP(intermediate2.ramps.size(), ==, 1);
                            retcheck = _feasibilitychecker.Check2(intermediate2.ramps[0], 0xffff, outramps2);
                            if (retcheck.retcode != 0) {
                                RAVELOG_WARN_FORMAT("env=%d, the final SolveMinTime generated infeasible segment retcode = 0x%x", GetEnv()->GetId()%retcheck.retcode);
                                // TODO probably never get here, so remove if not necessary
                                ParabolicRamp::ParabolicRampND &temp2 = intermediate2.ramps[0];
                                std::vector<dReal> &xmin = _parameters->_vConfigLowerLimit, &xmax = _parameters->_vConfigUpperLimit, &vmax = _parameters->_vConfigVelocityLimit, &amax = _parameters->_vConfigAccelerationLimit;
                                std::vector<std::vector<ParabolicRamp::ParabolicRamp1D> > tempramps1d;
                                std::vector<ParabolicRamp::ParabolicRampND> temprampsnd;
                                dReal endtime = temp2.endTime;
                                size_t maxIncrement = 3;
                                for (size_t idilation = 0; idilation < maxIncrement; ++idilation) {
                                    RAVELOG_DEBUG_FORMAT("env=%d, idilation = %d/%d", GetEnv()->GetId()%idilation%maxIncrement);
                                    bool res2 = ParabolicRamp::SolveAccelBounded(temp2.x0, temp2.dx0, temp2.x1, temp2.dx1, endtime, amax, vmax, xmin, xmax, tempramps1d, 0, 6);
                                    if (res2) {
                                        temprampsnd.resize(0);
                                        CombineRamps(tempramps1d, temprampsnd);
                                        if (temprampsnd[0].endTime > endTime*10) {
                                            RAVELOG_WARN_FORMAT("env=%d, new endtime (%.15e) is too big compared to prev endtime (%.15e)", GetEnv()->GetId()%temprampsnd[0].endTime%endtime);
                                            break;
                                        }
                                        endtime = temprampsnd[0].endTime;
                                        if (!(endtime < outramp.endTime + allowedstretchtime)) {
                                            // endtime has become too long that the shortcut is longer than t2 - t1
                                            break;
                                        }
                                        // Check feasibility
                                        bool bhasbadramp = false;
                                        RAVELOG_VERBOSE_FORMAT("env=%d, original endtime = %.15e; new endtime = %.15e", GetEnv()->GetId()%outramp.endTime%endtime);
                                        FOREACH(itnewrampnd, temprampsnd) {
                                            retcheck = _feasibilitychecker.Check2(*itnewrampnd, 0xffff, outramps2);
                                            if (retcheck.retcode != 0) {
                                                RAVELOG_VERBOSE_FORMAT("env=%d, has bad ramp: retcode = 0x%x", GetEnv()->GetId()%retcheck.retcode);
                                                bhasbadramp = true;
                                                break;
                                            }
                                            if (retcheck.bDifferentVelocity) {
                                                RAVELOG_VERBOSE_FORMAT("env=%d, still ends with different velocity. terminated", GetEnv()->GetId());
                                                retcheck.retcode = CFO_FinalValuesNotReached;
                                                break;
                                            }
                                        }
                                        if (!bhasbadramp) {
                                            RAVELOG_DEBUG_FORMAT("env=%d, idilation = %d/%d produces feasible ramp", GetEnv()->GetId()%idilation%maxIncrement);
                                            RAVELOG_DEBUG_FORMAT("env=%d, time increment = %.15e; allowed stretched time = %.15e", GetEnv()->GetId()%(endtime - outramp.endTime)%allowedstretchtime);
                                            break;
                                        }
                                    }
                                    // Here we increase endtime by only EpsilonT. As determined
                                    // empirically, only a few increments with EpsilonT
                                    // increment would be sufficient if the segment can be
                                    // feasible at all.
                                    endtime += ParabolicRamp::EpsilonT;

                                    if (!(endtime < outramp.endTime + allowedstretchtime)) {
                                        // endtime has become too long that the shortcut is longer than t2 - t1
                                        break;
                                    }
                                }
                                ////////////////////////////////////////////////////////////////////////////////
                                break;
                            }
                            else if (retcheck.bDifferentVelocity) {
                                // Give up and continue to the next iteration.
                                RAVELOG_DEBUG_FORMAT("env=%d, after Check2, intermediate2 does not end at the desired velocity", GetEnv()->GetId());
                                retcheck.retcode = CFO_FinalValuesNotReached;
                                break;
                            }
                            else {
                                // Otherwise, the new segment is good.
                                RAVELOG_DEBUG_FORMAT("env=%d, the final SolveMinTime generated feasible segment, inserting it to outramps", GetEnv()->GetId());
                                outramps.pop_back();
                                outramps.insert(outramps.end(), outramps2.begin(), outramps2.end());
                            }

                        }
                        else {
                            RAVELOG_DEBUG("new shortcut is aligned with boundary values after running Check2");
                        }
                        accumoutramps.insert(accumoutramps.end(), outramps.begin(), outramps.end());
                    }
                    iIterProgress += 0x1000;
                    if(retcheck.retcode == 0) {
                        bsuccess = true;
                        break;
                    }

                    if( retcheck.retcode == CFO_CheckTimeBasedConstraints ) {
                        RAVELOG_VERBOSE_FORMAT("env=%d, shortcut iter=%d, slow down ramp by fTimeBasedSurpassMult=%.15e, fcurmult=%.15e", GetEnv()->GetId()%iters%retcheck.fTimeBasedSurpassMult%fcurmult);
                        for(size_t j = 0; j < vellimits.size(); ++j) {
                            // have to watch out that velocities don't drop under dx0 & dx1!
                            dReal fminvel = max(RaveFabs(dx0[j]), RaveFabs(dx1[j]));
                            vellimits[j] = max(vellimits[j]*retcheck.fTimeBasedSurpassMult, fminvel);
                            accellimits[j] *= retcheck.fTimeBasedSurpassMult;
                        }
                        fcurmult *= retcheck.fTimeBasedSurpassMult;
                        if( fcurmult < 0.01 ) {
                            RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d, fcurmult is too small (%.15e) so giving up on this ramp", GetEnv()->GetId()%iters%fcurmult);
                            //retcheck = check.Check2(intermediate.ramps.at(0), 0xffff, outramps);
                            break;
                        }
                        numslowdowns += 1;
                    }
                    else {
                        RAVELOG_VERBOSE_FORMAT("env=%d, shortcut iter=%d rejected due to constraints 0x%x", GetEnv()->GetId()%iters%retcheck.retcode);
                        break;
                    }
                    iIterProgress += 0x1000;
                }

                if( !bsuccess ) {
                    continue;
                }

                if( accumoutramps.size() == 0 ) {
                    RAVELOG_WARN("accumulated ramps are empty!\n");
                    continue;
                }
                fstarttimemult = min(1.0, fcurmult*fiSearchVelAccelMult); // the new start time mult should be increased by one timemult

                // perform shortcut. use accumoutramps rather than intermediate.ramps!
                shortcuts++;

                if( i1 == i2 ) {
                    // the same ramp is being cut on both sides, so copy the ramp
                    ramps.insert(ramps.begin()+i1, ramps.at(i1));
                    i2 = i1+1;
                }

                ramps.at(i1).TrimBack(ramps[i1].endTime-u1); // use at for bounds checking
                ramps[i1].x1 = accumoutramps.front().x0;
                ramps[i1].dx1 = accumoutramps.front().dx0;
                ramps.at(i2).TrimFront(u2); // use at for bounds checking
                ramps[i2].x0 = accumoutramps.back().x1;
                ramps[i2].dx0 = accumoutramps.back().dx1;

                //RAVELOG_VERBOSE_FORMAT("replacing [%d, %d] with %d ramps", i1%i2%accumoutramps.size());
                // replace with accumoutramps
                if( i1+1 < i2 ) {
                    ramps.erase(ramps.begin()+i1+1, ramps.begin()+i2);
                }
                ramps.insert(ramps.begin()+i1+1,accumoutramps.begin(),accumoutramps.end());
                iIterProgress += 0x10000000;

                //check for consistency
                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                    for(size_t i=0; i+1<ramps.size(); i++) {
                        for(size_t j = 0; j < ramps[i].x1.size(); ++j) {
                            OPENRAVE_ASSERT_OP(RaveFabs(ramps[i].x1[j]-ramps[i+1].x0[j]), <=, ParabolicRamp::EpsilonX);
                            OPENRAVE_ASSERT_OP(RaveFabs(ramps[i].dx1[j]-ramps[i+1].dx0[j]), <=, ParabolicRamp::EpsilonV);
                        }
                    }
                }
                iIterProgress += 0x10000000;

                //revise the timing
                rampStartTime.resize(ramps.size());
                dummyEndTime = endTime;
                endTime=0;
                for(size_t i=0; i<ramps.size(); i++) {
                    rampStartTime[i] = endTime;
                    endTime += ramps[i].endTime;
                }
                RAVELOG_VERBOSE_FORMAT("env=%d, shortcut iter=%d slowdowns=%d, endTime: %.15e -> %.15e; diff = %.15e",GetEnv()->GetId()%iters%numslowdowns%dummyEndTime%endTime%(dummyEndTime - endTime));
                //DumpDynamicPath(dynamicpath);
            }
            catch(const std::exception& ex) {
                RAVELOG_WARN_FORMAT("env=%d, exception happened during shortcut iteration progress=0x%x: %s", GetEnv()->GetId()%iIterProgress%ex.what());
                // continue to next iteration...
            }
        }

        RAVELOG_DEBUG_FORMAT("finished at shortcut iter=%d, successful=%d, slowdowns=%d, endTime: %.15e -> %.15e; diff = %.15e",iters%shortcuts%numslowdowns%originalEndTime%endTime%(originalEndTime - endTime));
        return shortcuts;
    }

    /// \brief extracts the unique switch points for every 1D ramp. endtime is included.
    ///
    /// \param binitialized if false then 0 is *not* included.
    static void _ExtractSwitchTimes(const ParabolicRamp::ParabolicRampND& rampnd, std::vector<dReal>& vswitchtimes, bool bincludezero=false)
    {
        vswitchtimes.resize(0);
        if( bincludezero ) {
            vswitchtimes.push_back(0);
        }
        vswitchtimes.push_back(rampnd.endTime);
        FOREACHC(itramp,rampnd.ramps) {
            vector<dReal>::iterator it;
            if( itramp->tswitch1 != 0 ) {
                it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch1);
                if( it != vswitchtimes.end() && RaveFabs(*it - itramp->tswitch1) > ParabolicRamp::EpsilonT ) {
                    vswitchtimes.insert(it,itramp->tswitch1);
                }
            }
            if( RaveFabs(itramp->tswitch1 - itramp->tswitch2) > ParabolicRamp::EpsilonT && RaveFabs(itramp->tswitch2) > ParabolicRamp::EpsilonT ) {
                it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch2);
                if( it != vswitchtimes.end() && RaveFabs(*it - itramp->tswitch2) > ParabolicRamp::EpsilonT ) {
                    vswitchtimes.insert(it,itramp->tswitch2);
                }
            }
            if( RaveFabs(itramp->ttotal - itramp->tswitch2) > ParabolicRamp::EpsilonT && RaveFabs(itramp->ttotal) > ParabolicRamp::EpsilonT ) {
                it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->ttotal);
                if( it != vswitchtimes.end() && RaveFabs(*it - itramp->ttotal) > ParabolicRamp::EpsilonT ) {
                    vswitchtimes.insert(it,itramp->ttotal);
                }
            }
        }
    }

    std::string _DumpTrajectory(TrajectoryBasePtr traj, DebugLevel level)
    {
        if( IS_DEBUGLEVEL(level) ) {
            std::string filename = _DumpTrajectory(traj);
            RavePrintfA(str(boost::format("env=%d, wrote parabolicsmoothing trajectory to %s")%GetEnv()->GetId()%filename), level);
            return filename;
        }
        return std::string();
    }

    std::string _DumpTrajectory(TrajectoryBasePtr traj)
    {
        // store the trajectory
        uint32_t randnum;
        if( !!_logginguniformsampler ) {
            randnum = _logginguniformsampler->SampleSequenceOneUInt32();
        }
        else {
            randnum = RaveRandomInt();
        }
        string filename = str(boost::format("%s/parabolicsmoother%d.traj.xml")%RaveGetHomeDirectory()%(randnum%1000));
        ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        traj->serialize(f);
        return filename;
    }

    void DumpDynamicPath(ParabolicRamp::DynamicPath& path) const {
        uint32_t randnum;
        if( !!_logginguniformsampler ) {
            randnum = _logginguniformsampler->SampleSequenceOneUInt32();
        }
        else {
            randnum = RaveRandomInt();
        }
        string filename = str(boost::format("%s/dynamicpath%d.xml")%RaveGetHomeDirectory()%(randnum%1000));
        path.Save(filename);
        dReal duration = 0;
        for (std::vector<ParabolicRamp::ParabolicRampND>::const_iterator it = path.ramps.begin(); it != path.ramps.end(); ++it) {
            duration += it->endTime;
        }
        RAVELOG_DEBUG_FORMAT("Wrote a dynamic path to %s (duration = %.15e)", filename%duration);
    }

    ConstraintTrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _uniformsampler; ///< used for planning, seed is controlled
    SpaceSamplerBasePtr _logginguniformsampler; ///< used for logging, seed is random
    ConstraintFilterReturnPtr _constraintreturn;
    MyRampFeasibilityChecker _feasibilitychecker;
    boost::shared_ptr<ManipConstraintChecker> _manipconstraintchecker;

    //@{ cache
    ParabolicRamp::DynamicPath _cacheintermediate, _cacheintermediate2, _cachedynamicpath;
    std::vector<ParabolicRamp::ParabolicRampND> _cacheaccumoutramps, _cacheoutramps, _cacheoutramps2;
    std::vector<dReal> _cachetrajpoints, _cacheswitchtimes;
    vector<ParabolicRamp::Vector> _cachepath;
    std::vector<dReal> _cachevellimits, _cacheaccellimits;
    std::vector<dReal> _x0cache, _dx0cache, _x1cache, _dx1cache;
    //@}

    TrajectoryBasePtr _dummytraj;
    PlannerProgress _progress;
    bool _bUsePerturbation;
    bool _bmanipconstraints; /// if true, check workspace manip constraints
};


PlannerBasePtr CreateParabolicSmoother(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PlannerBasePtr(new ParabolicSmoother(penv,sinput));
}

} // using namespace rplanners

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(ParabolicRamp::ParabolicRamp1D)
BOOST_TYPEOF_REGISTER_TYPE(ParabolicRamp::ParabolicRampND)
#endif
