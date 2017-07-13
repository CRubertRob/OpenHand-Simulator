// -*- coding: utf-8 -*-
// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
#define NO_IMPORT_ARRAY
#include "openravepy_kinbody.h"

namespace openravepy {

class PyRobotBase : public PyKinBody
{
protected:
    RobotBasePtr _probot;
public:
    RobotBasePtr GetRobot() {
        return _probot;
    }

    class PyManipulatorInfo
    {
public:
        PyManipulatorInfo() {
            _tLocalTool = ReturnTransform(Transform());
            _vChuckingDirection = numeric::array(boost::python::list());
            _vdirection = toPyVector3(Vector(0,0,1));
            _vGripperJointNames = boost::python::list();
        }
        PyManipulatorInfo(const RobotBase::ManipulatorInfo& info) {
            _name = ConvertStringToUnicode(info._name);
            _sBaseLinkName = ConvertStringToUnicode(info._sBaseLinkName);
            _sEffectorLinkName = ConvertStringToUnicode(info._sEffectorLinkName);
            _tLocalTool = ReturnTransform(info._tLocalTool);
            _vChuckingDirection = toPyArray(info._vChuckingDirection);
            _vdirection = toPyVector3(info._vdirection);
            _sIkSolverXMLId = info._sIkSolverXMLId;
            boost::python::list vGripperJointNames;
            FOREACHC(itname, info._vGripperJointNames) {
                vGripperJointNames.append(ConvertStringToUnicode(*itname));
            }
            _vGripperJointNames = vGripperJointNames;
        }

        RobotBase::ManipulatorInfoPtr GetManipulatorInfo() const
        {
            RobotBase::ManipulatorInfoPtr pinfo(new RobotBase::ManipulatorInfo());
            pinfo->_name = boost::python::extract<std::string>(_name);
            pinfo->_sBaseLinkName = boost::python::extract<std::string>(_sBaseLinkName);
            pinfo->_sEffectorLinkName = boost::python::extract<std::string>(_sEffectorLinkName);
            pinfo->_tLocalTool = ExtractTransform(_tLocalTool);
            pinfo->_vChuckingDirection = ExtractArray<dReal>(_vChuckingDirection);
            pinfo->_vdirection = ExtractVector3(_vdirection);
            pinfo->_sIkSolverXMLId = _sIkSolverXMLId;
            pinfo->_vGripperJointNames = ExtractArray<std::string>(_vGripperJointNames);
            return pinfo;
        }

        object _name, _sBaseLinkName, _sEffectorLinkName;
        object _tLocalTool;
        object _vChuckingDirection;
        object _vdirection;
        std::string _sIkSolverXMLId;
        object _vGripperJointNames;
    };
    typedef boost::shared_ptr<PyManipulatorInfo> PyManipulatorInfoPtr;

    class PyManipulator
    {
        RobotBase::ManipulatorPtr _pmanip;
        PyEnvironmentBasePtr _pyenv;
public:
        PyManipulator(RobotBase::ManipulatorPtr pmanip, PyEnvironmentBasePtr pyenv) : _pmanip(pmanip),_pyenv(pyenv) {
        }
        virtual ~PyManipulator() {
        }

        RobotBase::ManipulatorPtr GetManipulator() const {
            return _pmanip;
        }

        object GetTransform() const {
            return ReturnTransform(_pmanip->GetTransform());
        }

        object GetTransformPose() const {
            return toPyArray(_pmanip->GetTransform());
        }

        object GetVelocity() const {
            std::pair<Vector, Vector> velocity;
            velocity = _pmanip->GetVelocity();
            boost::array<dReal,6> v = {{ velocity.first.x, velocity.first.y, velocity.first.z, velocity.second.x, velocity.second.y, velocity.second.z}};
            return toPyArray<dReal,6>(v);
        }

        object GetName() const {
            return ConvertStringToUnicode(_pmanip->GetName());
        }

        void SetName(const std::string& s) {
            _pmanip->SetName(s);
        }

        PyRobotBasePtr GetRobot() {
            return PyRobotBasePtr(new PyRobotBase(_pmanip->GetRobot(),_pyenv));
        }

        bool SetIkSolver(PyIkSolverBasePtr iksolver) {
            return _pmanip->SetIkSolver(openravepy::GetIkSolver(iksolver));
        }
        object GetIkSolver() {
            return object(openravepy::toPyIkSolver(_pmanip->GetIkSolver(),_pyenv));
        }

        object GetBase() {
            return toPyKinBodyLink(_pmanip->GetBase(),_pyenv);
        }
        object GetEndEffector() {
            return toPyKinBodyLink(_pmanip->GetEndEffector(),_pyenv);
        }
        object GetGraspTransform() {
            RAVELOG_WARN("Robot.Manipulator.GetGraspTransform deprecated, use GetLocalToolTransform\n");
            return ReturnTransform(_pmanip->GetLocalToolTransform());
        }
        object GetLocalToolTransform() {
            return ReturnTransform(_pmanip->GetLocalToolTransform());
        }
        object GetLocalToolTransformPose() {
            return toPyArray(_pmanip->GetLocalToolTransform());
        }
        void SetLocalToolTransform(object otrans) {
            _pmanip->SetLocalToolTransform(ExtractTransform(otrans));
        }
        void SetLocalToolDirection(object odirection) {
            _pmanip->SetLocalToolDirection(ExtractVector3(odirection));
        }
        void SetClosingDirection(object oclosingdirection)
        {
            RAVELOG_WARN("SetClosingDirection is deprecated, use SetChuckingDirection\n");
            _pmanip->SetChuckingDirection(ExtractArray<dReal>(oclosingdirection));
        }
        void SetChuckingDirection(object ochuckingdirection)
        {
            _pmanip->SetChuckingDirection(ExtractArray<dReal>(ochuckingdirection));
        }
        object GetGripperJoints() {
            RAVELOG_DEBUG("GetGripperJoints is deprecated, use GetGripperIndices\n");
            return toPyArray(_pmanip->GetGripperIndices());
        }
        object GetGripperIndices() {
            return toPyArray(_pmanip->GetGripperIndices());
        }
        object GetArmJoints() {
            RAVELOG_DEBUG("GetArmJoints is deprecated, use GetArmIndices\n");
            return toPyArray(_pmanip->GetArmIndices());
        }
        object GetArmIndices() {
            return toPyArray(_pmanip->GetArmIndices());
        }
        object GetArmDOFValues()
        {
            if( _pmanip->GetArmDOF() == 0 ) {
                return numeric::array(boost::python::list());
            }
            vector<dReal> values;
            _pmanip->GetArmDOFValues(values);
            return toPyArray(values);
        }
        object GetGripperDOFValues()
        {
            if( _pmanip->GetGripperDOF() == 0 ) {
                return numeric::array(boost::python::list());
            }
            vector<dReal> values;
            _pmanip->GetGripperDOFValues(values);
            return toPyArray(values);
        }
        int GetArmDOF() {
            return _pmanip->GetArmDOF();
        }
        int GetGripperDOF() {
            return _pmanip->GetGripperDOF();
        }
        object GetClosingDirection() {
            RAVELOG_WARN("GetClosingDirection is deprecated, use GetChuckingDirection\n");
            return toPyArray(_pmanip->GetChuckingDirection());
        }
        object GetChuckingDirection() {
            return toPyArray(_pmanip->GetChuckingDirection());
        }
        object GetPalmDirection() {
            RAVELOG_INFO("GetPalmDirection deprecated to GetDirection\n");
            return toPyVector3(_pmanip->GetDirection());
        }
        object GetDirection() {
            return toPyVector3(_pmanip->GetLocalToolDirection());
        }
        object GetLocalToolDirection() {
            return toPyVector3(_pmanip->GetLocalToolDirection());
        }
        bool IsGrabbing(PyKinBodyPtr pbody) {
            return _pmanip->IsGrabbing(pbody->GetBody());
        }

        int GetNumFreeParameters() const {
            RAVELOG_WARN("Manipulator::GetNumFreeParameters() is deprecated\n");
            return _pmanip->GetIkSolver()->GetNumFreeParameters();
        }

        object GetFreeParameters() const {
            RAVELOG_WARN("Manipulator::GetFreeParameters() is deprecated\n");
            if( _pmanip->GetIkSolver()->GetNumFreeParameters() == 0 ) {
                return numeric::array(boost::python::list());
            }
            vector<dReal> values;
            _pmanip->GetIkSolver()->GetFreeParameters(values);
            return toPyArray(values);
        }

        bool _FindIKSolution(const IkParameterization& ikparam, std::vector<dReal>& solution, int filteroptions, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolution(ikparam,solution,filteroptions);

        }
        bool _FindIKSolution(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, std::vector<dReal>& solution, int filteroptions, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolution(ikparam,vFreeParameters, solution,filteroptions);
        }
        bool _FindIKSolution(const IkParameterization& ikparam, int filteroptions, IkReturn& ikreturn, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolution(ikparam,filteroptions,IkReturnPtr(&ikreturn,utils::null_deleter()));
        }
        bool _FindIKSolution(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturn& ikreturn, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolution(ikparam,vFreeParameters, filteroptions,IkReturnPtr(&ikreturn,utils::null_deleter()));
        }

        bool _FindIKSolutions(const IkParameterization& ikparam, std::vector<std::vector<dReal> >& solutions, int filteroptions, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolutions(ikparam,solutions,filteroptions);
        }
        bool _FindIKSolutions(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, std::vector<std::vector<dReal> >& solutions, int filteroptions, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolutions(ikparam,vFreeParameters,solutions,filteroptions);
        }
        bool _FindIKSolutions(const IkParameterization& ikparam, int filteroptions, std::vector<IkReturnPtr>& vikreturns, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolutions(ikparam,filteroptions,vikreturns);
        }
        bool _FindIKSolutions(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector<IkReturnPtr>& vikreturns, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolutions(ikparam,vFreeParameters,filteroptions,vikreturns);
        }

        object FindIKSolution(object oparam, int filteroptions, bool ikreturn=false, bool releasegil=false) const
        {
            IkParameterization ikparam;
            EnvironmentMutex::scoped_lock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
            if( ExtractIkParameterization(oparam,ikparam) ) {
                if( ikreturn ) {
                    IkReturn ikreturn(IKRA_Reject);
                    _FindIKSolution(ikparam,filteroptions,ikreturn,releasegil);
                    return openravepy::toPyIkReturn(ikreturn);
                }
                else {
                    vector<dReal> solution;
                    if( !_FindIKSolution(ikparam,solution,filteroptions,releasegil) ) {
                        return object();
                    }
                    return toPyArray(solution);
                }
            }
            // assume transformation matrix
            else {
                if( ikreturn ) {
                    IkReturn ikreturn(IKRA_Reject);
                    _FindIKSolution(ExtractTransform(oparam),filteroptions,ikreturn,releasegil);
                    return openravepy::toPyIkReturn(ikreturn);
                }
                else {
                    vector<dReal> solution;
                    if( !_FindIKSolution(ExtractTransform(oparam),solution,filteroptions,releasegil) ) {
                        return object();
                    }
                    return toPyArray(solution);
                }
            }
        }

        object FindIKSolution(object oparam, object freeparams, int filteroptions, bool ikreturn=false, bool releasegil=false) const
        {
            vector<dReal> vfreeparams = ExtractArray<dReal>(freeparams);
            IkParameterization ikparam;
            EnvironmentMutex::scoped_lock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
            if( ExtractIkParameterization(oparam,ikparam) ) {
                if( ikreturn ) {
                    IkReturn ikreturn(IKRA_Reject);
                    _FindIKSolution(ikparam,vfreeparams,filteroptions,ikreturn,releasegil);
                    return openravepy::toPyIkReturn(ikreturn);
                }
                else {
                    vector<dReal> solution;
                    if( !_FindIKSolution(ikparam,vfreeparams,solution,filteroptions,releasegil) ) {
                        return object();
                    }
                    return toPyArray(solution);
                }
            }
            // assume transformation matrix
            else {
                if( ikreturn ) {
                    IkReturn ikreturn(IKRA_Reject);
                    _FindIKSolution(ikparam,vfreeparams,filteroptions,ikreturn,releasegil);
                    return openravepy::toPyIkReturn(ikreturn);
                }
                else {
                    vector<dReal> solution;
                    if( !_FindIKSolution(ExtractTransform(oparam),vfreeparams, solution,filteroptions,releasegil) ) {
                        return object();
                    }
                    return toPyArray(solution);
                }
            }
        }

        object FindIKSolutions(object oparam, int filteroptions, bool ikreturn=false, bool releasegil=false) const
        {
            IkParameterization ikparam;
            EnvironmentMutex::scoped_lock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
            if( ikreturn ) {
                std::vector<IkReturnPtr> vikreturns;
                if( ExtractIkParameterization(oparam,ikparam) ) {
                    if( !_FindIKSolutions(ikparam,filteroptions,vikreturns,releasegil) ) {
                        return boost::python::list();
                    }
                }
                // assume transformation matrix
                else if( !_FindIKSolutions(ExtractTransform(oparam),filteroptions,vikreturns,releasegil) ) {
                    return boost::python::list();
                }

                boost::python::list oikreturns;
                FOREACH(it,vikreturns) {
                    oikreturns.append(openravepy::toPyIkReturn(**it));
                }
                return oikreturns;
            }
            else {
                std::vector<std::vector<dReal> > vsolutions;
                if( ExtractIkParameterization(oparam,ikparam) ) {
                    if( !_FindIKSolutions(ikparam,vsolutions,filteroptions,releasegil) ) {
                        return numeric::array(boost::python::list());
                    }
                }
                // assume transformation matrix
                else if( !_FindIKSolutions(ExtractTransform(oparam),vsolutions,filteroptions,releasegil) ) {
                    return numeric::array(boost::python::list());
                }

                npy_intp dims[] = { npy_intp(vsolutions.size()), npy_intp(_pmanip->GetArmIndices().size()) };
                PyObject *pysolutions = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
                dReal* ppos = (dReal*)PyArray_DATA(pysolutions);
                FOREACH(itsol,vsolutions) {
                    BOOST_ASSERT(itsol->size()==size_t(dims[1]));
                    std::copy(itsol->begin(),itsol->end(),ppos);
                    ppos += itsol->size();
                }
                return static_cast<numeric::array>(handle<>(pysolutions));
            }
        }

        object FindIKSolutions(object oparam, object freeparams, int filteroptions, bool ikreturn=false, bool releasegil=false) const
        {
            vector<dReal> vfreeparams = ExtractArray<dReal>(freeparams);
            IkParameterization ikparam;
            EnvironmentMutex::scoped_lock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
            if( ikreturn ) {
                std::vector<IkReturnPtr> vikreturns;
                if( ExtractIkParameterization(oparam,ikparam) ) {
                    if( !_FindIKSolutions(ikparam,vfreeparams,filteroptions,vikreturns,releasegil) ) {
                        return boost::python::list();
                    }
                }
                // assume transformation matrix
                else if( !_FindIKSolutions(ExtractTransform(oparam),vfreeparams,filteroptions,vikreturns,releasegil) ) {
                    return boost::python::list();
                }

                boost::python::list oikreturns;
                FOREACH(it,vikreturns) {
                    oikreturns.append(openravepy::toPyIkReturn(**it));
                }
                return oikreturns;
            }
            else {
                std::vector<std::vector<dReal> > vsolutions;
                if( ExtractIkParameterization(oparam,ikparam) ) {
                    if( !_FindIKSolutions(ikparam,vfreeparams,vsolutions,filteroptions,releasegil) ) {
                        return numeric::array(boost::python::list());
                    }
                }
                // assume transformation matrix
                else if( !_FindIKSolutions(ExtractTransform(oparam),vfreeparams, vsolutions,filteroptions,releasegil) ) {
                    return numeric::array(boost::python::list());
                }

                npy_intp dims[] = { npy_intp(vsolutions.size()), npy_intp(_pmanip->GetArmIndices().size()) };
                PyObject *pysolutions = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
                dReal* ppos = (dReal*)PyArray_DATA(pysolutions);
                FOREACH(itsol,vsolutions) {
                    BOOST_ASSERT(itsol->size()==size_t(dims[1]));
                    std::copy(itsol->begin(),itsol->end(),ppos);
                    ppos += itsol->size();
                }
                return static_cast<numeric::array>(handle<>(pysolutions));
            }
        }

        object GetIkParameterization(object oparam, bool inworld=true)
        {
            IkParameterization ikparam;
            if( ExtractIkParameterization(oparam,ikparam) ) {
                return toPyIkParameterization(_pmanip->GetIkParameterization(ikparam,inworld));
            }
            // must be IkParameterizationType
            return toPyIkParameterization(_pmanip->GetIkParameterization((IkParameterizationType)extract<IkParameterizationType>(oparam),inworld));
        }

        object GetChildJoints() {
            std::vector<KinBody::JointPtr> vjoints;
            _pmanip->GetChildJoints(vjoints);
            boost::python::list joints;
            FOREACH(itjoint,vjoints) {
                joints.append(toPyKinBodyJoint(*itjoint,_pyenv));
            }
            return joints;
        }
        object GetChildDOFIndices() {
            std::vector<int> vdofindices;
            _pmanip->GetChildDOFIndices(vdofindices);
            boost::python::list dofindices;
            FOREACH(itindex,vdofindices) {
                dofindices.append(*itindex);
            }
            return dofindices;
        }

        object GetChildLinks() {
            std::vector<KinBody::LinkPtr> vlinks;
            _pmanip->GetChildLinks(vlinks);
            boost::python::list links;
            FOREACH(itlink,vlinks) {
                links.append(toPyKinBodyLink(*itlink,_pyenv));
            }
            return links;
        }

        bool IsChildLink(object pylink)
        {
            CHECK_POINTER(pylink);
            return _pmanip->IsChildLink(GetKinBodyLink(pylink));
        }

        object GetIndependentLinks() {
            std::vector<KinBody::LinkPtr> vlinks;
            _pmanip->GetIndependentLinks(vlinks);
            boost::python::list links;
            FOREACH(itlink,vlinks) {
                links.append(toPyKinBodyLink(*itlink,_pyenv));
            }
            return links;
        }

        object GetArmConfigurationSpecification(const std::string& interpolation="") const {
            return object(openravepy::toPyConfigurationSpecification(_pmanip->GetArmConfigurationSpecification(interpolation)));
        }

        object GetIkConfigurationSpecification(IkParameterizationType iktype, const std::string& interpolation="") const {
            return object(openravepy::toPyConfigurationSpecification(_pmanip->GetIkConfigurationSpecification(iktype, interpolation)));
        }

        bool CheckEndEffectorCollision(PyCollisionReportPtr pyreport) const
        {
            bool bcollision = _pmanip->CheckEndEffectorCollision(openravepy::GetCollisionReport(pyreport));
            openravepy::UpdateCollisionReport(pyreport,_pyenv);
            return bcollision;
        }

        bool CheckEndEffectorCollision(object otrans, PyCollisionReportPtr pyreport=PyCollisionReportPtr(), int numredundantsamples=0) const
        {
            bool bCollision;
            IkParameterization ikparam;
            if( ExtractIkParameterization(otrans,ikparam) ) {
                bCollision = _pmanip->CheckEndEffectorCollision(ikparam, !pyreport ? CollisionReportPtr() : openravepy::GetCollisionReport(pyreport), numredundantsamples);
            }
            else {
                bCollision = _pmanip->CheckEndEffectorCollision(ExtractTransform(otrans),!pyreport ? CollisionReportPtr() : openravepy::GetCollisionReport(pyreport), numredundantsamples);
            }
            if( !!pyreport ) {
                openravepy::UpdateCollisionReport(pyreport,_pyenv);
            }
            return bCollision;
        }
        bool CheckIndependentCollision() const
        {
            return _pmanip->CheckIndependentCollision();
        }
        bool CheckIndependentCollision(PyCollisionReportPtr pReport) const
        {
            bool bCollision = _pmanip->CheckIndependentCollision(openravepy::GetCollisionReport(pReport));
            openravepy::UpdateCollisionReport(pReport,_pyenv);
            return bCollision;
        }

        object CalculateJacobian()
        {
            std::vector<dReal> vjacobian;
            _pmanip->CalculateJacobian(vjacobian);
            std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _pmanip->GetArmIndices().size();
            return toPyArray(vjacobian,dims);
        }

        object CalculateRotationJacobian()
        {
            std::vector<dReal> vjacobian;
            _pmanip->CalculateRotationJacobian(vjacobian);
            std::vector<npy_intp> dims(2); dims[0] = 4; dims[1] = _pmanip->GetArmIndices().size();
            return toPyArray(vjacobian,dims);
        }

        object CalculateAngularVelocityJacobian()
        {
            std::vector<dReal> vjacobian;
            _pmanip->CalculateAngularVelocityJacobian(vjacobian);
            std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _pmanip->GetArmIndices().size();
            return toPyArray(vjacobian,dims);
        }

        object GetInfo() {
            return object(PyManipulatorInfoPtr(new PyManipulatorInfo(_pmanip->GetInfo())));
        }

        string GetStructureHash() const {
            return _pmanip->GetStructureHash();
        }
        string GetKinematicsStructureHash() const {
            return _pmanip->GetKinematicsStructureHash();
        }
        string GetInverseKinematicsStructureHash(IkParameterizationType iktype) const {
            return _pmanip->GetInverseKinematicsStructureHash(iktype);
        }

        string __repr__() {
            return boost::str(boost::format("RaveGetEnvironment(%d).GetRobot('%s').GetManipulator('%s')")%RaveGetEnvironmentId(_pmanip->GetRobot()->GetEnv())%_pmanip->GetRobot()->GetName()%_pmanip->GetName());
        }
        string __str__() {
            return boost::str(boost::format("<manipulator:%s, parent=%s>")%_pmanip->GetName()%_pmanip->GetRobot()->GetName());
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
        bool __eq__(boost::shared_ptr<PyManipulator> p) {
            return !!p && _pmanip==p->_pmanip;
        }
        bool __ne__(boost::shared_ptr<PyManipulator> p) {
            return !p || _pmanip!=p->_pmanip;
        }
        long __hash__() {
            return static_cast<long>(uintptr_t(_pmanip.get()));
        }
    };
    typedef boost::shared_ptr<PyManipulator> PyManipulatorPtr;
    PyManipulatorPtr _GetManipulator(RobotBase::ManipulatorPtr pmanip) {
        return !pmanip ? PyManipulatorPtr() : PyManipulatorPtr(new PyManipulator(pmanip,_pyenv));
    }

    class PyAttachedSensorInfo
    {
public:
        PyAttachedSensorInfo() {
        }
        PyAttachedSensorInfo(const RobotBase::AttachedSensorInfo& info) {
            _name = ConvertStringToUnicode(info._name);
            _linkname = ConvertStringToUnicode(info._linkname);
            _trelative = ReturnTransform(info._trelative);
            _sensorname = ConvertStringToUnicode(info._sensorname);
            _sensorgeometry = toPySensorGeometry(info._sensorgeometry);
        }

        RobotBase::AttachedSensorInfoPtr GetAttachedSensorInfo() const
        {
            RobotBase::AttachedSensorInfoPtr pinfo(new RobotBase::AttachedSensorInfo());
            pinfo->_name = boost::python::extract<std::string>(_name);
            pinfo->_linkname = boost::python::extract<std::string>(_linkname);
            pinfo->_trelative = ExtractTransform(_trelative);
            pinfo->_sensorname = boost::python::extract<std::string>(_sensorname);
            pinfo->_sensorgeometry = _sensorgeometry->GetGeometry();
            return pinfo;
        }

        object _name, _linkname;
        object _trelative;
        object _sensorname;
        PySensorGeometryPtr _sensorgeometry;
    };
    typedef boost::shared_ptr<PyAttachedSensorInfo> PyAttachedSensorInfoPtr;

    class PyAttachedSensor
    {
        RobotBase::AttachedSensorPtr _pattached;
        PyEnvironmentBasePtr _pyenv;
public:
        PyAttachedSensor(RobotBase::AttachedSensorPtr pattached, PyEnvironmentBasePtr pyenv) : _pattached(pattached),_pyenv(pyenv) {
        }
        virtual ~PyAttachedSensor() {
        }

        RobotBase::AttachedSensorPtr GetAttachedSensor() const {
            return _pattached;
        }
        object GetSensor() {
            return object(openravepy::toPySensor(_pattached->GetSensor(),_pyenv));
        }
        object GetAttachingLink() const {
            return toPyKinBodyLink(_pattached->GetAttachingLink(), _pyenv);
        }
        object GetRelativeTransform() const {
            return ReturnTransform(_pattached->GetRelativeTransform());
        }
        object GetTransform() const {
            return ReturnTransform(_pattached->GetTransform());
        }
        object GetTransformPose() const {
            return toPyArray(_pattached->GetTransform());
        }
        PyRobotBasePtr GetRobot() const {
            return _pattached->GetRobot() ? PyRobotBasePtr() : PyRobotBasePtr(new PyRobotBase(_pattached->GetRobot(), _pyenv));
        }
        object GetName() const {
            return ConvertStringToUnicode(_pattached->GetName());
        }

        object GetData()
        {
            return openravepy::toPySensorData(_pattached->GetSensor(),_pyenv);
        }

        void SetRelativeTransform(object transform) {
            _pattached->SetRelativeTransform(ExtractTransform(transform));
        }
        string GetStructureHash() const {
            return _pattached->GetStructureHash();
        }

        void UpdateInfo(SensorBase::SensorType type=SensorBase::ST_Invalid) {
            _pattached->UpdateInfo(type);
        }

        object UpdateAndGetInfo(SensorBase::SensorType type=SensorBase::ST_Invalid) {
            return object(PyAttachedSensorInfoPtr(new PyAttachedSensorInfo(_pattached->UpdateAndGetInfo(type))));
        }

        object GetInfo() {
            return object(PyAttachedSensorInfoPtr(new PyAttachedSensorInfo(_pattached->GetInfo())));
        }

        string __repr__() {
            return boost::str(boost::format("RaveGetEnvironment(%d).GetRobot('%s').GetAttachedSensor('%s')")%RaveGetEnvironmentId(_pattached->GetRobot()->GetEnv())%_pattached->GetRobot()->GetName()%_pattached->GetName());
        }
        string __str__() {
            return boost::str(boost::format("<attachedsensor:%s, parent=%s>")%_pattached->GetName()%_pattached->GetRobot()->GetName());
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
        bool __eq__(boost::shared_ptr<PyAttachedSensor> p) {
            return !!p && _pattached==p->_pattached;
        }
        bool __ne__(boost::shared_ptr<PyAttachedSensor> p) {
            return !p || _pattached!=p->_pattached;
        }
        long __hash__() {
            return static_cast<long>(uintptr_t(_pattached.get()));
        }
    };

    typedef boost::shared_ptr<PyAttachedSensor> PyAttachedSensorPtr;
    boost::shared_ptr<PyAttachedSensor> _GetAttachedSensor(RobotBase::AttachedSensorPtr pattachedsensor)
    {
        return !pattachedsensor ? PyAttachedSensorPtr() : PyAttachedSensorPtr(new PyAttachedSensor(pattachedsensor, _pyenv));
    }

    class PyGrabbedInfo
    {
public:
        PyGrabbedInfo() {
            _trelative = ReturnTransform(Transform());
        }
        PyGrabbedInfo(const RobotBase::GrabbedInfo& info) {
            _grabbedname = ConvertStringToUnicode(info._grabbedname);
            _robotlinkname = ConvertStringToUnicode(info._robotlinkname);
            _trelative = ReturnTransform(info._trelative);
            boost::python::list setRobotLinksToIgnore;
            FOREACHC(itindex, info._setRobotLinksToIgnore) {
                setRobotLinksToIgnore.append(*itindex);
            }
            _setRobotLinksToIgnore = setRobotLinksToIgnore;
        }

        RobotBase::GrabbedInfoPtr GetGrabbedInfo() const
        {
            RobotBase::GrabbedInfoPtr pinfo(new RobotBase::GrabbedInfo());
            pinfo->_grabbedname = boost::python::extract<std::string>(_grabbedname);
            pinfo->_robotlinkname = boost::python::extract<std::string>(_robotlinkname);
            pinfo->_trelative = ExtractTransform(_trelative);
            std::vector<int> v = ExtractArray<int>(_setRobotLinksToIgnore);
            pinfo->_setRobotLinksToIgnore.clear();
            FOREACHC(it,v) {
                pinfo->_setRobotLinksToIgnore.insert(*it);
            }
            return pinfo;
        }

        object _grabbedname, _robotlinkname;
        object _trelative;
        object _setRobotLinksToIgnore;
    };
    typedef boost::shared_ptr<PyGrabbedInfo> PyGrabbedInfoPtr;

    class PyRobotStateSaver
    {
        PyEnvironmentBasePtr _pyenv;
        RobotBase::RobotStateSaver _state;
public:
        PyRobotStateSaver(PyRobotBasePtr pyrobot) : _pyenv(pyrobot->GetEnv()), _state(pyrobot->GetRobot()) {
            // python should not support restoring on destruction since there's garbage collection
            _state.SetRestoreOnDestructor(false);

        }
        PyRobotStateSaver(PyRobotBasePtr pyrobot, object options) : _pyenv(pyrobot->GetEnv()), _state(pyrobot->GetRobot(),pyGetIntFromPy(options,0)) {
            // python should not support restoring on destruction since there's garbage collection
            _state.SetRestoreOnDestructor(false);
        }
        PyRobotStateSaver(RobotBasePtr probot, PyEnvironmentBasePtr pyenv) : _pyenv(pyenv), _state(probot) {
            // python should not support restoring on destruction since there's garbage collection
            _state.SetRestoreOnDestructor(false);
        }
        PyRobotStateSaver(RobotBasePtr probot, PyEnvironmentBasePtr pyenv, object options) : _pyenv(pyenv), _state(probot,pyGetIntFromPy(options,0)) {
            // python should not support restoring on destruction since there's garbage collection
            _state.SetRestoreOnDestructor(false);
        }
        virtual ~PyRobotStateSaver() {
        }

        object GetBody() const {
            return object(toPyRobot(RaveInterfaceCast<RobotBase>(_state.GetBody()),_pyenv));
        }

        void Restore(PyRobotBasePtr pyrobot=PyRobotBasePtr()) {
            _state.Restore(!pyrobot ? RobotBasePtr() : pyrobot->GetRobot());
        }

        void Release() {
            _state.Release();
        }

        std::string __str__() {
            KinBodyPtr pbody = _state.GetBody();
            if( !pbody ) {
                return "robot state empty";
            }
            return boost::str(boost::format("robot state for %s")%pbody->GetName());
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
    };
    typedef boost::shared_ptr<PyRobotStateSaver> PyRobotStateSaverPtr;

    PyRobotBase(RobotBasePtr probot, PyEnvironmentBasePtr pyenv) : PyKinBody(probot,pyenv), _probot(probot) {
    }
    PyRobotBase(const PyRobotBase &r) : PyKinBody(r._probot,r._pyenv) {
        _probot = r._probot;
    }
    virtual ~PyRobotBase() {
    }

    bool Init(object olinkinfos, object ojointinfos, object omanipinfos, object oattachedsensorinfos, const std::string& uri=std::string()) {
        std::vector<KinBody::LinkInfoConstPtr> vlinkinfos;
        _ParseLinkInfos(olinkinfos, vlinkinfos);
        std::vector<KinBody::JointInfoConstPtr> vjointinfos;
        _ParseJointInfos(ojointinfos, vjointinfos);
        std::vector<RobotBase::ManipulatorInfoConstPtr> vmanipinfos(len(omanipinfos));
        for(size_t i = 0; i < vmanipinfos.size(); ++i) {
            PyManipulatorInfoPtr pymanip = boost::python::extract<PyManipulatorInfoPtr>(omanipinfos[i]);
            if( !pymanip ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.ManipInfo"),ORE_InvalidArguments);
            }
            vmanipinfos[i] = pymanip->GetManipulatorInfo();
        }
        std::vector<RobotBase::AttachedSensorInfoConstPtr> vattachedsensorinfos(len(oattachedsensorinfos));
        for(size_t i = 0; i < vattachedsensorinfos.size(); ++i) {
            PyAttachedSensorInfoPtr pyattachedsensor = boost::python::extract<PyAttachedSensorInfoPtr>(oattachedsensorinfos[i]);
            if( !pyattachedsensor ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.AttachedsensorInfo"),ORE_InvalidArguments);
            }
            vattachedsensorinfos[i] = pyattachedsensor->GetAttachedSensorInfo();
        }
        return _probot->Init(vlinkinfos, vjointinfos, vmanipinfos, vattachedsensorinfos, uri);
    }

    object GetManipulators()
    {
        boost::python::list manips;
        FOREACH(it, _probot->GetManipulators()) {
            manips.append(_GetManipulator(*it));
        }
        return manips;
    }

    object GetManipulators(const string& manipname)
    {
        boost::python::list manips;
        FOREACH(it, _probot->GetManipulators()) {
            if( (*it)->GetName() == manipname ) {
                manips.append(_GetManipulator(*it));
            }
        }
        return manips;
    }
    PyManipulatorPtr GetManipulator(const string& manipname)
    {
        FOREACH(it, _probot->GetManipulators()) {
            if( (*it)->GetName() == manipname ) {
                return _GetManipulator(*it);
            }
        }
        return PyManipulatorPtr();
    }

    PyManipulatorPtr SetActiveManipulator(int index) {
        RAVELOG_WARN("SetActiveManipulator(int) is deprecated\n");
        _probot->SetActiveManipulator(index);
        return GetActiveManipulator();
    }
    PyManipulatorPtr SetActiveManipulator(const std::string& manipname) {
        _probot->SetActiveManipulator(manipname);
        return GetActiveManipulator();
    }
    PyManipulatorPtr SetActiveManipulator(PyManipulatorPtr pmanip) {
        _probot->SetActiveManipulator(pmanip->GetManipulator());
        return GetActiveManipulator();
    }
    PyManipulatorPtr GetActiveManipulator() {
        return _GetManipulator(_probot->GetActiveManipulator());
    }
    int GetActiveManipulatorIndex() const {
        RAVELOG_WARN("GetActiveManipulatorIndex is deprecated\n");
        return _probot->GetActiveManipulatorIndex();
    }
    PyManipulatorPtr AddManipulator(PyManipulatorInfoPtr pmanipinfo, bool removeduplicate=false) {
        return _GetManipulator(_probot->AddManipulator(*pmanipinfo->GetManipulatorInfo(), removeduplicate));
    }
    bool RemoveManipulator(PyManipulatorPtr pmanip) {
        return _probot->RemoveManipulator(pmanip->GetManipulator());
    }

    PyAttachedSensorPtr AddAttachedSensor(PyAttachedSensorInfoPtr pattsensorinfo, bool removeduplicate=false) {
        return _GetAttachedSensor(_probot->AddAttachedSensor(*pattsensorinfo->GetAttachedSensorInfo(), removeduplicate));
    }
    bool RemoveAttachedSensor(PyAttachedSensorPtr pyattsensor) {
        return _probot->RemoveAttachedSensor(pyattsensor->GetAttachedSensor());
    }

    object GetSensors()
    {
        RAVELOG_WARN("GetSensors is deprecated, please use GetAttachedSensors\n");
        return GetAttachedSensors();
    }

    object GetAttachedSensors()
    {
        boost::python::list sensors;
        FOREACH(itsensor, _probot->GetAttachedSensors()) {
            sensors.append(boost::shared_ptr<PyAttachedSensor>(new PyAttachedSensor(*itsensor,_pyenv)));
        }
        return sensors;
    }
    boost::shared_ptr<PyAttachedSensor> GetSensor(const string& sensorname)
    {
        RAVELOG_WARN("GetSensor is deprecated, please use GetAttachedSensor\n");
        return GetAttachedSensor(sensorname);
    }

    boost::shared_ptr<PyAttachedSensor> GetAttachedSensor(const string& sensorname)
    {
        return _GetAttachedSensor(_probot->GetAttachedSensor(sensorname));
    }

    object GetController() const {
        CHECK_POINTER(_probot);
        return object(openravepy::toPyController(_probot->GetController(),_pyenv));
    }

    bool SetController(PyControllerBasePtr pController, const string& args) {
        RAVELOG_WARN("RobotBase::SetController(PyControllerBasePtr,args) is deprecated\n");
        std::vector<int> dofindices;
        for(int i = 0; i < _probot->GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        return _probot->SetController(openravepy::GetController(pController),dofindices,1);
    }

    bool SetController(PyControllerBasePtr pController, object odofindices, int nControlTransformation) {
        CHECK_POINTER(pController);
        vector<int> dofindices = ExtractArray<int>(odofindices);
        return _probot->SetController(openravepy::GetController(pController),dofindices,nControlTransformation);
    }

    bool SetController(PyControllerBasePtr pController) {
        RAVELOG_VERBOSE("RobotBase::SetController(PyControllerBasePtr) will control all DOFs and transformation\n");
        std::vector<int> dofindices;
        for(int i = 0; i < _probot->GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        return _probot->SetController(openravepy::GetController(pController),dofindices,1);
    }

    void SetActiveDOFs(object dofindices) {
        _probot->SetActiveDOFs(ExtractArray<int>(dofindices));
    }
    void SetActiveDOFs(object dofindices, int nAffineDOsBitmask) {
        _probot->SetActiveDOFs(ExtractArray<int>(dofindices), nAffineDOsBitmask);
    }
    void SetActiveDOFs(object dofindices, int nAffineDOsBitmask, object rotationaxis) {
        _probot->SetActiveDOFs(ExtractArray<int>(dofindices), nAffineDOsBitmask, ExtractVector3(rotationaxis));
    }

    int GetActiveDOF() const {
        return _probot->GetActiveDOF();
    }
    int GetAffineDOF() const {
        return _probot->GetAffineDOF();
    }
    int GetAffineDOFIndex(DOFAffine dof) const {
        return _probot->GetAffineDOFIndex(dof);
    }

    object GetAffineRotationAxis() const {
        return toPyVector3(_probot->GetAffineRotationAxis());
    }
    void SetAffineTranslationLimits(object lower, object upper) {
        return _probot->SetAffineTranslationLimits(ExtractVector3(lower),ExtractVector3(upper));
    }
    void SetAffineRotationAxisLimits(object lower, object upper) {
        return _probot->SetAffineRotationAxisLimits(ExtractVector3(lower),ExtractVector3(upper));
    }
    void SetAffineRotation3DLimits(object lower, object upper) {
        return _probot->SetAffineRotation3DLimits(ExtractVector3(lower),ExtractVector3(upper));
    }
    void SetAffineRotationQuatLimits(object quatangle) {
        return _probot->SetAffineRotationQuatLimits(ExtractVector4(quatangle));
    }
    void SetAffineTranslationMaxVels(object vels) {
        _probot->SetAffineTranslationMaxVels(ExtractVector3(vels));
    }
    void SetAffineRotationAxisMaxVels(object vels) {
        _probot->SetAffineRotationAxisMaxVels(ExtractVector3(vels));
    }
    void SetAffineRotation3DMaxVels(object vels) {
        _probot->SetAffineRotation3DMaxVels(ExtractVector3(vels));
    }
    void SetAffineRotationQuatMaxVels(dReal vels) {
        _probot->SetAffineRotationQuatMaxVels(vels);
    }
    void SetAffineTranslationResolution(object resolution) {
        _probot->SetAffineTranslationResolution(ExtractVector3(resolution));
    }
    void SetAffineRotationAxisResolution(object resolution) {
        _probot->SetAffineRotationAxisResolution(ExtractVector3(resolution));
    }
    void SetAffineRotation3DResolution(object resolution) {
        _probot->SetAffineRotation3DResolution(ExtractVector3(resolution));
    }
    void SetAffineRotationQuatResolution(dReal resolution) {
        _probot->SetAffineRotationQuatResolution(resolution);
    }
    void SetAffineTranslationWeights(object weights) {
        _probot->SetAffineTranslationWeights(ExtractVector3(weights));
    }
    void SetAffineRotationAxisWeights(object weights) {
        _probot->SetAffineRotationAxisWeights(ExtractVector4(weights));
    }
    void SetAffineRotation3DWeights(object weights) {
        _probot->SetAffineRotation3DWeights(ExtractVector3(weights));
    }
    void SetAffineRotationQuatWeights(dReal weights) {
        _probot->SetAffineRotationQuatWeights(weights);
    }

    object GetAffineTranslationLimits() const
    {
        Vector lower, upper;
        _probot->GetAffineTranslationLimits(lower,upper);
        return boost::python::make_tuple(toPyVector3(lower),toPyVector3(upper));
    }
    object GetAffineRotationAxisLimits() const
    {
        Vector lower, upper;
        _probot->GetAffineRotationAxisLimits(lower,upper);
        return boost::python::make_tuple(toPyVector3(lower),toPyVector3(upper));
    }
    object GetAffineRotation3DLimits() const
    {
        Vector lower, upper;
        _probot->GetAffineRotation3DLimits(lower,upper);
        return boost::python::make_tuple(toPyVector3(lower),toPyVector3(upper));
    }
    object GetAffineRotationQuatLimits() const
    {
        return toPyVector4(_probot->GetAffineRotationQuatLimits());
    }
    object GetAffineTranslationMaxVels() const {
        return toPyVector3(_probot->GetAffineTranslationMaxVels());
    }
    object GetAffineRotationAxisMaxVels() const {
        return toPyVector3(_probot->GetAffineRotationAxisMaxVels());
    }
    object GetAffineRotation3DMaxVels() const {
        return toPyVector3(_probot->GetAffineRotation3DMaxVels());
    }
    dReal GetAffineRotationQuatMaxVels() const {
        return _probot->GetAffineRotationQuatMaxVels();
    }
    object GetAffineTranslationResolution() const {
        return toPyVector3(_probot->GetAffineTranslationResolution());
    }
    object GetAffineRotationAxisResolution() const {
        return toPyVector4(_probot->GetAffineRotationAxisResolution());
    }
    object GetAffineRotation3DResolution() const {
        return toPyVector3(_probot->GetAffineRotation3DResolution());
    }
    dReal GetAffineRotationQuatResolution() const {
        return _probot->GetAffineRotationQuatResolution();
    }
    object GetAffineTranslationWeights() const {
        return toPyVector3(_probot->GetAffineTranslationWeights());
    }
    object GetAffineRotationAxisWeights() const {
        return toPyVector4(_probot->GetAffineRotationAxisWeights());
    }
    object GetAffineRotation3DWeights() const {
        return toPyVector3(_probot->GetAffineRotation3DWeights());
    }
    dReal GetAffineRotationQuatWeights() const {
        return _probot->GetAffineRotationQuatWeights();
    }

    void SetActiveDOFValues(object values, uint32_t checklimits=KinBody::CLA_CheckLimits) const
    {
        vector<dReal> vvalues = ExtractArray<dReal>(values);
        if( vvalues.size() > 0 ) {
            _probot->SetActiveDOFValues(vvalues,checklimits);
        }
        else {
            OPENRAVE_ASSERT_OP_FORMAT((int)vvalues.size(),>=,_probot->GetActiveDOF(), "not enough values %d<%d",vvalues.size()%_probot->GetActiveDOF(),ORE_InvalidArguments);
        }
    }
    object GetActiveDOFValues() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> values;
        _probot->GetActiveDOFValues(values);
        return toPyArray(values);
    }

    object GetActiveDOFWeights() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> weights;
        _probot->GetActiveDOFWeights(weights);
        return toPyArray(weights);
    }

    void SetActiveDOFVelocities(object velocities, uint32_t checklimits=KinBody::CLA_CheckLimits)
    {
        _probot->SetActiveDOFVelocities(ExtractArray<dReal>(velocities), checklimits);
    }
    object GetActiveDOFVelocities() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> values;
        _probot->GetActiveDOFVelocities(values);
        return toPyArray(values);
    }

    object GetActiveDOFLimits() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return boost::python::make_tuple(numeric::array(boost::python::list()), numeric::array(boost::python::list())); // always need 2 since users can do lower, upper = GetDOFLimits()
        }
        vector<dReal> lower, upper;
        _probot->GetActiveDOFLimits(lower,upper);
        return boost::python::make_tuple(toPyArray(lower),toPyArray(upper));
    }

    object GetActiveDOFMaxVel() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> values;
        _probot->GetActiveDOFMaxVel(values);
        return toPyArray(values);
    }

    object GetActiveDOFMaxAccel() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> values;
        _probot->GetActiveDOFMaxAccel(values);
        return toPyArray(values);
    }

    object GetActiveDOFResolutions() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> values;
        _probot->GetActiveDOFResolutions(values);
        return toPyArray(values);
    }

    object GetActiveConfigurationSpecification(const std::string& interpolation="") const {
        return object(openravepy::toPyConfigurationSpecification(_probot->GetActiveConfigurationSpecification(interpolation)));
    }

    object GetActiveJointIndices() {
        RAVELOG_WARN("GetActiveJointIndices deprecated. Use GetActiveDOFIndices\n"); return toPyArray(_probot->GetActiveDOFIndices());
    }
    object GetActiveDOFIndices() {
        return toPyArray(_probot->GetActiveDOFIndices());
    }

    object SubtractActiveDOFValues(object ovalues0, object ovalues1)
    {
        vector<dReal> values0 = ExtractArray<dReal>(ovalues0);
        vector<dReal> values1 = ExtractArray<dReal>(ovalues1);
        BOOST_ASSERT((int)values0.size() == GetActiveDOF() );
        BOOST_ASSERT((int)values1.size() == GetActiveDOF() );
        _probot->SubtractActiveDOFValues(values0,values1);
        return toPyArray(values0);
    }

    object CalculateActiveJacobian(int index, object offset) const
    {
        std::vector<dReal> vjacobian;
        _probot->CalculateActiveJacobian(index,ExtractVector3(offset),vjacobian);
        std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _probot->GetActiveDOF();
        return toPyArray(vjacobian,dims);
    }

    object CalculateActiveRotationJacobian(int index, object q) const
    {
        std::vector<dReal> vjacobian;
        _probot->CalculateActiveRotationJacobian(index,ExtractVector4(q),vjacobian);
        std::vector<npy_intp> dims(2); dims[0] = 4; dims[1] = _probot->GetActiveDOF();
        return toPyArray(vjacobian,dims);
    }

    object CalculateActiveAngularVelocityJacobian(int index) const
    {
        std::vector<dReal> vjacobian;
        _probot->CalculateActiveAngularVelocityJacobian(index,vjacobian);
        std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _probot->GetActiveDOF();
        return toPyArray(vjacobian,dims);
    }

    bool Grab(PyKinBodyPtr pbody) {
        CHECK_POINTER(pbody); return _probot->Grab(pbody->GetBody());
    }
    bool Grab(PyKinBodyPtr pbody, object pylink_or_linkstoignore)
    {
        CHECK_POINTER(pbody);
        CHECK_POINTER(pylink_or_linkstoignore);
        KinBody::LinkPtr plink = GetKinBodyLink(pylink_or_linkstoignore);
        if( !!plink ) {
            return _probot->Grab(pbody->GetBody(), plink);
        }
        // maybe it is a set?
        std::set<int> setlinkstoignore = ExtractSet<int>(pylink_or_linkstoignore);
        return _probot->Grab(pbody->GetBody(), setlinkstoignore);
    }
    bool Grab(PyKinBodyPtr pbody, object pylink, object linkstoignore)
    {
        CHECK_POINTER(pbody);
        CHECK_POINTER(pylink);
        std::set<int> setlinkstoignore = ExtractSet<int>(linkstoignore);
        return _probot->Grab(pbody->GetBody(), GetKinBodyLink(pylink), setlinkstoignore);
    }
    void Release(PyKinBodyPtr pbody) {
        CHECK_POINTER(pbody); _probot->Release(pbody->GetBody());
    }
    void ReleaseAllGrabbed() {
        _probot->ReleaseAllGrabbed();
    }
    void RegrabAll() {
        _probot->RegrabAll();
    }
    object IsGrabbing(PyKinBodyPtr pbody) const {
        CHECK_POINTER(pbody);
        KinBody::LinkPtr plink = _probot->IsGrabbing(pbody->GetBody());
        return toPyKinBodyLink(plink,_pyenv);
    }

    object GetGrabbed() const
    {
        boost::python::list bodies;
        std::vector<KinBodyPtr> vbodies;
        _probot->GetGrabbed(vbodies);
        FOREACH(itbody, vbodies) {
            bodies.append(PyKinBodyPtr(new PyKinBody(*itbody,_pyenv)));
        }
        return bodies;
    }

    object GetGrabbedInfo() const
    {
        boost::python::list ograbbed;
        std::vector<RobotBase::GrabbedInfoPtr> vgrabbedinfo;
        _probot->GetGrabbedInfo(vgrabbedinfo);
        FOREACH(itgrabbed, vgrabbedinfo) {
            ograbbed.append(PyGrabbedInfoPtr(new PyGrabbedInfo(**itgrabbed)));
        }
        return ograbbed;
    }

    void ResetGrabbed(object ograbbedinfos)
    {
        std::vector<RobotBase::GrabbedInfoConstPtr> vgrabbedinfos(len(ograbbedinfos));
        for(size_t i = 0; i < vgrabbedinfos.size(); ++i) {
            PyGrabbedInfoPtr pygrabbed = boost::python::extract<PyGrabbedInfoPtr>(ograbbedinfos[i]);
            if( !pygrabbed ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to Robot.GrabbedInfo"),ORE_InvalidArguments);
            }
            vgrabbedinfos[i] = pygrabbed->GetGrabbedInfo();
        }
        _probot->ResetGrabbed(vgrabbedinfos);
    }

    bool WaitForController(float ftimeout)
    {
        ControllerBasePtr pcontroller = _probot->GetController();
        if( !pcontroller ) {
            return false;
        }
        if( pcontroller->IsDone() ) {
            return true;
        }
        bool bSuccess = true;
        Py_BEGIN_ALLOW_THREADS;

        try {
            uint64_t starttime = GetMicroTime();
            uint64_t deltatime = (uint64_t)(ftimeout*1000000.0);
            while( !pcontroller->IsDone() ) {
                Sleep(1);
                if(( deltatime > 0) &&( (GetMicroTime()-starttime)>deltatime) ) {
                    bSuccess = false;
                    break;
                }
            }
        }
        catch(...) {
            RAVELOG_ERROR("exception raised inside WaitForController:\n");
            PyErr_Print();
            bSuccess = false;
        }

        Py_END_ALLOW_THREADS;
        return bSuccess;
    }

    string GetRobotStructureHash() const {
        return _probot->GetRobotStructureHash();
    }

    PyStateRestoreContextBase* CreateRobotStateSaver(object options=object()) {
        PyRobotStateSaverPtr saver;
        if( IS_PYTHONOBJECT_NONE(options) ) {
            saver.reset(new PyRobotStateSaver(_probot,_pyenv));
        }
        else {
            saver.reset(new PyRobotStateSaver(_probot,_pyenv,options));
        }
        return new PyStateRestoreContext<PyRobotStateSaverPtr, PyRobotBasePtr>(saver);
    }

    virtual string __repr__() {
        return boost::str(boost::format("RaveGetEnvironment(%d).GetRobot('%s')")%RaveGetEnvironmentId(_probot->GetEnv())%_probot->GetName());
    }
    virtual string __str__() {
        return boost::str(boost::format("<%s:%s - %s (%s)>")%RaveGetInterfaceName(_probot->GetInterfaceType())%_probot->GetXMLId()%_probot->GetName()%_probot->GetRobotStructureHash());
    }
    virtual object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }
    virtual void __enter__()
    {
        // necessary to lock physics to prevent multiple threads from interfering
        if( _listStateSavers.size() == 0 ) {
            openravepy::LockEnvironment(_pyenv);
        }
        _listStateSavers.push_back(boost::shared_ptr<void>(new RobotBase::RobotStateSaver(_probot)));
    }
};

class ManipulatorInfo_pickle_suite : public pickle_suite
{
public:
    static boost::python::tuple getstate(const PyRobotBase::PyManipulatorInfo& r)
    {
        return boost::python::make_tuple(r._name, r._sBaseLinkName, r._sEffectorLinkName, r._tLocalTool, r._vChuckingDirection, r._vdirection, r._sIkSolverXMLId, r._vGripperJointNames);
    }
    static void setstate(PyRobotBase::PyManipulatorInfo& r, boost::python::tuple state) {
        r._name = state[0];
        r._sBaseLinkName = state[1];
        r._sEffectorLinkName = state[2];
        r._tLocalTool = state[3];
        r._vChuckingDirection = state[4];
        r._vdirection = state[5];
        r._sIkSolverXMLId = boost::python::extract<std::string>(state[6]);
        r._vGripperJointNames = state[7];
    }
};

class GrabbedInfo_pickle_suite : public pickle_suite
{
public:
    static boost::python::tuple getstate(const PyRobotBase::PyGrabbedInfo& r)
    {
        return boost::python::make_tuple(r._grabbedname, r._robotlinkname, r._trelative, r._setRobotLinksToIgnore);
    }
    static void setstate(PyRobotBase::PyGrabbedInfo& r, boost::python::tuple state) {
        r._grabbedname = state[0];
        r._robotlinkname = state[1];
        r._trelative = state[2];
        r._setRobotLinksToIgnore = state[3];
    }
};

RobotBasePtr GetRobot(object o)
{
    extract<PyRobotBasePtr> pyrobot(o);
    if( pyrobot.check() ) {
        return GetRobot((PyRobotBasePtr)pyrobot);
    }
    return RobotBasePtr();
}

RobotBasePtr GetRobot(PyRobotBasePtr pyrobot)
{
    return !pyrobot ? RobotBasePtr() : pyrobot->GetRobot();
}

PyInterfaceBasePtr toPyRobot(RobotBasePtr probot, PyEnvironmentBasePtr pyenv)
{
    return !probot ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyRobotBase(probot,pyenv));
}

RobotBase::ManipulatorPtr GetRobotManipulator(object o)
{
    extract<PyRobotBase::PyManipulatorPtr> pymanipulator(o);
    if( pymanipulator.check() ) {
        return ((PyRobotBase::PyManipulatorPtr)pymanipulator)->GetManipulator();
    }
    return RobotBase::ManipulatorPtr();
}

object toPyRobotManipulator(RobotBase::ManipulatorPtr pmanip, PyEnvironmentBasePtr pyenv)
{
    return !pmanip ? object() : object(PyRobotBase::PyManipulatorPtr(new PyRobotBase::PyManipulator(pmanip,pyenv)));
}

PyRobotBasePtr RaveCreateRobot(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    RobotBasePtr p = OpenRAVE::RaveCreateRobot(openravepy::GetEnvironment(pyenv), name);
    if( !p ) {
        return PyRobotBasePtr();
    }
    return PyRobotBasePtr(new PyRobotBase(p,pyenv));
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIkParameterization_overloads, GetIkParameterization, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckEndEffectorCollision_overloads, CheckEndEffectorCollision, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolution_overloads, FindIKSolution, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutionFree_overloads, FindIKSolution, 3, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutions_overloads, FindIKSolutions, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutionsFree_overloads, FindIKSolutions, 3, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetArmConfigurationSpecification_overloads, GetArmConfigurationSpecification, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIkConfigurationSpecification_overloads, GetIkConfigurationSpecification, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateRobotStateSaver_overloads, CreateRobotStateSaver, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetActiveDOFValues_overloads, SetActiveDOFValues, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetActiveDOFVelocities_overloads, SetActiveDOFVelocities, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddManipulator_overloads, AddManipulator, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddAttachedSensor_overloads, AddAttachedSensor, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetActiveConfigurationSpecification_overloads, GetActiveConfigurationSpecification, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Restore_overloads, Restore, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Init_overloads, Init, 4,5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(UpdateInfo_overloads, UpdateInfo, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(UpdateAndGetInfo_overloads, UpdateAndGetInfo, 0,1)

void init_openravepy_robot()
{
    object dofaffine = enum_<DOFAffine>("DOFAffine" DOXY_ENUM(DOFAffine))
                       .value("NoTransform",DOF_NoTransform)
                       .value("X",DOF_X)
                       .value("Y",DOF_Y)
                       .value("Z",DOF_Z)
                       .value("RotationAxis",DOF_RotationAxis)
                       .value("Rotation3D",DOF_Rotation3D)
                       .value("RotationQuat",DOF_RotationQuat)
                       .value("RotationMask",DOF_RotationMask)
                       .value("Transform",DOF_Transform)
    ;


    object manipulatorinfo = class_<PyRobotBase::PyManipulatorInfo, boost::shared_ptr<PyRobotBase::PyManipulatorInfo> >("ManipulatorInfo", DOXY_CLASS(RobotBase::ManipulatorInfo))
                             .def_readwrite("_name",&PyRobotBase::PyManipulatorInfo::_name)
                             .def_readwrite("_sBaseLinkName",&PyRobotBase::PyManipulatorInfo::_sBaseLinkName)
                             .def_readwrite("_sEffectorLinkName",&PyRobotBase::PyManipulatorInfo::_sEffectorLinkName)
                             .def_readwrite("_tLocalTool",&PyRobotBase::PyManipulatorInfo::_tLocalTool)
                             .def_readwrite("_vChuckingDirection",&PyRobotBase::PyManipulatorInfo::_vChuckingDirection)
                             .def_readwrite("_vClosingDirection",&PyRobotBase::PyManipulatorInfo::_vChuckingDirection) // back compat
                             .def_readwrite("_vdirection",&PyRobotBase::PyManipulatorInfo::_vdirection)
                             .def_readwrite("_sIkSolverXMLId",&PyRobotBase::PyManipulatorInfo::_sIkSolverXMLId)
                             .def_readwrite("_vGripperJointNames",&PyRobotBase::PyManipulatorInfo::_vGripperJointNames)
                             .def_pickle(ManipulatorInfo_pickle_suite())
    ;

    object attachedsensorinfo = class_<PyRobotBase::PyAttachedSensorInfo, boost::shared_ptr<PyRobotBase::PyAttachedSensorInfo> >("AttachedSensorInfo", DOXY_CLASS(RobotBase::AttachedSensorInfo))
                                .def_readwrite("_name", &PyRobotBase::PyAttachedSensorInfo::_name)
                                .def_readwrite("_linkname", &PyRobotBase::PyAttachedSensorInfo::_linkname)
                                .def_readwrite("_trelative", &PyRobotBase::PyAttachedSensorInfo::_trelative)
                                .def_readwrite("_sensorname", &PyRobotBase::PyAttachedSensorInfo::_sensorname)
                                .def_readwrite("_sensorgeometry", &PyRobotBase::PyAttachedSensorInfo::_sensorgeometry)
    ;

    object grabbedinfo = class_<PyRobotBase::PyGrabbedInfo, boost::shared_ptr<PyRobotBase::PyGrabbedInfo> >("GrabbedInfo", DOXY_CLASS(RobotBase::GrabbedInfo))
                         .def_readwrite("_grabbedname",&PyRobotBase::PyGrabbedInfo::_grabbedname)
                         .def_readwrite("_robotlinkname",&PyRobotBase::PyGrabbedInfo::_robotlinkname)
                         .def_readwrite("_trelative",&PyRobotBase::PyGrabbedInfo::_trelative)
                         .def_readwrite("_setRobotLinksToIgnore",&PyRobotBase::PyGrabbedInfo::_setRobotLinksToIgnore)
                         .def_pickle(GrabbedInfo_pickle_suite())
    ;

    {
        void (PyRobotBase::*psetactivedofs1)(object) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs2)(object, int) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs3)(object, int, object) = &PyRobotBase::SetActiveDOFs;

        bool (PyRobotBase::*pgrab1)(PyKinBodyPtr) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab2)(PyKinBodyPtr,object) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab4)(PyKinBodyPtr,object,object) = &PyRobotBase::Grab;

        PyRobotBase::PyManipulatorPtr (PyRobotBase::*setactivemanipulator1)(int) = &PyRobotBase::SetActiveManipulator;
        PyRobotBase::PyManipulatorPtr (PyRobotBase::*setactivemanipulator2)(const std::string&) = &PyRobotBase::SetActiveManipulator;
        PyRobotBase::PyManipulatorPtr (PyRobotBase::*setactivemanipulator3)(PyRobotBase::PyManipulatorPtr) = &PyRobotBase::SetActiveManipulator;

        object (PyRobotBase::*GetManipulators1)() = &PyRobotBase::GetManipulators;
        object (PyRobotBase::*GetManipulators2)(const string &) = &PyRobotBase::GetManipulators;
        bool (PyRobotBase::*setcontroller1)(PyControllerBasePtr,const string &) = &PyRobotBase::SetController;
        bool (PyRobotBase::*setcontroller2)(PyControllerBasePtr,object,int) = &PyRobotBase::SetController;
        bool (PyRobotBase::*setcontroller3)(PyControllerBasePtr) = &PyRobotBase::SetController;
        bool (PyRobotBase::*initrobot)(object, object, object, object, const std::string&) = &PyRobotBase::Init;
        scope robot = class_<PyRobotBase, boost::shared_ptr<PyRobotBase>, bases<PyKinBody, PyInterfaceBase> >("Robot", DOXY_CLASS(RobotBase), no_init)
                      .def("Init", initrobot, Init_overloads(args("linkinfos", "jointinfos", "manipinfos", "attachedsensorinfos", "uri"), DOXY_FN(RobotBase, Init)))
                      .def("GetManipulators",GetManipulators1, DOXY_FN(RobotBase,GetManipulators))
                      .def("GetManipulators",GetManipulators2,args("manipname"), DOXY_FN(RobotBase,GetManipulators))
                      .def("GetManipulator",&PyRobotBase::GetManipulator,args("manipname"), "Return the manipulator whose name matches")
                      .def("SetActiveManipulator",setactivemanipulator1,args("manipindex"), DOXY_FN(RobotBase,SetActiveManipulator "int"))
                      .def("SetActiveManipulator",setactivemanipulator2,args("manipname"), DOXY_FN(RobotBase,SetActiveManipulator "const std::string"))
                      .def("SetActiveManipulator",setactivemanipulator3,args("manip"), "Set the active manipulator given a pointer")
                      .def("GetActiveManipulator",&PyRobotBase::GetActiveManipulator, DOXY_FN(RobotBase,GetActiveManipulator))
                      .def("AddManipulator",&PyRobotBase::AddManipulator, AddManipulator_overloads(args("manipinfo", "removeduplicate"), DOXY_FN(RobotBase,AddManipulator)))
                      .def("AddAttachedSensor",&PyRobotBase::AddAttachedSensor, AddAttachedSensor_overloads(args("attachedsensorinfo", "removeduplicate"), DOXY_FN(RobotBase,AddAttachedSensor)))
                      .def("RemoveAttachedSensor",&PyRobotBase::RemoveAttachedSensor, args("attsensor"), DOXY_FN(RobotBase,RemoveAttachedSensor))
                      .def("RemoveManipulator",&PyRobotBase::RemoveManipulator, args("manip"), DOXY_FN(RobotBase,RemoveManipulator))
                      .def("GetActiveManipulatorIndex",&PyRobotBase::GetActiveManipulatorIndex, DOXY_FN(RobotBase,GetActiveManipulatorIndex))
                      .def("GetAttachedSensors",&PyRobotBase::GetAttachedSensors, DOXY_FN(RobotBase,GetAttachedSensors))
                      .def("GetAttachedSensor",&PyRobotBase::GetAttachedSensor,args("sensorname"), "Return the attached sensor whose name matches")
                      .def("GetSensors",&PyRobotBase::GetSensors)
                      .def("GetSensor",&PyRobotBase::GetSensor,args("sensorname"))
                      .def("GetController",&PyRobotBase::GetController, DOXY_FN(RobotBase,GetController))
                      .def("SetController",setcontroller1,DOXY_FN(RobotBase,SetController))
                      .def("SetController",setcontroller2,args("robot","dofindices","controltransform"), DOXY_FN(RobotBase,SetController))
                      .def("SetController",setcontroller3,DOXY_FN(RobotBase,SetController))
                      .def("SetActiveDOFs",psetactivedofs1,args("dofindices"), DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int"))
                      .def("SetActiveDOFs",psetactivedofs2,args("dofindices","affine"), DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int"))
                      .def("SetActiveDOFs",psetactivedofs3,args("dofindices","affine","rotationaxis"), DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int; const Vector"))
                      .def("GetActiveDOF",&PyRobotBase::GetActiveDOF, DOXY_FN(RobotBase,GetActiveDOF))
                      .def("GetAffineDOF",&PyRobotBase::GetAffineDOF, DOXY_FN(RobotBase,GetAffineDOF))
                      .def("GetAffineDOFIndex",&PyRobotBase::GetAffineDOFIndex,args("index"), DOXY_FN(RobotBase,GetAffineDOFIndex))
                      .def("GetAffineRotationAxis",&PyRobotBase::GetAffineRotationAxis, DOXY_FN(RobotBase,GetAffineRotationAxis))
                      .def("SetAffineTranslationLimits",&PyRobotBase::SetAffineTranslationLimits,args("lower","upper"), DOXY_FN(RobotBase,SetAffineTranslationLimits))
                      .def("SetAffineRotationAxisLimits",&PyRobotBase::SetAffineRotationAxisLimits,args("lower","upper"), DOXY_FN(RobotBase,SetAffineRotationAxisLimits))
                      .def("SetAffineRotation3DLimits",&PyRobotBase::SetAffineRotation3DLimits,args("lower","upper"), DOXY_FN(RobotBase,SetAffineRotation3DLimits))
                      .def("SetAffineRotationQuatLimits",&PyRobotBase::SetAffineRotationQuatLimits,args("quatangle"), DOXY_FN(RobotBase,SetAffineRotationQuatLimits))
                      .def("SetAffineTranslationMaxVels",&PyRobotBase::SetAffineTranslationMaxVels,args("lower","upper"), DOXY_FN(RobotBase,SetAffineTranslationMaxVels))
                      .def("SetAffineRotationAxisMaxVels",&PyRobotBase::SetAffineRotationAxisMaxVels,args("velocity"), DOXY_FN(RobotBase,SetAffineRotationAxisMaxVels))
                      .def("SetAffineRotation3DMaxVels",&PyRobotBase::SetAffineRotation3DMaxVels,args("velocity"), DOXY_FN(RobotBase,SetAffineRotation3DMaxVels))
                      .def("SetAffineRotationQuatMaxVels",&PyRobotBase::SetAffineRotationQuatMaxVels,args("velocity"), DOXY_FN(RobotBase,SetAffineRotationQuatMaxVels))
                      .def("SetAffineTranslationResolution",&PyRobotBase::SetAffineTranslationResolution,args("resolution"), DOXY_FN(RobotBase,SetAffineTranslationResolution))
                      .def("SetAffineRotationAxisResolution",&PyRobotBase::SetAffineRotationAxisResolution,args("resolution"), DOXY_FN(RobotBase,SetAffineRotationAxisResolution))
                      .def("SetAffineRotation3DResolution",&PyRobotBase::SetAffineRotation3DResolution,args("resolution"), DOXY_FN(RobotBase,SetAffineRotation3DResolution))
                      .def("SetAffineRotationQuatResolution",&PyRobotBase::SetAffineRotationQuatResolution,args("resolution"), DOXY_FN(RobotBase,SetAffineRotationQuatResolution))
                      .def("SetAffineTranslationWeights",&PyRobotBase::SetAffineTranslationWeights,args("weights"), DOXY_FN(RobotBase,SetAffineTranslationWeights))
                      .def("SetAffineRotationAxisWeights",&PyRobotBase::SetAffineRotationAxisWeights,args("weights"), DOXY_FN(RobotBase,SetAffineRotationAxisWeights))
                      .def("SetAffineRotation3DWeights",&PyRobotBase::SetAffineRotation3DWeights,args("weights"), DOXY_FN(RobotBase,SetAffineRotation3DWeights))
                      .def("SetAffineRotationQuatWeights",&PyRobotBase::SetAffineRotationQuatWeights,args("weights"), DOXY_FN(RobotBase,SetAffineRotationQuatWeights))
                      .def("GetAffineTranslationLimits",&PyRobotBase::GetAffineTranslationLimits, DOXY_FN(RobotBase,GetAffineTranslationLimits))
                      .def("GetAffineRotationAxisLimits",&PyRobotBase::GetAffineRotationAxisLimits, DOXY_FN(RobotBase,GetAffineRotationAxisLimits))
                      .def("GetAffineRotation3DLimits",&PyRobotBase::GetAffineRotation3DLimits, DOXY_FN(RobotBase,GetAffineRotation3DLimits))
                      .def("GetAffineRotationQuatLimits",&PyRobotBase::GetAffineRotationQuatLimits, DOXY_FN(RobotBase,GetAffineRotationQuatLimits))
                      .def("GetAffineTranslationMaxVels",&PyRobotBase::GetAffineTranslationMaxVels, DOXY_FN(RobotBase,GetAffineTranslationMaxVels))
                      .def("GetAffineRotationAxisMaxVels",&PyRobotBase::GetAffineRotationAxisMaxVels, DOXY_FN(RobotBase,GetAffineRotationAxisMaxVels))
                      .def("GetAffineRotation3DMaxVels",&PyRobotBase::GetAffineRotation3DMaxVels, DOXY_FN(RobotBase,GetAffineRotation3DMaxVels))
                      .def("GetAffineRotationQuatMaxVels",&PyRobotBase::GetAffineRotationQuatMaxVels, DOXY_FN(RobotBase,GetAffineRotationQuatMaxVels))
                      .def("GetAffineTranslationResolution",&PyRobotBase::GetAffineTranslationResolution, DOXY_FN(RobotBase,GetAffineTranslationResolution))
                      .def("GetAffineRotationAxisResolution",&PyRobotBase::GetAffineRotationAxisResolution, DOXY_FN(RobotBase,GetAffineRotationAxisResolution))
                      .def("GetAffineRotation3DResolution",&PyRobotBase::GetAffineRotation3DResolution, DOXY_FN(RobotBase,GetAffineRotation3DResolution))
                      .def("GetAffineRotationQuatResolution",&PyRobotBase::GetAffineRotationQuatResolution, DOXY_FN(RobotBase,GetAffineRotationQuatResolution))
                      .def("GetAffineTranslationWeights",&PyRobotBase::GetAffineTranslationWeights, DOXY_FN(RobotBase,GetAffineTranslationWeights))
                      .def("GetAffineRotationAxisWeights",&PyRobotBase::GetAffineRotationAxisWeights, DOXY_FN(RobotBase,GetAffineRotationAxisWeights))
                      .def("GetAffineRotation3DWeights",&PyRobotBase::GetAffineRotation3DWeights, DOXY_FN(RobotBase,GetAffineRotation3DWeights))
                      .def("GetAffineRotationQuatWeights",&PyRobotBase::GetAffineRotationQuatWeights, DOXY_FN(RobotBase,GetAffineRotationQuatWeights))
                      .def("SetActiveDOFValues",&PyRobotBase::SetActiveDOFValues,SetActiveDOFValues_overloads(args("values","checklimits"), DOXY_FN(RobotBase,SetActiveDOFValues)))
                      .def("GetActiveDOFValues",&PyRobotBase::GetActiveDOFValues, DOXY_FN(RobotBase,GetActiveDOFValues))
                      .def("GetActiveDOFWeights",&PyRobotBase::GetActiveDOFWeights, DOXY_FN(RobotBase,GetActiveDOFWeights))
                      .def("SetActiveDOFVelocities",&PyRobotBase::SetActiveDOFVelocities, SetActiveDOFVelocities_overloads(args("velocities","checklimits"), DOXY_FN(RobotBase,SetActiveDOFVelocities)))
                      .def("GetActiveDOFVelocities",&PyRobotBase::GetActiveDOFVelocities, DOXY_FN(RobotBase,GetActiveDOFVelocities))
                      .def("GetActiveDOFLimits",&PyRobotBase::GetActiveDOFLimits, DOXY_FN(RobotBase,GetActiveDOFLimits))
                      .def("GetActiveDOFMaxVel",&PyRobotBase::GetActiveDOFMaxVel, DOXY_FN(RobotBase,GetActiveDOFMaxVel))
                      .def("GetActiveDOFMaxAccel",&PyRobotBase::GetActiveDOFMaxAccel, DOXY_FN(RobotBase,GetActiveDOFMaxAccel))
                      .def("GetActiveDOFResolutions",&PyRobotBase::GetActiveDOFResolutions, DOXY_FN(RobotBase,GetActiveDOFResolutions))
                      .def("GetActiveConfigurationSpecification",&PyRobotBase::GetActiveConfigurationSpecification, GetActiveConfigurationSpecification_overloads(args("interpolation"),DOXY_FN(RobotBase,GetActiveConfigurationSpecification)))
                      .def("GetActiveJointIndices",&PyRobotBase::GetActiveJointIndices)
                      .def("GetActiveDOFIndices",&PyRobotBase::GetActiveDOFIndices, DOXY_FN(RobotBase,GetActiveDOFIndices))
                      .def("SubtractActiveDOFValues",&PyRobotBase::SubtractActiveDOFValues, args("values0","values1"), DOXY_FN(RobotBase,SubtractActiveDOFValues))
                      .def("CalculateActiveJacobian",&PyRobotBase::CalculateActiveJacobian,args("linkindex","offset"), DOXY_FN(RobotBase,CalculateActiveJacobian "int; const Vector; std::vector"))
                      .def("CalculateActiveRotationJacobian",&PyRobotBase::CalculateActiveRotationJacobian,args("linkindex","quat"), DOXY_FN(RobotBase,CalculateActiveRotationJacobian "int; const Vector; std::vector"))
                      .def("CalculateActiveAngularVelocityJacobian",&PyRobotBase::CalculateActiveAngularVelocityJacobian,args("linkindex"), DOXY_FN(RobotBase,CalculateActiveAngularVelocityJacobian "int; std::vector"))
                      .def("Grab",pgrab1,args("body"), DOXY_FN(RobotBase,Grab "KinBodyPtr"))
                      .def("Grab",pgrab2,args("body","grablink"), DOXY_FN(RobotBase,Grab "KinBodyPtr; LinkPtr"))
                      .def("Grab",pgrab4,args("body","grablink","linkstoignore"), DOXY_FN(RobotBase,Grab "KinBodyPtr; LinkPtr; const std::set"))
                      .def("Release",&PyRobotBase::Release,args("body"), DOXY_FN(RobotBase,Release))
                      .def("ReleaseAllGrabbed",&PyRobotBase::ReleaseAllGrabbed, DOXY_FN(RobotBase,ReleaseAllGrabbed))
                      .def("RegrabAll",&PyRobotBase::RegrabAll, DOXY_FN(RobotBase,RegrabAll))
                      .def("IsGrabbing",&PyRobotBase::IsGrabbing,args("body"), DOXY_FN(RobotBase,IsGrabbing))
                      .def("GetGrabbed",&PyRobotBase::GetGrabbed, DOXY_FN(RobotBase,GetGrabbed))
                      .def("GetGrabbedInfo",&PyRobotBase::GetGrabbedInfo, DOXY_FN(RobotBase,GetGrabbedInfo))
                      .def("ResetGrabbed",&PyRobotBase::ResetGrabbed, args("grabbedinfos"), DOXY_FN(RobotBase,ResetGrabbed))
                      .def("WaitForController",&PyRobotBase::WaitForController,args("timeout"), "Wait until the robot controller is done")
                      .def("GetRobotStructureHash",&PyRobotBase::GetRobotStructureHash, DOXY_FN(RobotBase,GetRobotStructureHash))
                      .def("CreateRobotStateSaver",&PyRobotBase::CreateRobotStateSaver, CreateRobotStateSaver_overloads(args("options"), "Creates an object that can be entered using 'with' and returns a RobotStateSaver")[return_value_policy<manage_new_object>()])
                      .def("__repr__", &PyRobotBase::__repr__)
                      .def("__str__", &PyRobotBase::__str__)
                      .def("__unicode__", &PyRobotBase::__unicode__)
        ;
        robot.attr("DOFAffine") = dofaffine; // deprecated (11/10/04)
        robot.attr("ManipulatorInfo") = manipulatorinfo;
        robot.attr("AttachedSensorInfo") = attachedsensorinfo;
        robot.attr("GrabbedInfo") = grabbedinfo;

        object (PyRobotBase::PyManipulator::*pmanipik)(object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipikf)(object, object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipiks)(object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolutions;
        object (PyRobotBase::PyManipulator::*pmanipiksf)(object, object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolutions;

        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision0)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision1)(object,PyCollisionReportPtr,int) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision1)() const = &PyRobotBase::PyManipulator::CheckIndependentCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision2)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckIndependentCollision;

        std::string GetIkParameterization_doc = std::string(DOXY_FN(RobotBase::Manipulator,GetIkParameterization "const IkParameterization; bool")) + std::string(DOXY_FN(RobotBase::Manipulator,GetIkParameterization "IkParameterizationType; bool"));
        class_<PyRobotBase::PyManipulator, boost::shared_ptr<PyRobotBase::PyManipulator> >("Manipulator", DOXY_CLASS(RobotBase::Manipulator), no_init)
        .def("GetEndEffectorTransform", &PyRobotBase::PyManipulator::GetTransform, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetTransform", &PyRobotBase::PyManipulator::GetTransform, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetTransformPose", &PyRobotBase::PyManipulator::GetTransformPose, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetVelocity", &PyRobotBase::PyManipulator::GetVelocity, DOXY_FN(RobotBase::Manipulator,GetVelocity))
        .def("GetName",&PyRobotBase::PyManipulator::GetName, DOXY_FN(RobotBase::Manipulator,GetName))
        .def("SetName",&PyRobotBase::PyManipulator::SetName, args("name"), DOXY_FN(RobotBase::Manipulator,SetName))
        .def("GetRobot",&PyRobotBase::PyManipulator::GetRobot, DOXY_FN(RobotBase::Manipulator,GetRobot))
        .def("SetIkSolver",&PyRobotBase::PyManipulator::SetIkSolver, DOXY_FN(RobotBase::Manipulator,SetIkSolver))
        .def("GetIkSolver",&PyRobotBase::PyManipulator::GetIkSolver, DOXY_FN(RobotBase::Manipulator,GetIkSolver))
        .def("SetIKSolver",&PyRobotBase::PyManipulator::SetIkSolver, DOXY_FN(RobotBase::Manipulator,SetIkSolver))
        .def("GetNumFreeParameters",&PyRobotBase::PyManipulator::GetNumFreeParameters, DOXY_FN(RobotBase::Manipulator,GetNumFreeParameters))
        .def("GetFreeParameters",&PyRobotBase::PyManipulator::GetFreeParameters, DOXY_FN(RobotBase::Manipulator,GetFreeParameters))
        .def("FindIKSolution",pmanipik,FindIKSolution_overloads(args("param","filteroptions","ikreturn","releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; std::vector; int")))
        .def("FindIKSolution",pmanipikf,FindIKSolutionFree_overloads(args("param","freevalues","filteroptions","ikreturn","releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; const std::vector; std::vector; int")))
        .def("FindIKSolutions",pmanipiks,FindIKSolutions_overloads(args("param","filteroptions","ikreturn","releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; std::vector; int")))
        .def("FindIKSolutions",pmanipiksf,FindIKSolutionsFree_overloads(args("param","freevalues","filteroptions","ikreturn","releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; const std::vector; std::vector; int")))
        .def("GetIkParameterization",&PyRobotBase::PyManipulator::GetIkParameterization, GetIkParameterization_overloads(args("iktype","inworld"), GetIkParameterization_doc.c_str()))
        .def("GetBase",&PyRobotBase::PyManipulator::GetBase, DOXY_FN(RobotBase::Manipulator,GetBase))
        .def("GetEndEffector",&PyRobotBase::PyManipulator::GetEndEffector, DOXY_FN(RobotBase::Manipulator,GetEndEffector))
        .def("GetGraspTransform",&PyRobotBase::PyManipulator::GetGraspTransform, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransform))
        .def("GetLocalToolTransform",&PyRobotBase::PyManipulator::GetLocalToolTransform, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransform))
        .def("GetLocalToolTransformPose",&PyRobotBase::PyManipulator::GetLocalToolTransformPose, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransformPose))
        .def("SetLocalToolTransform",&PyRobotBase::PyManipulator::SetLocalToolTransform, args("transform"), DOXY_FN(RobotBase::Manipulator,SetLocalToolTransform))
        .def("SetLocalToolDirection",&PyRobotBase::PyManipulator::SetLocalToolDirection, args("direction"), DOXY_FN(RobotBase::Manipulator,SetLocalToolDirection))
        .def("SetClosingDirection",&PyRobotBase::PyManipulator::SetClosingDirection, args("closingdirection"), DOXY_FN(RobotBase::Manipulator,SetClosingDirection))
        .def("SetChuckingDirection",&PyRobotBase::PyManipulator::SetChuckingDirection, args("chuckingdirection"), DOXY_FN(RobotBase::Manipulator,SetChuckingDirection))
        .def("GetGripperJoints",&PyRobotBase::PyManipulator::GetGripperJoints, DOXY_FN(RobotBase::Manipulator,GetGripperIndices))
        .def("GetGripperIndices",&PyRobotBase::PyManipulator::GetGripperIndices, DOXY_FN(RobotBase::Manipulator,GetGripperIndices))
        .def("GetArmJoints",&PyRobotBase::PyManipulator::GetArmJoints, DOXY_FN(RobotBase::Manipulator,GetArmIndices))
        .def("GetArmIndices",&PyRobotBase::PyManipulator::GetArmIndices, DOXY_FN(RobotBase::Manipulator,GetArmIndices))
        .def("GetArmDOFValues",&PyRobotBase::PyManipulator::GetArmDOFValues, DOXY_FN(RobotBase::Manipulator,GetArmDOFValues))
        .def("GetGripperDOFValues",&PyRobotBase::PyManipulator::GetGripperDOFValues, DOXY_FN(RobotBase::Manipulator,GetGripperDOFValues))
        .def("GetArmDOF",&PyRobotBase::PyManipulator::GetArmDOF, DOXY_FN(RobotBase::Manipulator,GetArmDOF))
        .def("GetGripperDOF",&PyRobotBase::PyManipulator::GetGripperDOF, DOXY_FN(RobotBase::Manipulator,GetGripperDOF))
        .def("GetClosingDirection",&PyRobotBase::PyManipulator::GetClosingDirection, DOXY_FN(RobotBase::Manipulator,GetClosingDirection))
        .def("GetChuckingDirection",&PyRobotBase::PyManipulator::GetChuckingDirection, DOXY_FN(RobotBase::Manipulator,GetChuckingDirection))
        .def("GetPalmDirection",&PyRobotBase::PyManipulator::GetPalmDirection, DOXY_FN(RobotBase::Manipulator,GetLocalToolDirection))
        .def("GetDirection",&PyRobotBase::PyManipulator::GetDirection, DOXY_FN(RobotBase::Manipulator,GetLocalToolDirection))
        .def("GetLocalToolDirection",&PyRobotBase::PyManipulator::GetLocalToolDirection, DOXY_FN(RobotBase::Manipulator,GetLocalToolDirection))
        .def("IsGrabbing",&PyRobotBase::PyManipulator::IsGrabbing,args("body"), DOXY_FN(RobotBase::Manipulator,IsGrabbing))
        .def("GetChildJoints",&PyRobotBase::PyManipulator::GetChildJoints, DOXY_FN(RobotBase::Manipulator,GetChildJoints))
        .def("GetChildDOFIndices",&PyRobotBase::PyManipulator::GetChildDOFIndices, DOXY_FN(RobotBase::Manipulator,GetChildDOFIndices))
        .def("GetChildLinks",&PyRobotBase::PyManipulator::GetChildLinks, DOXY_FN(RobotBase::Manipulator,GetChildLinks))
        .def("IsChildLink",&PyRobotBase::PyManipulator::IsChildLink, DOXY_FN(RobotBase::Manipulator,IsChildLink))
        .def("GetIndependentLinks",&PyRobotBase::PyManipulator::GetIndependentLinks, DOXY_FN(RobotBase::Manipulator,GetIndependentLinks))
        .def("GetArmConfigurationSpecification",&PyRobotBase::PyManipulator::GetArmConfigurationSpecification, GetArmConfigurationSpecification_overloads(args("interpolation"),DOXY_FN(RobotBase::Manipulator,GetArmConfigurationSpecification)))
        .def("GetIkConfigurationSpecification",&PyRobotBase::PyManipulator::GetIkConfigurationSpecification, GetIkConfigurationSpecification_overloads(args("iktype", "interpolation"),DOXY_FN(RobotBase::Manipulator,GetIkConfigurationSpecification)))
        .def("CheckEndEffectorCollision",pCheckEndEffectorCollision1,CheckEndEffectorCollision_overloads(args("transform", "report", "numredundantsamples"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorCollision)))
        .def("CheckEndEffectorCollision",pCheckEndEffectorCollision0,args("report"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorCollision))
        .def("CheckIndependentCollision",pCheckIndependentCollision1, DOXY_FN(RobotBase::Manipulator,CheckIndependentCollision))
        .def("CheckIndependentCollision",pCheckIndependentCollision2,args("report"), DOXY_FN(RobotBase::Manipulator,CheckIndependentCollision))
        .def("CalculateJacobian",&PyRobotBase::PyManipulator::CalculateJacobian,DOXY_FN(RobotBase::Manipulator,CalculateJacobian))
        .def("CalculateRotationJacobian",&PyRobotBase::PyManipulator::CalculateRotationJacobian,DOXY_FN(RobotBase::Manipulator,CalculateRotationJacobian))
        .def("CalculateAngularVelocityJacobian",&PyRobotBase::PyManipulator::CalculateAngularVelocityJacobian,DOXY_FN(RobotBase::Manipulator,CalculateAngularVelocityJacobian))
        .def("GetStructureHash",&PyRobotBase::PyManipulator::GetStructureHash, DOXY_FN(RobotBase::Manipulator,GetStructureHash))
        .def("GetKinematicsStructureHash",&PyRobotBase::PyManipulator::GetKinematicsStructureHash, DOXY_FN(RobotBase::Manipulator,GetKinematicsStructureHash))
        .def("GetInverseKinematicsStructureHash",&PyRobotBase::PyManipulator::GetInverseKinematicsStructureHash, args("iktype"), DOXY_FN(RobotBase::Manipulator,GetInverseKinematicsStructureHash))
        .def("GetInfo",&PyRobotBase::PyManipulator::GetInfo, DOXY_FN(RobotBase::Manipulator,GetInfo))
        .def("__repr__",&PyRobotBase::PyManipulator::__repr__)
        .def("__str__",&PyRobotBase::PyManipulator::__str__)
        .def("__unicode__",&PyRobotBase::PyManipulator::__unicode__)
        .def("__eq__",&PyRobotBase::PyManipulator::__eq__)
        .def("__ne__",&PyRobotBase::PyManipulator::__ne__)
        .def("__hash__",&PyRobotBase::PyManipulator::__hash__)
        ;

        class_<PyRobotBase::PyAttachedSensor, boost::shared_ptr<PyRobotBase::PyAttachedSensor> >("AttachedSensor", DOXY_CLASS(RobotBase::AttachedSensor), no_init)
        .def("GetSensor",&PyRobotBase::PyAttachedSensor::GetSensor, DOXY_FN(RobotBase::AttachedSensor,GetSensor))
        .def("GetAttachingLink",&PyRobotBase::PyAttachedSensor::GetAttachingLink, DOXY_FN(RobotBase::AttachedSensor,GetAttachingLink))
        .def("GetRelativeTransform",&PyRobotBase::PyAttachedSensor::GetRelativeTransform, DOXY_FN(RobotBase::AttachedSensor,GetRelativeTransform))
        .def("GetTransform",&PyRobotBase::PyAttachedSensor::GetTransform, DOXY_FN(RobotBase::AttachedSensor,GetTransform))
        .def("GetTransformPose",&PyRobotBase::PyAttachedSensor::GetTransformPose, DOXY_FN(RobotBase::AttachedSensor,GetTransform))
        .def("GetRobot",&PyRobotBase::PyAttachedSensor::GetRobot, DOXY_FN(RobotBase::AttachedSensor,GetRobot))
        .def("GetName",&PyRobotBase::PyAttachedSensor::GetName, DOXY_FN(RobotBase::AttachedSensor,GetName))
        .def("GetData",&PyRobotBase::PyAttachedSensor::GetData, DOXY_FN(RobotBase::AttachedSensor,GetData))
        .def("SetRelativeTransform",&PyRobotBase::PyAttachedSensor::SetRelativeTransform,args("transform"), DOXY_FN(RobotBase::AttachedSensor,SetRelativeTransform))
        .def("GetStructureHash",&PyRobotBase::PyAttachedSensor::GetStructureHash, DOXY_FN(RobotBase::AttachedSensor,GetStructureHash))
        .def("UpdateInfo",&PyRobotBase::PyAttachedSensor::UpdateInfo, UpdateInfo_overloads(args("type"), DOXY_FN(RobotBase::AttachedSensor,UpdateInfo)))
        .def("GetInfo",&PyRobotBase::PyAttachedSensor::GetInfo, DOXY_FN(RobotBase::AttachedSensor,GetInfo))
        .def("UpdateAndGetInfo",&PyRobotBase::PyAttachedSensor::UpdateAndGetInfo, UpdateAndGetInfo_overloads(DOXY_FN(RobotBase::AttachedSensor,UpdateAndGetInfo)))
        .def("__str__",&PyRobotBase::PyAttachedSensor::__str__)
        .def("__repr__",&PyRobotBase::PyAttachedSensor::__repr__)
        .def("__unicode__",&PyRobotBase::PyAttachedSensor::__unicode__)
        .def("__eq__",&PyRobotBase::PyAttachedSensor::__eq__)
        .def("__ne__",&PyRobotBase::PyAttachedSensor::__ne__)
        .def("__hash__",&PyRobotBase::PyAttachedSensor::__hash__)
        ;

        class_<PyRobotBase::PyRobotStateSaver, boost::shared_ptr<PyRobotBase::PyRobotStateSaver> >("RobotStateSaver", DOXY_CLASS(Robot::RobotStateSaver), no_init)
        .def(init<PyRobotBasePtr>(args("robot")))
        .def(init<PyRobotBasePtr,object>(args("robot","options")))
        .def("GetBody",&PyRobotBase::PyRobotStateSaver::GetBody,DOXY_FN(Robot::RobotStateSaver, GetBody))
        .def("Restore",&PyRobotBase::PyRobotStateSaver::Restore,Restore_overloads(args("body"), DOXY_FN(Robot::RobotStateSaver, Restore)))
        .def("Release",&PyRobotBase::PyRobotStateSaver::Release,DOXY_FN(Robot::RobotStateSaver, Release))
        .def("__str__",&PyRobotBase::PyRobotStateSaver::__str__)
        .def("__unicode__",&PyRobotBase::PyRobotStateSaver::__unicode__)
        ;
    }

    def("RaveCreateRobot",openravepy::RaveCreateRobot,args("env","name"),DOXY_FN1(RaveCreateRobot));
}

}
