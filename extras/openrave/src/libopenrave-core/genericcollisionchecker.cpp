// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "ravep.h"

namespace OpenRAVE {

class GenericCollisionChecker : public CollisionCheckerBase
{
public:
    GenericCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput) : CollisionCheckerBase(penv) {
    }
    virtual ~GenericCollisionChecker() {
    }

    void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        CollisionCheckerBase::Clone(preference, cloningoptions);
        boost::shared_ptr<GenericCollisionChecker const > r = boost::dynamic_pointer_cast<GenericCollisionChecker const>(preference);
        _geometrygroup = r->_geometrygroup;
    }

    virtual bool InitEnvironment() {
        return true;
    }
    virtual void DestroyEnvironment() {
    }

    virtual bool InitKinBody(KinBodyPtr pbody) {
        return true;
    }
    virtual void RemoveKinBody(KinBodyPtr pbody) {
    }
    virtual bool Enable(KinBodyConstPtr pbody, bool bEnable) {
        return true;
    }
    virtual bool EnableLink(KinBody::LinkConstPtr pbody, bool bEnable) {
        return true;
    }

    virtual bool SetCollisionOptions(int collisionoptions) {
        return true;
    }
    virtual int GetCollisionOptions() const {
        return 0;
    }

    virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput) {
        return true;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr) {
        return false;
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr) {
        return false;
    }

    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckStandaloneSelfCollision(KinBody::LinkConstPtr pbody, CollisionReportPtr) {
        return false;
    }

    virtual void SetTolerance(dReal tolerance) {
    }

    virtual void SetGeometryGroup(const std::string& groupname)
    {
        _geometrygroup = groupname;
    }

    virtual const std::string& GetGeometryGroup() const
    {
        return _geometrygroup;
    }

    virtual void SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname) {
    }

    virtual const std::string& GetBodyGeometryGroup(KinBodyConstPtr pbody) const {
        return _geometrygroup;
    }

    std::string _geometrygroup;
};

CollisionCheckerBasePtr CreateGenericCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput)
{
    return CollisionCheckerBasePtr(new GenericCollisionChecker(penv,sinput));
}

}
