# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2011 Rosen Diankov <rosen.diankov@gmail.com>'
__license__ = 'Apache License, Version 2.0'
# python 2.5 raises 'import *' not allowed with 'from .'
from ..openravepy_int import RaveCreateModule, RaveCreateTrajectory, matrixSerialization, matrixFromPose
from .. import PlanningError

from numpy import *
from copy import copy as shallowcopy

import logging
log = logging.getLogger('openravepy.interfaces.Grasper')

class Grasper:
    """Interface wrapper for :ref:`module-grasper`
    """
    def __init__(self,robot,friction=0.3,avoidlinks=[],plannername=None):
        env = robot.GetEnv()
        self.prob = RaveCreateModule(env,'Grasper')
        self.robot = robot
        self.friction = friction
        self.avoidlinks = avoidlinks
        if self.avoidlinks is None:
            self.avoidlinks = []
        self.plannername=plannername
        self.args = self.robot.GetName()
        if plannername is not None and len(plannername)>0:
            self.args += ' planner %s '%plannername
        env.Add(self.prob,True,self.args)
    def  __del__(self):
        # need to lock the environment since Remove locks it
        env = self.prob.GetEnv()
        if env.Lock(1.0):
            try:
                env.Remove(self.prob)
            finally:
                env.Unlock()
        else:
            log.warn('failed to lock environment for Grasper.__del__!')
            
    def clone(self,envother):
        """Clones the interface into another environment
        """
        clone = shallowcopy(self)
        clone.prob = RaveCreateModule(envother,'Grasper')
        clone.robot = envother.GetRobot(self.robot.GetName())
        clone.avoidlinks = [clone.robot.GetLink(link.GetName()) for link in self.avoidlinks]
        envother.Add(clone.prob,True,clone.args)
        return clone
    def Grasp(self,direction=None,roll=None,position=None,standoff=None,target=None,stablecontacts=False,forceclosure=False,transformrobot=True,onlycontacttarget=True,tightgrasp=False,graspingnoise=None,execute=None,translationstepmult=None,outputfinal=False,manipulatordirection=None,finestep=None,vintersectplane=None,chuckingdirection=None):
        """See :ref:`module-grasper-grasp`
        """
        cmd = 'Grasp '
        if direction is not None:
            cmd += 'direction %.15e %.15e %.15e '%(direction[0],direction[1],direction[2])
        if transformrobot:
            cmd += 'roll %.15e position %.15e %.15e %.15e manipulatordirection %.15e %.15e %.15e '%(roll,position[0],position[1],position[2],manipulatordirection[0],manipulatordirection[1],manipulatordirection[2])
        if standoff is not None:
            cmd += 'standoff %.15e '%standoff
        if target is not None:
            cmd += 'target %s '%target.GetName()
        cmd += 'stablecontacts %d forceclosure %d transformrobot %d onlycontacttarget %d tightgrasp %d outputfinal %d '%(stablecontacts,forceclosure,transformrobot,onlycontacttarget,tightgrasp,outputfinal)
        if self.friction is not None:
            cmd += 'friction %.15e '%self.friction
        for link in self.avoidlinks:
            cmd += 'avoidlink %s '%link.GetName()
        if graspingnoise is not None:
            cmd += 'graspingnoise %.15e '%graspingnoise
        if translationstepmult is not None:
            cmd += 'translationstepmult %.15e '%translationstepmult
        if finestep is not None:
            cmd += 'finestep %.15e '%finestep
        if vintersectplane is not None:
            cmd += 'vintersectplane %.15e %.15e %.15e %.15e '%(vintersectplane[0], vintersectplane[1], vintersectplane[2], vintersectplane[3])
        if chuckingdirection is not None:
            cmd += 'chuckingdirection '
            for value in chuckingdirection:
                cmd += '%.15e '%value
        if execute is not None:
            cmd += 'execute %d '%execute
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise PlanningError('Grasp failed')
        resvalues = res.split()
        mindist = None
        volume = None
        contacts = None
        finalconfig = None
        if forceclosure:
            volume = float64(resvalues.pop())
            mindist = float64(resvalues.pop())
        if outputfinal:
            jointvalues = []
            for i in range(self.robot.GetDOF()):
                jointvalues.insert(0,float64(resvalues.pop()))
            pose = []
            for i in range(7):
                pose.insert(0,float64(resvalues.pop()))
            finalconfig = (array(jointvalues),matrixFromPose(pose))
        contacts = reshape(array([float64(s) for s in resvalues],float64),(len(resvalues)/6,6))
        return contacts,finalconfig,mindist,volume

    def GraspThreaded(self,approachrays,standoffs,preshapes,rolls,manipulatordirections=None,target=None,transformrobot=True,onlycontacttarget=True,tightgrasp=False,graspingnoise=None,forceclosurethreshold=None,collisionchecker=None,translationstepmult=None,numthreads=None,startindex=None,maxgrasps=None,finestep=None):
        """See :ref:`module-grasper-graspthreaded`
        """
        cmd = 'GraspThreaded '
        if target is not None:
            cmd += 'target %s '%target.GetName()
        cmd += 'forceclosure %d %g onlycontacttarget %d tightgrasp %d '%(forceclosurethreshold is not None,forceclosurethreshold,onlycontacttarget,tightgrasp)
        if self.friction is not None:
            cmd += 'friction %.15e '%self.friction
        if startindex is not None:
            cmd += 'startindex %d '%startindex
        if maxgrasps is not None:
            cmd += 'maxgrasps %d '%maxgrasps
        for link in self.avoidlinks:
            cmd += 'avoidlink %s '%link.GetName()
        if graspingnoise is not None:
            cmd += 'graspingnoise %.15e %d '%graspingnoise
        if translationstepmult is not None:
            cmd += 'translationstepmult %.15e '%translationstepmult
        if finestep is not None:
            cmd += 'finestep %.15e '%finestep
        if numthreads is not None:
            cmd += 'numthreads %d '%numthreads
        cmd += 'approachrays %d '%len(approachrays)
        for f in approachrays.flat:
            cmd += str(f) + ' '
        cmd += 'rolls %d '%len(rolls)
        for f in rolls.flat:
            cmd += str(f) + ' '
        cmd += 'standoffs %d '%len(standoffs)
        for f in standoffs.flat:
            cmd += str(f) + ' '
        cmd += 'preshapes %d '%len(preshapes)
        for f in preshapes.flat:
            cmd += str(f) + ' '
        cmd += 'manipulatordirections %d '%len(manipulatordirections)
        for f in manipulatordirections.flat:
            cmd += str(f) + ' '
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise PlanningError('Grasp failed')
        resultgrasps = res.split()
        resvalues=[]
        nextid = int(resultgrasps.pop(0))
        preshapelen = len(self.robot.GetActiveManipulator().GetGripperIndices())
        for i in range(int(resultgrasps.pop(0))):
            position = array([float64(resultgrasps.pop(0)) for i in range(3)])
            direction = array([float64(resultgrasps.pop(0)) for i in range(3)])
            roll = float64(resultgrasps.pop(0))
            standoff = float64(resultgrasps.pop(0))
            manipulatordirection = array([float64(resultgrasps.pop(0)) for i in range(3)])
            mindist = float64(resultgrasps.pop(0))
            volume = float64(resultgrasps.pop(0))
            preshape = [float64(resultgrasps.pop(0)) for i in range(preshapelen)]
            Tfinal = matrixFromPose([float64(resultgrasps.pop(0)) for i in range(7)])
            finalshape = array([float64(resultgrasps.pop(0)) for i in range(self.robot.GetDOF())])
            contacts_num=int(resultgrasps.pop(0))
            contacts=[float64(resultgrasps.pop(0)) for i in range(contacts_num*6)]
            contacts = reshape(contacts,(contacts_num,6))
            resvalues.append([position, direction, roll, standoff, manipulatordirection, mindist, volume, preshape,Tfinal,finalshape,contacts])
        return nextid, resvalues

    def ConvexHull(self,points,returnplanes=True,returnfaces=True,returntriangles=True):
        """See :ref:`module-grasper-convexhull`
        """
        dim = len(points[0])
        cmd = 'ConvexHull points %d %d '%(len(points),dim) + ' '.join(str(f) for f in points.flat) + ' '
        if returnplanes is not None:
            cmd += 'returnplanes %d '%returnplanes
        if returnfaces is not None:
            cmd += 'returnfaces %d '%returnfaces
        if returntriangles is not None:
            cmd += 'returntriangles %d '%returntriangles
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise PlanningError('ConvexHull')
        resvalues = res.split()
        planes = None
        faces = None
        triangles = None
        if returnplanes:
            numplanes = int(resvalues.pop(0))
            planes = reshape(array([float64(resvalues.pop(0)) for i in range((dim+1)*numplanes)],float64),(numplanes,dim+1))
        if returnfaces:
            numfaces = int(resvalues.pop(0))
            faces = []
            for i in range(numfaces):
                numvertices = int(resvalues.pop(0))
                faces.append(array([int(resvalues.pop(0)) for i in range(numvertices)],int))
        if returntriangles:
            numtriangles = int(resvalues.pop(0))
            triangles = reshape(array([int(resvalues.pop(0)) for i in range(3*numtriangles)],int),(numtriangles,3))
        return planes,faces,triangles
