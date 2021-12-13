from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
from pinocchio import SE3, Model, Inertia, JointModelFreeFlyer, \
    JointModelRX, JointModelRY, JointModelRZ, \
    JointModelPX, JointModelPY, JointModelPZ, \
    forwardKinematics, neutral
import gepetto.corbaserver
from display import Display

class Visual:
    '''
    Class representing one 3D mesh of the robot, to be attached to a joint. The class contains:
    * the name of the 3D objects inside Gepetto viewer.
    * the ID of the joint in the kinematic tree to which the body is attached.
    * the placement of the body with respect to the joint frame.
    This class is only used in the list Robot.visuals (see below).
    '''
    def __init__(self,name,jointParent,placement):
        self.name = name                  # Name in gepetto viewer
        self.jointParent = jointParent    # ID (int) of the joint
        self.placement = placement        # placement of the body wrt joint, i.e. bodyMjoint
    def place(self,display,oMjoint):
        oMbody = oMjoint*self.placement
        display.place(self.name,oMbody,False)

class Robot:
    '''
    Define a class Robot representing a biped robot
    The members of the class are:
    * viewer: a display encapsulating a gepetto viewer client to create 3D
      objects and place them.
    * model: the kinematic tree of the robot.
    * data: the temporary variables to be used by the kinematic algorithms.
    * visuals: the list of all the 'visual' 3D objects to render the robot,
      each element of the list being an object Visual (see above).
    '''

    def __init__(self):
        self.viewer = Display()
        self.visuals = []
        self.model = Model ()
        self.createLeggedRobot ()
        self.data = self.model.createData()
        self.q0 = neutral (self.model)

    def createLeggedRobot(self, rootId=0, prefix=''):
        color = [1,1,0.78,1.0]
        colorred = [1.0,0.0,0.0,1.0]

        root2Id = self.model.addJoint(rootId, JointModelPX(), SE3.Identity(), prefix + "waist_x")
        root2Id = self.model.addJoint(root2Id, JointModelPY(), SE3.Identity(), prefix + "waist_xy")
        root2Id = self.model.addJoint(root2Id, JointModelPZ(), SE3.Identity(), prefix + "waist_xyz")
        self.waistJointId = root2Id

        self.viewer.viewer.gui.addBox('world/' + prefix + 'hips', .6, .1, .2, color)
        self.visuals.append(Visual('world/' + prefix + 'hips', root2Id, SE3(eye(3), np.array([0., 0., 0.]))))

        for side in ["left", "right"]:
            jointName = prefix + side + "hip_joint"
            x = .3 if side == "right" else -.3
            jointPlacement = SE3(eye(3),np.array([x, 0., -.2]))
            joint = JointModelRX()
            jointId = self.model.addJoint(root2Id, joint, jointPlacement, jointName + "x")
            joint = JointModelRY()
            jointId = self.model.addJoint(jointId, joint, SE3.Identity(), jointName + "y")
            joint = JointModelRZ()
            jointId = self.model.addJoint(jointId, joint, SE3.Identity(), jointName + "z")
            self.model.appendBodyToJoint(jointId, Inertia.Random(), SE3.Identity())
            self.viewer.viewer.gui.addSphere('world/' + prefix + side + 'hip', 0.1, colorred)
            self.visuals.append(Visual('world/' + prefix + side + 'hip', jointId, SE3.Identity()) )
            self.viewer.viewer.gui.addBox('world/' + prefix + side + 'upperleg', .1, .1, .8, color)
            self.visuals.append(Visual('world/' + prefix + side + 'upperleg', jointId, SE3(eye(3), np.array([0., 0., -0.5]))))

            jointName = prefix + side + "knee_joint"
            jointPlacement = SE3(eye(3),np.array([0, 0, -1]))
            joint = JointModelRX()
            jointId = self.model.addJoint(jointId, joint, jointPlacement, jointName)
            self.model.appendBodyToJoint(jointId, Inertia.Random(), SE3.Identity())
            self.viewer.viewer.gui.addSphere('world/' + prefix + side + 'knee', 0.1, colorred)
            self.visuals.append(Visual('world/' + prefix + side + 'knee', jointId, SE3.Identity()) )
            self.viewer.viewer.gui.addBox('world/' + prefix + side + 'lowerleg', .1, .1, .8, color)
            self.visuals.append(Visual('world/' + prefix + side + 'lowerleg', jointId, SE3(eye(3), np.array([0., 0., -0.5]))))

            jointName = prefix + side + "ankle_joint"
            jointPlacement = SE3(eye(3),np.array([0, 0, -1]))
            joint = JointModelRX()
            jointId = self.model.addJoint(jointId, joint, jointPlacement, jointName + "x")
            joint = JointModelRY()
            jointId = self.model.addJoint(jointId, joint, SE3.Identity(), jointName + "y")
            self.model.appendBodyToJoint(jointId, Inertia.Random(), SE3.Identity())
            self.viewer.viewer.gui.addSphere('world/' + prefix + side + 'ankle', 0.1, colorred)
            self.visuals.append(Visual('world/' + prefix + side + 'ankle', jointId, SE3.Identity()) )
            self.viewer.viewer.gui.addBox('world/' + prefix + side + 'foot', .2, .4, .1, color)
            self.visuals.append(Visual('world/' + prefix + side + 'foot', jointId, SE3(eye(3), np.array([0., -.1, -0.15]))))
            
            if side == 'left':
                self.leftFootJointId = jointId
            else:
                self.rightFootJointId = jointId

    def display(self,q):
        forwardKinematics(self.model,self.data,q)
        for visual in self.visuals:
            visual.place( self.viewer,self.data.oMi[visual.jointParent] )
        self.viewer.viewer.gui.refresh()
