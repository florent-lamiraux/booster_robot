from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import newProblem, ProblemSolver, ConstraintGraph, Constraints, \
    ConstraintGraphFactory
from hpp.corbaserver.manipulation.robot import HumanoidRobot as Robot
from hpp.gepetto import displayGripper
from hpp.gepetto.manipulation import ViewerFactory

class Box:
    urdfFilename = "package://booster_robot/urdf/box.urdf"
    srdfFilename = "package://booster_robot/srdf/box.srdf"
    rootJointType = "freeflyer"

loadServerPlugin ("corbaserver", "manipulation-corba.so")
newProblem()

Robot.urdfFilename = "package://booster_robot/urdf/T1_Serial.urdf"
Robot.srdfFilename = "package://booster_robot/srdf/T1_Serial.srdf"

robot = Robot("booster", "T1", rootJointType="freeflyer")
robot.leftAnkle = "T1/Left_Ankle_Roll"
robot.rightAnkle = "T1/Right_Ankle_Roll"
robot.setJointBounds('T1/root_joint', [-1., 1., -1., 1., -1., 1., -1., 1., -1., 1., -1., 1.,
                                       -1., 1.,])

# Create initial configuration
q0 = robot.getCurrentConfig()
q0[2] = 0.6658526350768468
r = robot.rankInConfiguration['T1/Left_Hip_Pitch']
q0[r] = -0.2
r = robot.rankInConfiguration['T1/Left_Knee_Pitch']
q0[r] = 0.4
r = robot.rankInConfiguration['T1/Left_Ankle_Pitch']
q0[r] = -0.2
r = robot.rankInConfiguration['T1/Right_Hip_Pitch']
q0[r] = -0.2
r = robot.rankInConfiguration['T1/Right_Knee_Pitch']
q0[r] = 0.4
r = robot.rankInConfiguration['T1/Right_Ankle_Pitch']
q0[r] = -0.2

ps = ProblemSolver(robot)
vf = ViewerFactory(ps)
vf.loadRobotModel(Box, "box")
robot.setJointBounds("box/root_joint", [-1., 1., -1., 1.,-1., 1.])

# Create initial configuration
q0 = robot.getCurrentConfig()
q0[2] = 0.6658526350768468
r = robot.rankInConfiguration['T1/Left_Hip_Pitch']
q0[r] = -0.2
r = robot.rankInConfiguration['T1/Left_Knee_Pitch']
q0[r] = 0.4
r = robot.rankInConfiguration['T1/Left_Ankle_Pitch']
q0[r] = -0.2
r = robot.rankInConfiguration['T1/Right_Hip_Pitch']
q0[r] = -0.2
r = robot.rankInConfiguration['T1/Right_Knee_Pitch']
q0[r] = 0.4
r = robot.rankInConfiguration['T1/Right_Ankle_Pitch']
q0[r] = -0.2
r = robot.rankInConfiguration['box/root_joint']
q0[r:r+3] = [.4, 0, .8]

ps.addPartialCom ("T1", ["T1/root_joint"])

robot.createStaticStabilityConstraint ('balance/', 'T1', robot.leftAnkle,
                                       robot.rightAnkle, q0)
balanceConstraints = ['balance/pose-left-foot',
                      'balance/pose-right-foot',
                      'balance/relative-com',]

# Build the constraint graph
cg = ConstraintGraph(robot, 'graph')
factory = ConstraintGraphFactory(cg)
factory.setGrippers(['T1/left_gripper', 'T1/right_gripper'])
factory.setObjects(["box"], [['box/handle1', 'box/handle2']], [[]])
factory.generate()
cg.addConstraints(graph=True, constraints = Constraints(numConstraints = balanceConstraints))
cg.initialize()

e = "T1/left_gripper > box/handle2 | f_01"
res, q1, err = cg.generateTargetConfig(e, q0, q0)

ps.addPathOptimizer('SimpleTimeParameterization')
ps.setParameter('SimpleTimeParameterization/maxAcceleration', .5)
ps.setParameter('SimpleTimeParameterization/order', 2)
ps.setParameter('SimpleTimeParameterization/safety', .95)

