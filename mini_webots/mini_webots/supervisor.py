

# from math import sqrt
# from controller import Supervisor

# TIME_STEP = 32

# supervisor = Supervisor()

# # get handle to robot's translation field
# robot_node = supervisor.getFromDef("minaros")
# trans_field = robot_node.getField("translation")

# # for a in range(0, 25):
# #     for b in range(0, 33):
# #         # evaluate robot during 60 seconds (simulation time)
# #         t = supervisor.getTime()
# #         while supervisor.getTime() - t < 60:

# #             # perform robot control according to a, b
# #             # (and possibly t) parameters.

# #             # controller termination
# #             if supervisor.step(TIME_STEP) == -1:
# #                 quit()

# #         # compute travelled distance
# #         values = trans_field.getSFVec3f()
# #         dist = sqrt(values[0] * values[0] + values[2] * values[2])
# #         print("a=%d, b=%d -> dist=%g" % (a, b, dist))

#         # reset robot position and physics
#         INITIAL = [0, 0.5, 0]
#         trans_field.setSFVec3f(INITIAL)
#         robot_node.resetPhysics()