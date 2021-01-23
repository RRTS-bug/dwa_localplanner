
import rospy
import numpy as np
import math
class DWA(object):
    def __init__(self,vehicleParam,dwaParam):
        """
        :param vehicleParam:
                maxSpeed       (float) : Maximum speed of the robot eg:2m/s
                minSpeed       (float) : minimum speed of the robot eg:0.5m/s
                maxAngularSpeed(float) : maximum angular Speed of the robot eg:70/180*math.pi rad/s
                maxAcc         (float) : maximum acceleration of the robot  eg:0.2m/s^2
                maxAngularAcc  (float) : maximum angular acceleration of the robot eg:70/180*math.pi rad/s^2


        :param dwaParam:
               speedReso      (float) : linearSpeed search space resolution of the dwa  eg:0.01m/s
               angularReso    (float) : angularSpeed search space resolution of the dwa eg:0.1/180*math.pi rad/s
               dt             (float) : sample time of the sensor eg:0.1/s
               predictTime    (float) : predict time of the dwa   eg:3/s
               headingWeight  (float) : Weight coefficient of head function eg:0.5
               speedWeight    (float) : Weight coefficient of speed function eg:1
               odistWeight    (float) : Weight coefficient of odist function eg:2
               gdistWeight    (float) : Weight coefficient of gdist function eg:2
        """
        self.maxSpeed = vehicleParam['maxSpeed']
        self.minSpeed = vehicleParam['minSpeed']
        self.maxAngularSpeed = vehicleParam['maxAngularSpeed']
        self.maxAcc = vehicleParam['maxAcc']
        self.maxAngularAcc = vehicleParam['maxAngularAcc']
        self.dt = vehicleParam['dt']
        self.speedReso = dwaParam['speedReso']
        self.angularReso = dwaParam['angularReso']
        self.predictTime = dwaParam['predictTime']
        self.headWeight = dwaParam['headWeight']
        self.speedWeight = dwaParam['speedWeight']
        self.odistWeight = dwaParam['odistWeight']
        self.gdistWeight = dwaParam['gdistWeight']


    def dynamicWindowFunc(self,odomLinearSpeed,odomAngularSpeed):##dynamic window func
        vsSpace = [self.minSpeed, self.maxSpeed,- self.maxAngularSpeed, self.maxAngularSpeed]

        # Dynamic window from motion model
        vdSpace = [odomLinearSpeed - self.maxAcc * self.dt,
              odomLinearSpeed + self.maxAcc * self.dt,
              odomAngularSpeed - self.maxAngularAcc * self.dt,
              odomAngularSpeed + self.maxAngularAcc * self.dt]

        dynamicWindow = [max(vsSpace[0], vdSpace[0]), min(vsSpace[1], vdSpace[1]), max(vsSpace[2], vdSpace[2]), min(vsSpace[3], vdSpace[3])]
        return dynamicWindow

    def headFunc(self,trajectory,goal):
        ##formular 7
        dx = goal[0] - trajectory[-1][0]
        dy = goal[1] - trajectory[-1][1]
        difAngle = math.atan2(dy, dx)
        costAngle = math.fabs(difAngle - trajectory[-1][2])
        cost = math.atan2(1, costAngle)
        return cost

    def odistFunc(self,obSum,trajectory):
        ##formular 8
        if obSum == []:
            return 0
        else:
            ox = np.array(obSum)[:, 0]
            oy = np.array(obSum)[:, 1]
            dx = np.array(trajectory)[:, 0] - ox[:, None]
            dy = np.array(trajectory)[:, 1] - oy[:, None]
            distToObstacle = np.hypot(dx, dy)
            ##Vd
            if (trajectory[-1][3] <= math.sqrt(2 * np.min(distToObstacle) * self.maxAcc)) and (trajectory[-1][4] <= math.sqrt(2 * np.min(distToObstacle) * self.maxAngularAcc)):
                min_dist = np.min(distToObstacle)
                return min_dist
            else:
                return float("inf")


    def speedFunc(self,trajectory):
        ##formular 9
        cost = math.fabs(trajectory[-1][3]) / self.maxSpeed
        return cost

    def gdistFunc(self,trajectory,goal):
        ##formula 10
        distToGoal = [math.hypot(goal[0]-trajectory[i][0],goal[1]-trajectory[i][1]) for i in range(len(trajectory))]
        cost = 1/min(distToGoal)
        return cost

    def predictFunc(self,linearSpeed,angularSpeed):
        trajectory = []
        time = 0
        xOrd = 0
        yOrd = 0
        angle = 0
        while True:
            if time <= self.predictTime:
                ##formula
                xOrd = xOrd + linearSpeed * math.cos(angle) * self.dt
                yOrd = yOrd + linearSpeed * math.sin(angle) * self.dt
                angle = angle + angularSpeed * self.dt
                trajectory.append([xOrd,yOrd,angle,linearSpeed,angularSpeed])
                time = time + self.dt
            else:
                break
        return trajectory

    def trajectoryGenerator(self,dynamicWindow,obSum,goal):
        trajectorySum = []
        cost = []
        for linearSpeed in np.arange(dynamicWindow[0],dynamicWindow[1],self.speedReso):
            for angularSpeed in np.arange(dynamicWindow[2],dynamicWindow[3],self.angularReso):

                trajectory = self.predictFunc(linearSpeed,angularSpeed)
                headCost =  self.headFunc(trajectory,goal)
                odistCost =  self.odistFunc(obSum,trajectory)
                if odistCost == float('inf'):
                    continue
                speedCost =  self.speedFunc(trajectory)
                gdistCost =  self.gdistFunc(trajectory,goal)
                trajectorySum.append(trajectory)
                cost.append([headCost,odistCost,speedCost,gdistCost])
        ##smoothing
        headCostSum = sum(np.array(cost)[:,0])
        odistCostSum = sum(np.array(cost)[:,1])
        speedCostSum = sum(np.array(cost)[:,2])
        gdistCostSum = sum(np.array(cost)[:,3])
        if odistCostSum == 0:
            costSum = [sum([self.headWeight * cost[i][0]/headCostSum , self.speedWeight * cost[i][2]/speedCostSum,self.gdistWeight * cost[i][3] / gdistCostSum]) for i in range(len(cost))]
        else:
            costSum = [sum([self.headWeight * cost[i][0]/headCostSum , self.odistWeight * cost[i][1] / odistCostSum, self.speedWeight * cost[i][2] / speedCostSum, self.gdistWeight * cost[i][3] / gdistCostSum]) for i in range(len(cost))]
        ind = costSum.index(max(costSum))
        bestlinear = trajectorySum[ind][-1][3]
        bestangluar = trajectorySum[ind][-1][4]
        return  bestlinear,bestangluar

    def main(self,obSum,goal,odomLinearSpeed,odomAngularSpeed):
        """
        The main function of dwa algorithm, select the best linearspeed and angularspeed at the next moment through the dynamic window
        robot coordinate system the positive direction of the x-axis is the forward direction of the robot, and the positive direction of the y-axis is the left side of the forward direction of the robot

        :param obSum       (list):     obstacle Point cloud ordinate received by the sensor,[[x1,y1],[x2,y2]......[xn,yn]]
        :param goal        (list):     Target point coordinates in robot coordinate system [x,y]
        :param odomSpeed   (float):    robot linearspeed  eg:1.01m/s
        :param odomAngular (float):    robot angularspeed eg:0.02rad/s
        :return:
            linearSpeed  (float): Optimal linearspeed calculated by dwa algorithm  eg:1.03m/s
            angularSpeed (float): Optimal angularspeed calculated by dwa algorithm eg:0.04 rad/s

        """
        ##get the dynamic window
        dynamicWindow = self.dynamicWindowFunc(odomLinearSpeed,odomAngularSpeed)
        ## Select the best linear velocity and angular velocity through the dynamic window, the point cloud and the target point
        linearspeed,angularSpeed = self.trajectoryGenerator(dynamicWindow,obSum,goal)

        return linearspeed,angularSpeed