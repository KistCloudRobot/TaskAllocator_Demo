from arbi_agent.model.generalized_list import GeneralizedList

import deps.matching as matching
import numpy as np
from threading import Thread, Condition
import os

from arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.configuration import BrokerType
from arbi_agent.agent import arbi_agent_excutor
from arbi_agent.model import generalized_list_factory as GLFactory

import time
import deps.printInColor as pic

superLargeCost = 99999999
robot_path_delim = ':'
robot_robot_delim = ';'
path_path_delim = '-'
arbiNavManager = "agent://www.arbi.com/Local/NavigationController"
arbiTaskManager = "agent://www.arbi.com/Local/TaskManager"
arbiContextManager = "agent://www.arbi.com/Local/ContextManager"
arbiMAPF = "agent://www.arbi.com/Local/MultiAgentPathFinder"
arbiMapManager = "agent://www.arbi.com/Local/MapManager"
arbiThis = "agent://www.arbi.com/Local/TaskAllocator"

robotMap = {"lift": ["AMR_LIFT1", "AMR_LIFT2"], "tow": ["AMR_TOW1", "AMR_TOW2"]}


class aAgent(ArbiAgent):
    def __init__(self, agent_name, broker_url="tcp://172.16.165.204:8000"):
        super().__init__()
        self.broker_url = broker_url
        self.agent_name = agent_name
        self.lock = Condition()
        self.response = None

    def on_data(self, sender: str, data: str):
        print(self.agent_url + "\t-> receive data : " + data)

    def on_request(self, sender: str, request: str) -> str:
        print(self.agent_url + "\t-> receive request : " + request)
        time.sleep(0.05)
        result = handleRequest(request)
        print("result :", result)
        return result

    def on_notify(self, sender: str, notification: str):
        print(sender + "\t-> notification : " + notification)

    def on_query(self, sender: str, query: str) -> str:
        print(self.agent_url + "\t-> receive query : " + query)
        return "(ok)"

    def execute(self, broker_type=2):
        arbi_agent_excutor.execute(self.broker_url, self.agent_name, self, broker_type)
        print(self.agent_name + " ready")


class robotPlan:
    def __init__(self, name, start, goal=""):
        self.name = name
        self.start = start
        self.goal = goal

    def __str__(self):
        result = "{"
        result += "\"name\" : \"" + self.name + "\" "
        result += "\"start\" : " + str(self.start) + " "
        result += "\"goal\" : " + str(self.goal)
        return result


def handleRequest(msg_gl):
    # (TaskAllocation $role (goal (metadata $goalID) $goalName (argument $arg1 $arg2 ...)))
    # keep all the other info in str, take goals out (picking)
    # figure out all applicable robot id from somewhere. Currently predefined
    # request robot avilability from map manager and remove unavailable robot, then do planning

    gl = GLFactory.new_gl_from_gl_string(msg_gl)
    request_name = gl.get_name()

    if request_name == 'TaskAllocation':
        print("before parse TM request")
        goalID, robotPlanSet, goals = parseTMreq(gl)
        print("after parse TM request")
        # assumeing for test
        # robotPlan(robot_id,current_vertex(in str))
        # for test
        # robots = (robotPlan("agent1","219"),robotPlan("agent2","222"));
        print("before check plan correct")
        if len(robotPlanSet) == 0:
            # no robot is available. return no allocation here
            time.sleep(0.05)
            arbiAgent.send(arbiTaskManager, "(AgentRecommanded " + "\"failed\" \"" + goalID + "\")")
        print("after check plan correct")
        robots = robotPlanSet
        # fill cost matrix
        # n by m matrix. n = n of robots (rows), m = number of goals(cols)
        # cost_mat = np.random.rand(nRobots, nRobots*numWays)*10
        print("before allocate")
        allocRobotPlans = allocationCore(robots, goals)
        print("after allocate")
        # final Multi Agent plan Request
        return generate_TM_response(allocRobotPlans, goalID)
        # return finalResult_gl
        # toss it to other ARBI Agent
        # arbiAgent.send(arbiNavManager, finalResult)
    else:
        pic.printC("Unknown Request GL Name " + request_name, 'fail')
        return "(fail)"


def handleData(msg_gl):
    pass


def glEx2str(glExpression):
    gl_str = str(glExpression)
    if gl_str[0] == '\"':
        return gl_str[1:-1]
    else:
        return gl_str


def str2Glstr(str):
    return "\"" + str + "\""


def parseTMreq(gl: GeneralizedList):
    print(1)
    role = gl.get_expression(0).as_value().string_value()
    print(2)
    corr_robots = None
    print(3)
    if role == "StoringCarrier" or role == "UnstoringLargeCarrier":  # LIFT
        print(4)
        corr_robots = robotMap['lift']
    elif role == "UnstoringSmallCarrier":
        print(5)
        corr_robots = robotMap['tow']
    else:
        print(6)
        pic.printC("Unknown Role: " + role, 'fail')
        # todo: handle exception
    print(7)

    goals = list()
    goalID = None
    # 3 expressions in goal gl

    print(8)
    gl_goal = gl.get_expression(1).as_generalized_list()
    print(9)
    if gl_goal.get_name() == "goal":
        print(10)
        # fist element = (metadata $goalID)
        goalID = gl_goal.get_expression(0).as_generalized_list().get_expression(0).as_value().string_value()
        # second element = $goalName
        goalName = gl_goal.get_expression(1).as_value().string_value()
        # third element = (argument $arg1 $arg2 $arg3)
        goalArgs = gl_goal.get_expression(2).as_generalized_list()
        # start vertex
        queryString = "(StationVertex \"" + goalArgs.get_expression(1).as_value().string_value() + "\" $vertex)"
        time.sleep(0.05)
        queryResult = arbiAgent.query(arbiContextManager, queryString)
        resultGL = GLFactory.new_gl_from_gl_string(queryResult)
        startVertex = resultGL.get_expression(1).as_value().int_value()
        goals.append(startVertex)
        # goal vertex
        queryString = "(StationVertex \"" + goalArgs.get_expression(2).as_value().string_value() + "\" $vertex)"
        time.sleep(0.05)
        queryResult = arbiAgent.query(arbiContextManager, queryString)
        resultGL = GLFactory.new_gl_from_gl_string(queryResult)
        goalVertex = resultGL.get_expression(1).as_value().int_value()
        goals.append(goalVertex)

    robotPlanSet = list()
    for r in corr_robots:
        print(11)
        # get vertex and availability of each robot from map manager
        time.sleep(0.05)
        res = arbiAgent.query(arbiMapManager, "(RobotSpecInfo \"" + r + "\")")
        print(12)
        # (RobotSpecInfo (RobotInfo $robot_id (vertex_id $v_id1 $v_id2) $load $goal), …)
        res_gl = GLFactory.new_gl_from_gl_string(res)
        print(13)
        # (RobotInfo $robot_id (vertex_id $v_id1 $v_id2) $load $goal)
        res_sub_gl = res_gl.get_expression(0).as_generalized_list()
        print(14)
        if res_sub_gl.get_expression(0).as_value().string_value() == r:
            # choose the first neighboring vertex as the start point
            print(15)
            v = res_sub_gl.get_expression(1).as_generalized_list().get_expression(0).as_value().int_value()
            print(16)
            # unload = 0, load = 1
            load = res_sub_gl.get_expression(2).as_value().int_value()
            print(17)
            # not currently in use
            cur_goal = res_sub_gl.get_expression(3).as_value().string_value()
            print(18)
            # do not add to robot list if robot is loaded
            if load == 0:
                print(19)
                robotPlanSet.append(robotPlan(r, v))
        else:
            print(20)
            pic.printC("Robot ID not Matched", 'fail')

    return goalID, robotPlanSet, goals


def allocationCore(robots, goals):
    cost_mat = generateCostMatrix(robots, goals, arbiAgent)
    # fill missing rows/cols to make it square
    cost_mat = makeSqMat(cost_mat)
    # assignment, cost = matching.matching(cost_mat, numWays)
    assignment, cost = matching.matching(cost_mat)
    allocMat = assignment.astype("int")

    # remove allocation with super large cost
    c_rows, c_cols = cost_mat.shape
    for i in range(c_rows):
        for j in range(c_cols):
            if cost_mat[i, j] > (superLargeCost - 1):
                allocMat[i, j] = int(0)

    # print("***RESULT (numWays = %d)***\n" %numWays)
    print("***RESULT***\n")
    print("The cost matrix")
    print(cost_mat)
    print("\nThe optimal allocation")
    print(allocMat)
    print("\nThe cost sum: %f" % cost)

    # iterate matrix to extract allocation result
    # result = name1:node1,node2, ... ,noden;name2:node1,node2, ... ,noden
    allocRobotPlans = []
    for row in range(len(robots)):
        for col in range(len(goals)):
            if allocMat[row][col] == 1:
                robots[row].goal = goals[col]
                allocRobotPlans.append(robots[row])

    return allocRobotPlans


def generate_TM_response(allocRobotPlans, goalID):
    # final Multi Agent plan Request
    finalResult_gl = planMultiAgentReqest(allocRobotPlans, arbiAgent)
    finalResult = arbi2msg_res(finalResult_gl)
    pic.printC("Allocation Result: " + finalResult, 'cyan')
    finalResult_list_by_robot = finalResult.split(robot_robot_delim)
    out_id_goal_pair_list = []
    for robot in finalResult_list_by_robot:
        robot_sp = robot.split(robot_path_delim)
        robot_id_out = robot_sp[0]
        goal_vertex = robot_sp[1].split(path_path_delim)[-1]
        # TODO get first result that means from start point to rack point
        if "LIFT2" in robot_id_out:
            out_id_goal_pair_list.append((robot_id_out, goal_vertex))
            break

    # generate returning gl string
    # currently assuming there is only one task allocation each time
    out_str = "(AgentRecommanded"
    for pair in out_id_goal_pair_list:
        out_str += " " + str2Glstr(pair[0]) + " \"" + goalID + "\""
    out_str += ")"

    return out_str


def msg2arbi_req(msg, header="MultiRobotPath", pathHeader="RobotPath"):
    # name1,start1,goal1;name2,start2,goal2, ...
    # (MultiRobotPath (RobotPath $robot_id $cur_vertex $goal_id), …)

    out_msg = "(" + header + " "
    planList = list()
    msgList = msg.split(robot_robot_delim)
    for r in msgList:
        # name1,start1,goal1
        # (RobotPath $robot_id $cur_vertex $goal_id) -> append to planList
        elems = r.split(robot_path_delim)
        planList.append('(' + pathHeader + " " + "\"" + elems[0] + "\" " + elems[1] + " " + elems[2] + ')')

    out_msg += ' '.join(planList)
    out_msg += ')'

    return out_msg


def arbi2msg_res(arbi_msg, header="MultiRobotPath", pathHeader="RobotPath", singlePathHeader="path"):
    # (MultiRobotPath (RobotPath "agent1" (path 219 220 221 222 223 224 225 15)))
    # name1,start1,goal1;name2,start2,goal2, ...
    gl = GLFactory.new_gl_from_gl_string(arbi_msg)
    robotSet = []
    if gl.get_name() == header:
        for r in range(gl.get_expression_size()):
            # (RobotPath "agent1" (path 219 220 221 222 223 224 225 15))
            rp = str(gl.get_expression(r))
            # back to gl
            gl_sub = GLFactory.new_gl_from_gl_string(rp)
            robot_name = str(gl_sub.get_expression(0))[1:-1]
            path_list = str(gl_sub.get_expression(1))[1:-1].split(' ')
            # remove gl name
            path_list.pop(0)

            robotSet.append(robot_name + robot_path_delim + path_path_delim.join(path_list))

    return robot_robot_delim.join(robotSet)


def responsToDict(res):
    out_dict = {}
    # robots are separated by robot_robot_delim
    resByRobots = res.split(robot_robot_delim)

    for pathMsg in resByRobots:
        # name and path nodes are separated by robot_path_delim
        pathMsg_sp = pathMsg.split(robot_path_delim)
        # value could be 'failed'. It will be turned to ['failed']
        path_list = pathMsg_sp[1].split(path_path_delim)
        out_dict[pathMsg_sp[0]] = path_list

    return out_dict


# respoinse in arbi Gl format
def planMultiAgentReqest(robotPlans, arbi):
    msgByRobot = []
    for r in robotPlans:
        if type(r.goal) == int:
            singleMsg = (r.name + robot_path_delim + str(r.start) + robot_path_delim + str(r.goal))
            msgByRobot.append(singleMsg)
        else:
            print("type error?")
    reqMsg = robot_robot_delim.join(msgByRobot)

    # convert to arbi msg
    arbiMsg = msg2arbi_req(reqMsg)

    reqStartTime = time.time()
    time.sleep(0.05)
    res = arbi.request(arbiMAPF, arbiMsg)
    reqEndTime = time.time()
    pic.printC("Request took " + str(reqEndTime - reqStartTime) + " seconds", 'warning')

    # return in gl format
    return res
    # return res


def generateCostMatrix(robotPlans, goals, arbi):
    costMat = np.ones((len(robotPlans), len(goals)))
    # iterate to fill matrix
    # for each robot
    serializedCosts = []
    for r in robotPlans:
        for g in goals:
            # name1,start1,goal1;name2,start2,goal2, ...
            # msg = (r.name + "," + r.start + "," + g)
            # send through arbi
            # reqStartTime = time.time()
            # pathMsg = arbi.query(arbiMAPF,msg)
            pathMsg_gl = planMultiAgentReqest([robotPlan(r.name, r.start, g)], arbi)
            pathMsg = arbi2msg_res(pathMsg_gl)
            # reqEndTime = time.time()
            # print("Request took " + str(reqEndTime - reqStartTime) + " seconds")
            # expect to have result for only one robot
            planResultDict = dict()
            if len(pathMsg) > 0:
                planResultDict = responsToDict(pathMsg)
            # fill cost matrix
            # if robot name is matched
            if r.name in planResultDict:
                # if plan was a success
                if planResultDict[r.name][0] != 'failed':
                    serializedCosts.append(len(planResultDict[r.name]))
                else:
                    # path plan failed. apply super large cost
                    serializedCosts.append(superLargeCost)
            else:
                # path plan failed. apply super large cost
                serializedCosts.append(superLargeCost)
            """
            pathMsg_sp = pathMsg.split(',')
            #if robot name is matched
            path_list = []
            if(pathMsg_sp[0] == r.name):
                if(pathMsg_sp != 'failed'):
                    path_list = pathMsg_sp[1].split('-')
                    serializedCosts.append(len(path_list))
                else:
                    #path plan failed. apply super large cost
                    serializedCosts.append(superLargeCost)
            """

    # fill matrix out of serialized costs
    # by row vectors
    for ind in range(0, len(serializedCosts)):
        row = ind // len(goals)
        col = ind % len(goals)
        costMat[row, col] = serializedCosts[ind]

    return costMat


def makeSqMat(mat):
    n, m = mat.shape
    if m > n:
        dummy_mat = superLargeCost * np.full(((m - n), m), 1)
        mat = np.vstack((mat, dummy_mat))
    elif m < n:
        dummy_mat = superLargeCost * np.full((n, (n - m)), 1)
        mat = np.hstack((mat, dummy_mat))

    return mat


if __name__ == "__main__":
    # Create and start an ARBI Agent
    global arbiAgent
    broker_url = 'tcp://' + os.environ["JMS_BROKER"]
    arbiAgent = aAgent(agent_name=arbiThis,
                       broker_url=broker_url)
    arbiAgent.execute()

    while True:
        time.sleep(0.01)
