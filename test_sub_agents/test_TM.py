from python_arbi_framework.arbi_agent.agent.arbi_agent import ArbiAgent
from python_arbi_framework.arbi_agent.configuration import BrokerType
from python_arbi_framework.arbi_agent.agent import arbi_agent_excutor
from arbi_agent.model import generalized_list_factory as GLFactory

import time

arbiNavManager = "agent://www.arbi.com/navManager"
arbiMAPF = "agent://www.arbi.com/MAPF"
arbiMapManager = "agent://www.arbi.com/MapManagerAgent"
arbiTA = "agent://www.arbi.com/TA"

robotMap = {"lift":["AMR_LIFT1", "AMR_LIFT2"], "tow":["AMR_TOW1","AMR_TOW2"]}


class aAgent(ArbiAgent):
    def __init__(self, agent_name, broker_url = "tcp://127.0.0.1:61616"):
        super().__init__()
        self.broker_url = broker_url
        self.agent_name = agent_name
        #self.agent_url = agent_url

    def on_data(self, sender: str, data: str):
        print(self.agent_url + "\t-> receive data : " + data)
    
    def on_request(self, sender: str, request: str) -> str:
        print(self.agent_url + "\t-> receive request : " + request)
        return "(request ok)"
    
    """
    def on_notify(self, content):
        gl_notify = GLFactory.new_gl_from_gl_string(content)
    """
    def on_query(self, sender: str, query: str) -> str:
        print(self.agent_url + "\t-> receive query : " + query)
        #print(query)
        #return handle_request(query)
        return "query"

    def execute(self, broker_type=2):
        arbi_agent_excutor.excute(self.broker_url, self.agent_name, self, broker_type)
        print(self.agent_name + " ready")



arbi = aAgent("agent://www.arbi.com/TM")
arbi.execute()

#testMsg = "(taskAllocation \"StoringCarrier\" (goal (metadata \"donno\") \"dc\" (argument \"idk\" \"station3\" \"station6\")))"
testMsg = "(taskAllocation \"StoringCarrier\" (goal (metadata \"donno\") \"dc\" (argument \"idk\" \"station3\" \"\")))"

print(arbi.request(arbiTA,testMsg))

while(1):
    time.sleep(0.01)
