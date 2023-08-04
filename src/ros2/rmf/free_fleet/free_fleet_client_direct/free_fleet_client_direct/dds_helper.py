from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic


class dds_subscriber():

    def __init__(self, dds_name, domain_id, topic_name, message_type):
        self.dds_name =dds_name
        domain_participant = DomainParticipant(domain_id=domain_id)
        topic = Topic(domain_participant, topic_name, message_type)
        self.reader= DataReader(domain_participant, topic)
    
    def dds_get_next_msg(self):
        return self.reader.take_next()