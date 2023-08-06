from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.core import Listener
from cyclonedds.pub import DataWriter
from threading import Thread

# import fastdds

class DDS_communicator( ):
    def __init__(self, dds_domain_id:int, message_path:str, message_typ):
        self.domain = DomainParticipant(dds_domain_id)
        self.topic = Topic(self.domain, message_path, message_typ)
        self.publischer = DataWriter(self.domain, self.topic)
        self.subscriber = DataReader(self.domain, self.topic)
    
    def publish(self, message):
        self.publischer.write(message)
    
    def get_next(self):
        return self.subscriber.take_next()



# def fastdds_publish(message, topic_name, topic_typ, dds_domain_id ):
#     factory = fastdds.DomainParticipantFactory.get_instance()
#     participant_qos = fastdds.DomainParticipantQos()
#     factory.get_default_participant_qos(participant_qos)
#     participant = factory.create_participant(dds_domain_id, participant_qos)
#     #    match(topic_typ):
#     #         case "j":
#     #             topic_data_type = HelloWorld.HelloWorldPubSubType()
#     #             topic_data_type.setName("HelloWorld")
                 
#     type_support = fastdds.TypeSupport(topic_data_type) 
#     #  topic_data_type = HelloWorld.HelloWorldPubSubType()
#     #  self.topic_data_type.setName("HelloWorld")
       
#     participant.register_type(type_support)

#     topic_qos = fastdds.TopicQos()
#     participant.get_default_topic_qos(topic_qos)
       
#     topic = participant.create_topic(topic_name, topic_data_type.getName(), topic_qos)
       
#     publisher_qos = fastdds.PublisherQos()
#     participant.get_default_publisher_qos(publisher_qos)
#     publisher = participant.create_publisher(publisher_qos)
       
#     writer_qos = fastdds.DataWriterQos()
#     publisher.get_default_datawriter_qos(writer_qos)
#     writer = publisher.create_datawriter(topic, writer_qos)      
#     writer.write(message)



#topic_data_type = HelloWorld.HelloWorldPubSubType()
#topic_data_type.setName("HelloWorld")
#topic_typ =HelloWorld.HelloWorld()
# def fastdds_subscriber(dds_domain_id, topic_name, topic_data_typ, topic_typ, function):
#     factory = fastdds.DomainParticipantFactory.get_instance()
#     participant_qos = fastdds.DomainParticipantQos()
#     factory.get_default_participant_qos(participant_qos)
#     participant = factory.create_participant(dds_domain_id, participant_qos)

#     type_support = fastdds.TypeSupport(topic_data_typ)
    
#     participant.register_type(type_support)
#     topic_qos = fastdds.TopicQos()
#     participant.get_default_topic_qos(topic_qos)
#     topic = participant.create_topic(topic_name, topic_data_typ.getName(), topic_qos)
#     subscriber_qos = fastdds.SubscriberQos()
#     participant.get_default_subscriber_qos(subscriber_qos)

#     subscriber = participant.create_subscriber(subscriber_qos)
#     listener = ReaderListener(function, topic_typ)
#     reader_qos = fastdds.DataReaderQos()
#     subscriber.get_default_datareader_qos(reader_qos)
#     reader = subscriber.create_datareader(topic, reader_qos, listener)

# class ReaderListener(fastdds.DataReaderListener):
#     def __init__(self, function, topic_typ):
#         super().__init__()
#         self.function = function
#         self.topic_typ = topic_typ 
    
#     def on_data_available(self, reader):
#         info = fastdds.SampleInfo()
#         data = self.topic_typ
#         reader.take_next_sample(data, info)
#         self.function(data)