import signal
import sys
import json

from google.oauth2 import service_account
from google.cloud import pubsub_v1


class Gesture_Subscriber_Node:
    def __init__(self, project_id, topic_name, subscriber_name):

        """Receives messages from a Pub/Sub subscription."""
        pubsub_key = json.load(open('pubsub_key.json'))
        credentials = (service_account.Credentials.from_service_account_info(pubsub_key))

        """Publishes a message to a Pub/Sub topic."""
        # Initialize a Subscriber client.
        self.subscriber_client = pubsub_v1.SubscriberClient(credentials=credentials)
        self.publisher_client = pubsub_v1.PublisherClient(credentials=credentials)

        # Create a fully qualified identifier in the form of
        # `projects/{project_id}/subscriptions/{subscription_id}`
        self.subscription_path = self.subscriber_client.subscription_path(project_id, subscriber_name)
        self.topic_path = self.publisher_client.topic_path(project_id, topic_name)


    def sub(self):

        sub = self.subscriber_client.create_subscription(name=self.subscription_path, topic=self.topic_path)

        def callback(message):
            print(
                "Received message {} of message ID {}\n".format(message, message.message_id)
            )
            # Acknowledge the message. Unack'ed messages will be redelivered.
            message.ack()
            print("Acknowledged message {}\n".format(message.message_id))

        streaming_pull_future = self.subscriber_client.subscribe(self.subscription_path, callback=callback)
        print("Listening for messages on {}..\n".format(self.subscription_path))

        try:
            # Calling result() on StreamingPullFuture keeps the main thread from
            # exiting while messages get processed in the callbacks.
            streaming_pull_future.result()
        except:  # noqa
            streaming_pull_future.cancel()

        #

    #cleanup function
    def signal_handler(self, sig, frame):
        print('Deleting subscriber before closing...')
        self.subscriber_client.delete_subscription(self.subscription_path)
        self.subscriber_client.close()
        print('Finished cleanup, exiting!')
        sys.exit(0)

if __name__ == "__main__":
    gsn = Gesture_Subscriber_Node('verification-of-hri', 'robot_topic', 'robot_sub')
    signal.signal(signal.SIGINT, gsn.signal_handler)
    gsn.sub()
# [END pubsub_quickstart_sub_all]