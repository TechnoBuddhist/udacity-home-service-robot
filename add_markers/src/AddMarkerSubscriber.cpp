#include <ros/ros.h>

class AddMarkerSubscriber {
  public:
    AddMarkerSubscriber() {
      //Topic you want to subscribe
      sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
    }

    void callback(const SUBSCRIBED_MESSAGE_TYPE& input) {
      PUBLISHED_MESSAGE_TYPE output;
      //.... do something with the input and generate the output...
      pub_.publish(output);
    }

  private:
    ros::NodeHandle n_; 
    ros::Subscriber sub_;
};