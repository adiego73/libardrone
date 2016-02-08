#ifndef MESSAGECLIENT_H
#define MESSAGECLIENT_H

#include <string>

namespace robot
{

class MessageClient
{
    public:
        std::string get( std::string topic, std::string default_msg );
        void announce( std::string topic );
        void publish( std::string topic,  std::string message );


        MessageClient();
        ~MessageClient();

    private:
        static const int MESSAGE_SERVER_PORT = 9090;
        static const std::string MESSAGE_SERVER_URL;
};

}
#endif // MESSAGECLIENT_H
