#ifndef MESSAGECLIENT_H
#define MESSAGECLIENT_H

#include <iostream>
#include <string>
#include <exception>

#include <boost/asio.hpp>

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
        
        boost::asio::ip::tcp::socket socket;
        boost::asio::ip::tcp::endpoint endpoint;
        boost::asio::io_service io_service;
        std::string writeAndRead(std::string msg);
};

}
#endif // MESSAGECLIENT_H

