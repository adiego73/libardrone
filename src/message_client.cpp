#include "message_client.hpp"

using namespace robot;

const std::string MessageClient::MESSAGE_SERVER_URL = "localhost";

MessageClient::MessageClient()
{

}

MessageClient::~MessageClient()
{

}

void MessageClient::announce( std::string topic )
{

}

std::string MessageClient::get( std::string topic, std::string default_msg )
{
    return "";
}

void MessageClient::publish( std::string topic, std::string message )
{

}
