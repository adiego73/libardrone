#include "message_client.hpp"

using namespace robot;

const std::string MessageClient::MESSAGE_SERVER_URL = "localhost";

MessageClient::MessageClient()
    : endpoint( boost::asio::ip::address::from_string( MessageClient::MESSAGE_SERVER_URL ), MessageClient::MESSAGE_SERVER_PORT ),
      socket( io_service )
{
    // this will try to connect the socket once the client is constructed.
    this->socket.connect( this->endpoint );
}

MessageClient::~MessageClient()
{
    this->socket.close();
}

void MessageClient::announce( std::string topic )
{
    std::string to_announce = "announce|" + topic;
    this->writeAndRead( to_announce );
}

std::string MessageClient::get( std::string topic, std::string default_msg )
{
    std::string to_get = "get|" + topic + "|" + default_msg;
    return this->writeAndRead( to_get );
}

void MessageClient::publish( std::string topic, std::string message )
{
    std::string to_publish = "publish|" + topic + "|" + message;

    this->writeAndRead( to_publish );
}

std::string MessageClient::writeAndRead( std::string msg )
{
    boost::system::error_code error;
    std::string response;

    boost::asio::write( this->socket, boost::asio::buffer( msg ), boost::asio::transfer_exactly( msg.size() ), error );

    boost::asio::streambuf buffer;

    std::size_t size = boost::asio::read( this->socket, buffer, error );

    if( error == boost::asio::error::eof )
    {
        std::string msg_error = std::string( "EOF reached when reading socket. " ) + "( " + error.message() + " )";
        throw std::runtime_error( msg_error );
    }
    else if( error )
    {
        std::string msg_error = std::string( "Error: " ) + error.message();
        throw std::runtime_error( msg_error );
    }

    std::istream str( &buffer );
    std::getline( str, response );

    return response;
}
