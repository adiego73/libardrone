#ifndef MESSAGESERVER_H
#define MESSAGESERVER_H

#include <stdexcept>
#include <sstream>
#include <vector>
#include <map>

#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>

namespace tesis
{

class MessageServer
{
    public:
        MessageServer();
        ~MessageServer();

        void announce( std::string topic );
        void publish( std::string topic, std::string message );
        std::string get( std::string topic );
        std::string get( std::string topic, std::string default_value );
        std::vector<std::string> topics();
        
    private:
        std::map<std::string, std::string> topic_map;
        boost::mutex msg_mutex;
};

}
#endif // MESSAGESERVER_H
