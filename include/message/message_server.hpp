#ifndef MESSAGESERVER_H
#define MESSAGESERVER_H

#include <stdexcept>
#include <sstream>
#include <vector>
#include <map>

#include <iostream>

#include <boost/any.hpp>
#include <boost/thread.hpp>

namespace tesis
{

class MessageServer
{
    public:
        MessageServer();
        ~MessageServer();

        void announce( std::string topic );
        template<typename T> void publish( std::string topic, T message );
        template<typename T> T get( std::string topic, T default_value );
        std::vector<std::string> topics();
};

}
#endif // MESSAGESERVER_H
