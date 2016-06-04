#ifndef MESSAGESERVER_H
#define MESSAGESERVER_H

#include <stdexcept>
#include <sstream>
#include <vector>
#include <map>

#include <iostream>

#include <boost/thread.hpp>

namespace tesis
{

class MessageServer
{
public:
	MessageServer();
	~MessageServer();

	void announce ( std::string topic );
	void publish ( std::string topic, std::string message );
	std::string get ( std::string topic );
	std::string get ( std::string topic, std::string default_value );
	float getFloat ( std::string topic );
	float getFloat ( std::string topic, float default_value );
	int getInt ( std::string topic );
	int getInt ( std::string topic, int default_value );
	long getLong ( std::string topic );
	long getLong ( std::string topic, long default_value );
	bool getBool ( std::string topic );
	bool getBool ( std::string topic, bool default_value );
	std::vector<std::string> topics();

private:
	std::map<std::string, std::string> topic_map;
	boost::mutex msg_mutex;
};

}
#endif // MESSAGESERVER_H
