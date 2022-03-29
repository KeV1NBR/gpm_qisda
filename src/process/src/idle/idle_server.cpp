#include "idle_server.h"

#include <sstream>

using std::cin;
using std::cout;
using std::endl;
using std::flush;
using std::getline;
using std::stoi;
using std::string;

using Poco::Net::SocketAddress;
using Poco::Net::SocketStream;
using Poco::Net::StreamSocket;

IdleServer::IdleServer(string actionName)
    : server(nh, actionName,
             boost::bind(&IdleServer::executeCallBack, this, _1), false) {
    server.start();
}

IdleServer::~IdleServer() { server.shutdown(); }

void IdleServer::executeCallBack(const process::fsmGoalConstPtr &goal) {
    SocketAddress addr("127.0.0.1", "6666");
    StreamSocket socket(addr);
    string s;
    cout << "Connect to " << addr.toString() << endl;
    SocketStream stream(socket);
    stream << "request" << endl;
    if (stream) {
        getline(stream, s);
        s.erase(s.begin());
        cout << s << endl;
    }
    std::istringstream istr(s);
    for (int i = 0; i getline(istr, tmp, ',');
}
