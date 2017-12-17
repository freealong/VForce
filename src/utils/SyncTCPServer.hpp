//
// Created by yongqi on 17-11-14.
//

#ifndef VFORCE_SYNCTCPSERVER_HPP
#define VFORCE_SYNCTCPSERVER_HPP

#include <boost/asio.hpp>

namespace VForce {
namespace Utils {

template <typename RecvMsgT, typename SendMsgT>
class SyncTCPServer {
 public:
  SyncTCPServer(std::string address = "127.0.0.1", unsigned short port = 8000);

  void WaitingClient() {
    acceptor_->accept(*socket_);
  }

  /**
   * Receive message from client
   * @param msg the received message
   * @return read bytes size
   */
  int RecvMsg(RecvMsgT &msg);

  /**
   * Send message to client
   * @param msg the message will be sent
   * @return write bytes size
   */
  int SendMsg(const SendMsgT &msg);

 private:
  boost::asio::io_service io_service_;
  boost::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor_;
  boost::shared_ptr<boost::asio::ip::tcp::socket> socket_;
};

using boost::asio::ip::tcp;

template <typename RecvMsgT, typename SendMsgT>
SyncTCPServer<RecvMsgT, SendMsgT>::SyncTCPServer(std::string address, unsigned short port) {
  auto addr = boost::asio::ip::address::from_string(address);
  acceptor_ = boost::shared_ptr<tcp::acceptor>(new tcp::acceptor(io_service_, tcp::endpoint(addr, port)));
  socket_ = boost::shared_ptr<tcp::socket>(new tcp::socket(io_service_));
}

template <typename RecvMsgT, typename SendMsgT>
int SyncTCPServer<RecvMsgT, SendMsgT>::RecvMsg(RecvMsgT &msg) {
  auto len = boost::asio::read(*socket_, boost::asio::buffer(&msg, sizeof(msg)));
//  DLOG(INFO) << "Receive Message(" << len << " bytes): " << msg;
  return static_cast<int>(len);
}

template <typename RecvMsgT, typename SendMsgT>
int SyncTCPServer<RecvMsgT, SendMsgT>::SendMsg(const SendMsgT &msg) {
  auto len = boost::asio::write(*socket_, boost::asio::buffer(&msg, sizeof(msg)));
//  DLOG(INFO) << "Send Message(" << len << " bytes): " << msg;
  return static_cast<int>(len);
}

}
}
#endif //VFORCE_SYNCTCPSERVER_HPP
