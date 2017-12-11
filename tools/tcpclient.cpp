//
// Created by yongqi on 4/25/17.
//

#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <iostream>

using namespace std;

int connect_server(const char *address, int port) {
  struct sockaddr_in serv_addr;
  struct hostent *server;

  int sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    printf("ERROR opening socket\n");
    return -1;
  }
  server = gethostbyname(address);
  if (server == NULL) {
    printf("ERROR, no such host\n");
    sockfd = -1;
    return -1;
  }
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *) server->h_addr,
        (char *) &serv_addr.sin_addr.s_addr,
        server->h_length);
  serv_addr.sin_port = htons(port);
//    qDebug() << "port = " << port;
//    qDebug() << "sockfd = " << sockfd;
  if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    printf("ERROR tcp connecting\n");
    sockfd = -1;
    return -1;
  }
  printf("connect to server successfully.\n");

  return sockfd;
}

int main(int argc, char **argv) {
  string address("127.0.0.1");
  int port = 8000;
  if (argc > 1) address = argv[1];
  if (argc > 2) port = atoi(argv[2]);
  cout << "connecting " << address << ":" << port << endl;
  int sockfd = connect_server(address.data(), port);
  string msg("a");

  while (true) {
    cin >> msg;
    if (msg == "a") {
      write(sockfd, msg.data(), 1);
      char buffer[256];
      printf("get target Pose(x, y, z, R, P, Y):\n");
      double flag;
      read(sockfd, &flag, sizeof(flag));
      for (int i = 0; i < 6; ++i) {
        double x;
        read(sockfd, &x, sizeof(x));
        printf("%f ", x);
      }
      printf("\n");
    } else if (msg == "b") {
      write(sockfd, msg.data(), 1);
      char buffer[256];
      printf("get refine Pose(x, y, z, R, P, Y):\n");
      read(sockfd, buffer, 41);
      buffer[41] = 0;
      printf("%s\n", buffer);
    } else if (msg == "q")
      break;
  }
/*  write(sockfd, msg.data(), msg.size());
  printf("\nget second target:\n");
  for (int i = 0; i < 6; ++i) {
    read(sockfd, buffer, 6);
    printf("%s ", buffer);
  }
*/
  close(sockfd);
}
