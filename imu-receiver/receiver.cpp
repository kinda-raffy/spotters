#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

int main() {
    const int port {9000}, descriptor {socket(AF_INET, SOCK_DGRAM, 0)};
    if (descriptor < 0) {
        std::cerr << "Error creating socket.\n";
        return EXIT_FAILURE;
    }
    // TODO: Add nodes and handler configuration values.
    std::string ip {"GAPE"};
    struct sockaddr_in server {};
    memset(&server, '\0', sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
    server.sin_addr.s_addr = inet_addr(ip.c_str());
    if (connect(descriptor, (struct sockaddr*) &server, sizeof(server)) < 0) {
        std::cerr << "Connection failed.\n";
        return EXIT_FAILURE;
    }

    const std::size_t length {4096}, required {10};
    char buffer[length + 1] {}, separator {','};
    std::vector<std::string> values {};
    while (true) { // TODO: Should loop while ROS is running.
        const ssize_t size {read(descriptor, buffer, length)};
        if (size <= 0) {
            std::cerr << "Error reading data.\n";
            continue;
        }
        buffer[size] = '\0';
        values.clear();
        std::stringstream input {buffer};
        while (input.good()) {
            std::string value {};
            std::getline(input, value, separator);
            values.push_back(value);
        }
        if (values.size() == required) {
            for (const std::string &value : values) {
                std::cout << value << "\n";
            }
        } else {
            std::cerr << "Wrong values received.\n";
        }
        sleep(1); // Should spin once.
    }
}
