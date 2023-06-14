#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>

#define REQUIRED_VALUES 100 // TODO: This should be the number of values.
#define BUFFER_SIZE 4096
#define WARNING_SIZE 750
#define INPUT_DELIMITER ','

typedef sockaddr SocketAddress;
typedef sockaddr_in InternetSocketAddress;

int main() {
    const int socket_file_descriptor {socket(AF_INET, SOCK_DGRAM, 0)}, port {}; // TODO: Add the port.
    if (socket_file_descriptor < 0) {
        std::cerr << "Error creating socket.\n";
        return EXIT_FAILURE;
    }

    char ip[] = {"1-800-GAPE"};  // TODO: Add the IP.
    InternetSocketAddress server_address;
    memset(&server_address, '\0', sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(port);
    server_address.sin_addr.s_addr = inet_addr(ip);
    if (connect(socket_file_descriptor, (SocketAddress*) &server_address, sizeof(server_address)) < 0) {
        std::cerr << "Connection failed.\n";
        return EXIT_FAILURE;
    }

    std::vector<std::string> values {};
    char input_buffer[BUFFER_SIZE];
    while (true) { // 
        const ssize_t read_size {read(socket_file_descriptor, input_buffer, BUFFER_SIZE)};
        if (read_size <= 0) {
            std::cerr << "Error reading data.\n";
            sleep(10); // I don't know, do something here.
            continue;
        }
        input_buffer[read_size] = '\0';
        values.clear();
        std::stringstream input_stream {input_buffer};
        while (input_stream.good()) {
            std::string value {};
            std::getline(input_stream, value, INPUT_DELIMITER);
            values.push_back(value);
        }
        const std::size_t count {values.size()};
        if (count > WARNING_SIZE) {
            std::cerr << "There are a lot of things here.\n";
        } else if (count == REQUIRED_VALUES) {
            for (const std::string &value : values) {
                std::cout << value << "\n";
            }
            std::cout << "Read and publish the values.";
        } else {
            std::cerr << "Received the wrong values.\n";
        }
        sleep(1); // Should spin once.
    }
    return EXIT_SUCCESS;
}
