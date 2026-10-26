#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>

#include "drone.h"
#include "flightcontroller.h"

#define PORT        80
#define BUFFER_SIZE 1024

bool stop = false;
//stop the program when ctr+c is received
void handle_signal(int signal) {
    // Find out which signal we're handling
    switch (signal) {
        case SIGINT://key 3
            stop = true;
            break;
    }
}

void hook_signal() {
    struct sigaction sa;

    // Setup the sighub handler
    sa.sa_handler = &handle_signal;
    // Restart the system call, if at all possible
    sa.sa_flags = SA_RESTART;
    if (sigaction(SIGINT, &sa, NULL) == -1) {
        printf("Failed to capture sigint 1\n");
    }
}

//g++ main.cpp motor.cpp utils.cpp -lpthread -lpigpio -0 main
//cmake -S . -B build
int main (int argc, char **argv ) {
    hook_signal();

    //initialize gpio
    gpioInitialise();
    log_set_filename("drone_log", 0, 0);

    int mode = MODE_INDOOR;
    if ((argc >= 3) && (strcmp(argv[1], "-m") == 0)) {
        if (strcmp(argv[2], "indoor") == 0) {
            mode = MODE_INDOOR;
        }
        else if (strcmp(argv[2], "outdoor") == 0) {
            mode = MODE_OUTDOOR;
        }
        else if (strcmp(argv[2], "training") == 0) {
            mode = MODE_TRAINING;
        }
        else {
            cout << "Unknown mode " << argv[1] << endl;
        }
    }

    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    socklen_t addrlen = sizeof(address);
    char buffer[BUFFER_SIZE] = {0};
    char okResponse[] = "HTTP/1.1 200 OK\nContent-Type: text/plain\nContent-Length: %d\r\n\r\n%s";
    char badRequest[] = "HTTP/1.1 400 Bad Request\nContent-Type: text/plain\nContent-Length: 0\r\n\r\n";

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 1) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    printf("Server listening on port %d\n", PORT);

    FlightController *flightController = new FlightController(mode);
    while (!stop) {
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) {
            perror("accept");
            exit(EXIT_FAILURE);
        }
        // GET / HTTP/1.1 CRLF
        // Host: localhost:8080 CRLF
        // User-Agent: curl/7.19.7 blar blar CRLF
        // Accept: */* CRLF
        // Content-Length: ?? CRLF
        // ...: ... CRLF
        // CRLF
        // octets
        char szBuf[1024];
        char *response = badRequest;
        int got = read(new_socket, buffer, BUFFER_SIZE);
        if (got > 0) {
            buffer[got] = '\0';
            char *ptr = strstr(buffer, "Content-Length");
            if (ptr) {
                //cmd=ping or config=
                char *str = ptr + 15;
                Utils::trim(str);
                char *sep = strstr(str, "=");
                if (sep) {
                    char szResponse[1024];
                    szResponse[0] = '\0';
                    char *name = str;
                    char *value= sep + 1;
                    *sep = '\0';
                    if (strcmp(name, "cmd") == 0) {
                        flightController->onCommand(value);
                        if (strcmp(name, "ping") == 0) {
                            char *status = flightController->getStatus();
                            strcpy (szResponse, status);
                        }
                        else if (strcmp(value, "shutdown") == 0) {
                            system("sudo shutdown now");
                         }
                    }
                    else if (strcmp(name, "config") == 0) {
                        flightController->onConfig(value);
                    }
                    sprintf(szBuf, okResponse, szResponse);
                    response = szBuf;
                }
            }
            write(new_socket, response, strlen(response));
        }
        close(new_socket);
    }
    close (server_fd);

    flightController->cmdStop(true, true, true);
    Utils::delayMs(1000);
    delete flightController;
    
    gpioTerminate();
    //close the log file
    log_enabled(false);

    return 0;
}


