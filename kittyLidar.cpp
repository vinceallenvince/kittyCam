#include <time.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>
#include <string.h>
#include "lidarLite.h"
#include "LED.h"

using namespace::std;

typedef int (*fn)(LED&);

struct ChildProcess {
    char description[128];
    fn func;
};

static int TOTAL_CALIBRATION_MEASUREMENTS = 10;
static int TRIGGER_PIN = 7;
static int TOTAL_CHILD_PROCESSES = 2;

/**
 * Initialize wiringPi and configure the gpio settings.
 */
void configureWiring();

/**
 * Finds the maximum distance the lidar sensor can detect.
 * @param fd The id of the lidar device.
 */
int findLidarMaxDistance(int fd);

/**
 * Make a call to the system camera to capture an image.
 */
int imageCapture(LED &ledIndicator);

/**
 * Launches a child process and blinks a status LED until complete.
 * @param processIndex The index in an array of function pointers.
 */
int launchChildProcess(int processIndex, ChildProcess *childProcesses, int totalChildProcesses, LED &ledIndicator);


int main(int argc,char *argv[]) {

    ChildProcess childProcesses[TOTAL_CHILD_PROCESSES];

    strcpy(childProcesses[0].description, "image capture");
    childProcesses[0].func = imageCapture;

    //

    initWiring();
    LED lidarIndicatorLED(TRIGGER_PIN);

    int fd, res, del;

    // First arg is delay in ms (default is 100)
    if (argc > 1) {
        del = atoi(argv[1]);
    } else {
        del = 100;
    }
    
    fd = lidar_init(false);
   
    if (fd == -1) {
        printf("initialization error\n");
    } else {

        int maxDistance = findLidarMaxDistance(fd);
        printf("trigger threshold: %d \n", maxDistance);

        for (;;) {
            res = lidar_read(fd);
            if (res < maxDistance * 0.9)
            {
                lidarIndicatorLED.on();
                if (launchChildProcess(0, childProcesses, TOTAL_CHILD_PROCESSES, lidarIndicatorLED) == 0) {
                    printf("PROCESS: Image capture successful.\n");
                }
            } else {
                lidarIndicatorLED.off();
            }

            delay(del);
        }
    }

    return 0;
}

void configureWiring() {
    wiringPiSetup();
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
}

int findLidarMaxDistance(int fd) {
    printf("finding max distance\n");
    int res, initialMeasurements = 0;

    for (int i = 0; i < TOTAL_CALIBRATION_MEASUREMENTS; ++i) {
        res = lidar_read(fd);
        printf("%3.0d cm \n", res);
        initialMeasurements += res;
        delay(250);
    }

    return initialMeasurements / TOTAL_CALIBRATION_MEASUREMENTS;
}

int imageCapture(LED &ledIndicator) { // TODO: remove LED param
    time_t when;
    time(&when);
    printf("Image capture process started at %s.", ctime(&when));
    int status = system("sudo raspistill -o /home/pi/photos/photo3.jpg"); // capture image
    printf("Image capture process ended at %s.", ctime(&when));
    return status;
}

int launchChildProcess(int processIndex, ChildProcess *childProcesses, int totalChildProcesses, LED &ledIndicator) {

    int status, waitTimeout = 0;
    pid_t childID, endID;
    time_t when;

    if ((childID = fork()) == -1) { // Start child process.
        perror("fork error");
        exit(EXIT_FAILURE);
    }
    else if (childID == 0) {    // The child process.
        exit(childProcesses[processIndex].func(ledIndicator));
    }
    else // The parent process.
    {
        char *descr = childProcesses[processIndex].description;
        time(&when);
        printf("Parent process started at %s", ctime(&when));

        for(;;) { // Wait for child process to terminate.
            endID = waitpid(childID, &status, WNOHANG|WUNTRACED);

            if (endID == -1) // Error calling waitpid.
            {
                perror("waitpid error");
                exit(EXIT_FAILURE);
            }
            else if (endID == 0) // Child still running.
            {
                time(&when);
                printf("Waiting for %s at %s", descr, ctime(&when));
                waitTimeout++ % 2 == 0 ? ledIndicator.on() : ledIndicator.off();
                sleep(1);
            }
            else if (endID == childID) // Child ended.
            {
                if (WIFEXITED(status))
                    printf("%s ended normally. status: %d at %s\n", descr, status, ctime(&when));
                else if (WIFSIGNALED(status))
                    printf("%s ended because of an uncaught signal at %s.\n", descr, ctime(&when));
                else if (WIFSTOPPED(status))
                    printf("%s process has stopped at %s.\n", descr, ctime(&when));
                return status;
            }
        }
    }
    return 1;
}

