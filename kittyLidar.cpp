#include <time.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>
#include <string.h>
#include "lidarLite.h"
#include "LED.h"

static int TOTAL_CALIBRATION_MEASUREMENTS = 10;
static int TRIGGER_PIN = 7;

void initWiring();
int findLidarMaxDistance(int fd);
int imageCapture(LED &ledIndicator);

int main(int argc,char *argv[]) {

    initWiring();
    LED lidarIndicatorLED(TRIGGER_PIN);

    int fd, res, del;

    pid_t childId = -1;

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
            if (res < maxDistance * 0.9 && childId == -1)
            {
                lidarIndicatorLED.on();
                if (imageCapture(lidarIndicatorLED) == 0) {
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

void initWiring() {
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

int imageCapture(LED &ledIndicator) {

    int status, waitTimeout = 0;
    pid_t childID, endID;
    time_t when;

    if ((childID = fork()) == -1) {     /* Start a child process.      */
        perror("fork error");
        exit(EXIT_FAILURE);
    }
    else if (childID == 0) {            /* This is the child.          */
        time(&when);
        printf("Image capture process started at %s", ctime(&when));
        exit(system("sudo raspistill -o /home/pi/photos/photo3.jpg")); // capture image
    }
    else {                              /* This is the parent.         */
        time(&when);
        printf("Parent process started at %s", ctime(&when));

        /* Wait for child process to terminate.           */
        for(;;) {
            endID = waitpid(childID, &status, WNOHANG|WUNTRACED);
            if (endID == -1) {            /* error calling waitpid       */
                perror("waitpid error");
                exit(EXIT_FAILURE);
            }
            else if (endID == 0) {        /* child still running         */
                time(&when);
                printf("Waiting for image capture at %s", ctime(&when));
                waitTimeout++ % 2 == 0 ? ledIndicator.on() : ledIndicator.off();
                //waitTimeout++;
                sleep(1);
            }
            else if (endID == childID) {  /* child ended                 */
                if (WIFEXITED(status))
                    printf("Image capture ended normally at %swith status: %d\n", ctime(&when), status);
                else if (WIFSIGNALED(status))
                    printf("Image capture ended because of an uncaught signal.\n");
                else if (WIFSTOPPED(status))
                    printf("Image capture process has stopped.\n");
                return status;
            }
        }
    }
    return 1;
}

