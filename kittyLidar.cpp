#include "lidarLite.h"
#include <time.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>
#include "LED.h"

static int TOTAL_CALIBRATION_MEASUREMENTS = 10;
static int TRIGGER_PIN = 7;
static int WAIT_TIMEOUT = 40;

void initWiring();
int findLidarMaxDistance(int fd);

int main(int argc,char *argv[]) {

    initWiring();
    LED lidarIndicatorLED(TRIGGER_PIN);

    int fd, res, del, status, waitTimeout = WAIT_TIMEOUT;

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
                childId = fork(); // spawn a child process
            } else {
                lidarIndicatorLED.off();
            }

            if (childId == 0)
            {
                exit(system("sudo raspistill -o /home/pi/photos/photo3.jpg")); // child process
            } else if (childId > 0) {
                waitTimeout % 2 == 0 ? lidarIndicatorLED.on() : lidarIndicatorLED.off();
                if (waitTimeout < 0) {
                    wait(&status);
                    printf("Child exit code: %d\n", WEXITSTATUS(status));
                    childId = -1;
                    waitTimeout = WAIT_TIMEOUT;
                    lidarIndicatorLED.off();
                }
                waitTimeout--;
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
