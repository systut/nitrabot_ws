#include <iostream>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <gpiod.h>



#define GREEN_BTN_PIN 27
#define RED_BTN_PIN 17
#define SW_BTN_PIN_LEFT 22
#define SW_BTN_PIN_RIGHT 24

using std::cout; using std::endl;

enum PathFinderDirection
{
    FORWARD,
    BACKWARD
};

pid_t runCommand(char * argv[], std::vector<int>& pids) {
    pid_t c_pid = fork();

    if (c_pid == -1) {
        perror("fork");
        exit(EXIT_FAILURE);
    } else if (c_pid > 0) {
        cout << "printed from parent process " << getpid() << endl;
        wait(nullptr);
    } else {
        cout << "printed from child process " << getpid() << endl;
        pids.push_back(getpid());
        execv("/usr/bin/gnome-terminal", argv);
    }
}

void startMode3(std::vector<int>& pids, PathFinderDirection& direction, int mode) {

    int FORK_NUM = 3; 

    std::vector<int> children;

    children.reserve(FORK_NUM);
    

    if (mode !=  2)
    {
        char * cartographerArgument[] = {
            "gnome-terminal", "--" , "bash", "-c", "cd /home/ubuntu/catkin_ws; bash start_pure_localization.sh; exec bash", NULL
        };
        
        children[0] = runCommand(cartographerArgument, pids);

        char * navigationArgument[] = {
            "gnome-terminal", "--" , "bash", "-c", "cd /home/ubuntu/Lumi-Robotics; bash start_navigation_carto.sh; exec bash", NULL
        };
        
        children[1] = runCommand(navigationArgument, pids);
    }

    if (direction == FORWARD) {
        char * finderArgument[] = {
            "gnome-terminal", "--" , "bash", "-c", "cd /home/ubuntu/Lumi-Robotics; bash start_finder_node_forward.sh; exec bash", NULL
        };
        
        children[2] = runCommand(finderArgument, pids);
        direction = BACKWARD;
    }
    else {
        char * finderArgument[] = {
            "gnome-terminal", "--" , "bash", "-c", "cd /home/ubuntu/Lumi-Robotics; bash start_finder_node_backward.sh; exec bash", NULL
        };
        
        children[2] = runCommand(finderArgument, pids);
        direction = FORWARD;
    }    
}

void killMode3(std::vector<int>& pids) {
    for (auto it = pids.begin(); it != pids.end(); ++it)
    {
        kill(*it, SIGTERM);    
    } 
    pids.clear();
}

int main (int argc, char **argv)
{	
	const char *chipname = "gpiochip0";
	printf("SYSTEM IS SETTING UP! \n");
	
	struct gpiod_chip *chip;
  	struct gpiod_line *redBTN;
  	struct gpiod_line *greenBTN;
  	struct gpiod_line *swLEFT;
  	struct gpiod_line *swRIGHT;
  	
  	// Open GPIO chip
	chip = gpiod_chip_open_by_name(chipname);
	
	printf("OPENNED CHIP! \n");
    
	// Open GPIO lines
	redBTN = gpiod_chip_get_line(chip, RED_BTN_PIN);
	greenBTN = gpiod_chip_get_line(chip, GREEN_BTN_PIN);
	swLEFT = gpiod_chip_get_line(chip, SW_BTN_PIN_LEFT);
	swRIGHT = gpiod_chip_get_line(chip, SW_BTN_PIN_RIGHT);
	
	printf("OPENNED GPIO lines! \n");

	// Open switch line for input
	gpiod_line_request_input(redBTN, "example1");
	gpiod_line_request_input(greenBTN, "example2");
	gpiod_line_request_input(swLEFT, "example3");
	gpiod_line_request_input(swRIGHT, "example4");
    std::vector<int> pids;
    
    PathFinderDirection direction = FORWARD;
    
    int mode = 3;

	printf("SYSTEM READY! \n");
	while (1) {
	    if (gpiod_line_get_value(redBTN)==1)
	    {
	    	printf("RED BTN clicked! \n");
            killMode3(pids);
            mode = 2;
	    }
	    else if (gpiod_line_get_value(greenBTN)==1)
	    {
	    	printf("GREEN BTN clicked! \n");
            startMode3(pids, direction,mode);
            mode = 3;
	    }
	    else if (gpiod_line_get_value(swLEFT)==1)
	    {
	    	//printf("SW is on the left! \n");
	    }
	    else if (gpiod_line_get_value(swRIGHT)==1)
	    {
	    	//printf("SW is on the right! \n");
	    }
	    else 
	    {
	    	printf("Waiting for input... \n");
	    }
		
	    std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	// Release lines and chip
	gpiod_line_release(redBTN);
	gpiod_line_release(greenBTN);
	gpiod_line_release(swLEFT);
	gpiod_line_release(swRIGHT);
	gpiod_chip_close(chip);
	return 0;
}

