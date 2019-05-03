/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "render/render.h"
#include "trajectory_arc.h"
//#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
//#include "processPointClouds.cpp"

class Player
{
public:

	float x_pos, y_pos;
	float angle, steering;
	float velocity;
	float Lf;

	Player()
		: x_pos(0), y_pos(0), angle(0), steering(0), velocity(0.2), Lf(2)
	{}

	Player(float xSet, float ySet, float angleSet)
		: x_pos(xSet), y_pos(ySet), angle(angleSet), steering(0), velocity(0.2), Lf(2)
	{}

	void Forward()
	{
		x_pos+=velocity*cos(angle);
		y_pos+=velocity*sin(angle);
		angle += velocity * steering / Lf;
	}
	void Back()
	{
		x_pos-=0.2*cos(angle);
		y_pos-=0.2*sin(angle);
		angle += velocity * steering / Lf;
	}
	void setSteeringAngle(float setSteering)
	{
		steering = setSteering;
	}

	void TurnRight()
	{
		steering -= 0.01;
		while (steering < -M_PI) { steering += 2 * M_PI; }
		while (steering >  M_PI) { steering -= 2 * M_PI; }
	}
	void TurnLeft()
	{
		steering += 0.01;
		while (steering < -M_PI) { steering += 2 * M_PI; }
		while (steering > M_PI)  { steering -= 2 * M_PI; }
	}
};

Player player(0,0,M_PI/2);
TrajectoryArc planner;

void setCamera(Vect3 pos, double angle, pcl::visualization::PCLVisualizer& viewer)
{
	double viewDist = 10;
	viewer.setCameraPosition(pos.x, pos.y, pos.z, pos.x + viewDist*cos(angle), pos.y + viewDist*sin(angle), 0, 0, 0, 1);
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer* viewer = static_cast<pcl::visualization::PCLVisualizer*>(viewer_void);
  	if (event.getKeySym () == "Up" && event.keyDown ())
  	{
    	player.Forward();
  	}
  	else if(event.getKeySym () == "Down" && event.keyDown ())
  	{
  		player.Back();
  	}
	else if (event.getKeySym() == "Right" && event.keyDown())
	{
		player.TurnRight();
	}
	else if (event.getKeySym() == "Left" && event.keyDown())
	{
		player.TurnLeft();
	}
  	else if(event.getKeySym () == "t" && event.keyDown ())
  	{
  		saveScreenshot(viewer,"screenshot.png");

		Mat image = imread("screenshot.png");

		player.setSteeringAngle(-planner.process(image, true));

		namedWindow("Display Image", WINDOW_AUTOSIZE);
		imshow("Display Image", planner.cloned_image_);

		waitKey(0);
  	}
	else if (event.getKeySym() == "space" && event.keyDown())
	{
		saveScreenshot(viewer, "screenshot.png");

		Mat image = imread("screenshot.png");

		player.setSteeringAngle(-planner.process(image, true));

		namedWindow("Display Image", WINDOW_AUTOSIZE);
		imshow("Display Image", planner.cloned_image_);

		player.Forward();

	}


	setCamera(Vect3(player.x_pos, player.y_pos, 1.5), player.angle, *viewer);
	
}



int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;
	
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	
    viewer->setBackgroundColor (0, 0, 0);
	
	setCamera(Vect3(player.x_pos, player.y_pos, 1.5), player.angle, *viewer);
    renderHighway(viewer);
	

    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
    
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
    

}