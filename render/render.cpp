/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#include "render.h"

void renderArc(Vect3 prev, Vect3 start, double radius, double range, int res, int direction, std::vector<Vect3>& path)
{
	double mag = sqrt((start.x - prev.x)*(start.x - prev.x) + (start.y - prev.y)*(start.y - prev.y));
	Vect3 rad( (radius/mag)* -1 * direction*(start.y - prev.y), (radius/mag)*direction*(start.x - prev.x),0);
	double g = start.x + rad.x;
	double f = start.y + rad.y;
	double theta = atan2(start.y - f, start.x - g);
	int seg = 0;
	while (seg <= res)
	{
		double x = radius * cos(theta) + g;
		double y = radius * sin(theta) + f;
		seg++;
		theta += direction * (range/res);
		path.push_back(Vect3(x, y, 0));
	}
}

void renderHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
	std::cout << "render highway" << std::endl;
	
	// units in meters
	//double roadLength = 50.0;
	double roadWidth = 3.0;
	//double roadHeight = 0.2;

	//viewer->addCube(-roadLength/2, roadLength/2, -roadWidth/2, roadWidth/2, -roadHeight, 0, 1, 0, 0, "highwayPavement");
    //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "highwayPavement"); 
    //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "highwayPavement");

	
    std::vector<Vect3> path {Vect3(0,9,0) };
	renderArc(Vect3(0, 9, 0), Vect3(0, 10, 0), 3, M_PI/4, 16, -1, path);
	renderArc(Vect3(0.879, 12.121, 0), Vect3(3.707, 14.95, 0), 4, 3*M_PI/4, 16, -1, path);
	renderArc(Vect3(13, 8, 0), Vect3(13, 7, 0), 5, M_PI / 2, 16, -1, path);
	renderArc(Vect3(4, 2, 0), Vect3(3, 2, 0), 3, M_PI / 2, 16, -1, path);
	path.push_back(Vect3(0, 8, 0));
	path.push_back(Vect3(0,9,0));

    Vect3 lastPoint = path[0];
	Vect3 lastVecPerpL = Vect3(-1.0, 0, 0);
	Vect3 lastVecPerpR = Vect3( 1.0, 0, 0);

    for(int i = 1; i < path.size(); i++)
    {
		pcl::PointCloud<pcl::PointXYZ>::Ptr seg(new pcl::PointCloud<pcl::PointXYZ>);
    	Vect3 vec = Vect3(path[i].x-lastPoint.x,path[i].y-lastPoint.y,0);
		float magVec = sqrt(vec.x * vec.x + vec.y * vec.y);
		Vect3 vecPerpL = Vect3(-vec.y/magVec, vec.x/magVec, vec.z/magVec);
		Vect3 vecPerpR = Vect3(-vecPerpL.x, -vecPerpL.y, vecPerpL.z);

		seg->points.push_back(pcl::PointXYZ(lastPoint.x + roadWidth/2 * lastVecPerpR.x, lastPoint.y + roadWidth/2 * lastVecPerpR.y, 0));
		seg->points.push_back(pcl::PointXYZ(lastPoint.x + roadWidth/2 * lastVecPerpL.x, lastPoint.y + roadWidth/2 * lastVecPerpL.y, 0));
		seg->points.push_back(pcl::PointXYZ(path[i].x + roadWidth/2 * vecPerpL.x, path[i].y + roadWidth/2 * vecPerpL.y, 0));
		seg->points.push_back(pcl::PointXYZ(path[i].x + roadWidth/2 * vecPerpR.x, path[i].y + roadWidth/2 * vecPerpR.y, 0));

		viewer->addPolygon<pcl::PointXYZ>(seg, "seg"+std::to_string(i));
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "seg" + std::to_string(i));
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "seg" + std::to_string(i));
		
		lastPoint = path[i];
		lastVecPerpL = vecPerpL;
		lastVecPerpR = vecPerpR;
    }
	
}

int countRays = 0;
void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

	for(pcl::PointXYZ point : cloud->points)
	{
		viewer->addLine(pcl::PointXYZ(origin.x, origin.y, origin.z), point,1,0,0,"ray"+std::to_string(countRays));
		countRays++;
	}
}

void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
	while(countRays)
	{
		countRays--;
		viewer->removeShape("ray"+std::to_string(countRays));
	}
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color)
{

  	viewer->addPointCloud<pcl::PointXYZ> (cloud, name);
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color)
{

	if(color.r==-1)
	{
		// Select color based off of cloud intensity
		pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud,"intensity");
  		viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
	}
	else
	{
		// Select color based off input value
		viewer->addPointCloud<pcl::PointXYZI> (cloud, name);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
	}

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

// Draw wire frame box with filled transparent color 
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color, float opacity)
{
	if(opacity > 1.0)
		opacity = 1.0;
	if(opacity < 0.0)
		opacity = 0.0;
	
	std::string cube = "box"+std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);
    
    std::string cubeFill = "boxFill"+std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color, float opacity)
{
	if(opacity > 1.0)
		opacity = 1.0;
	if(opacity < 0.0)
		opacity = 0.0;
	
	std::string cube = "box"+std::to_string(id);
    viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);
    
    std::string cubeFill = "boxFill"+std::to_string(id);
    viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}

void saveScreenshot(pcl::visualization::PCLVisualizer* viewer, const std::string &file)
{
	
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();

	// Create the image filter and PNG writer objects
	vtkSmartPointer<vtkRenderWindow> renderWindow = viewer->getRenderWindow();
	windowToImageFilter->SetInput(renderWindow);
	windowToImageFilter->Modified();
	writer->SetInputConnection(windowToImageFilter->GetOutputPort());
	writer->Modified();
	writer->SetFileName(file.c_str());
	writer->Write();

}







	