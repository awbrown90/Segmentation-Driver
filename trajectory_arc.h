#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "iostream"
#include <math.h>
#include <vector> 

using namespace std;
using namespace cv;

struct Pos
{
	int y, x;

	Pos(int set_y, int set_x)
		: y(set_y), x(set_x)
	{}
};

class TrajectoryArc {
public:
	TrajectoryArc();

	float steering_theta = 0.0;
	float horizon = 0.0;
	float height = 0.0;
	float width = 0.0;
	bool _params_initialized = false;
	float alpha = 0.2;

	Mat cloned_image_;

	/* Functions */
	int is_red_pixel(cv::Mat image, int x, int y);
	int is_cone_or_lane(cv::Mat image, int x, int y);
	void EraseCone(cv::Mat& image, int y, int x);
	std::vector<Pos> cone_pos(cv::Mat image);
	void cone_lines(cv::Mat image);
	vector<float> softmax(vector<float> x);
	int center_trajectories(cv::Mat image, int r, bool visualize);
	int right_trajectories(cv::Mat image, int R, int r, int LTolerance, bool visualize);
	int left_trajectories(cv::Mat image, int R, int r, int LTolerance, bool visualize);
	int center_trajectories_count(cv::Mat image, int r);
	int right_trajectories_count(cv::Mat image, int R, int r);
	int left_trajectories_count(cv::Mat image, int R, int r);
	float dot(vector<float> v_a, vector<float> v_b);
	float process(cv::Mat image, bool visualize);
};
