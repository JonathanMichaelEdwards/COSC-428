#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include "pxcsensemanager.h"
#include <opencv2/aruco.hpp>
#include <time.h>
#include <ctime>
#include <future>
#include <queue>
#include <iostream>
#include <limits.h>

using namespace cv;

std::string videofiles[] = {
	"C:\\Users\\james\\OneDrive\\Documents\\Uni\\Year 4\\COSC428 Computer Vision\\Project\\Video Resources\\VID_20160407_141251.mp4",
	"C:\\Users\\james\\OneDrive\\Documents\\Uni\\Year 4\\COSC428 Computer Vision\\Project\\Video Resources\\VID_20160407_141320.mp4",
	"C:\\Users\\james\\OneDrive\\Documents\\Uni\\Year 4\\COSC428 Computer Vision\\Project\\Video Resources\\VID_20160407_141530.mp4",
	"C:\\Users\\james\\OneDrive\\Documents\\Uni\\Year 4\\COSC428 Computer Vision\\Project\\Video Resources\\VID_20160407_141631.mp4"};

enum DetctionType
{
	Marker = 0,
	NFT = 1
};

struct Stats
{
	int keypoints;
	int matches;
	int inliers;
	float ratio;
};

struct NFTResult
{
	Mat ResultantImage;
	Mat Homography;
	Stats Stats;
};

const double ransac_thresh = 2.5f; // RANSAC inlier threshold
const double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
const double movement_cutoff_nft = 100;

std::vector <std::string> split(const std::string& str, const std::string& delimiter = " ") {
	std::vector <std::string> tokens;

	std::string::size_type lastPos = 0;
	std::string::size_type pos = str.find(delimiter, lastPos);

	while (std::string::npos != pos) {
		// Found a token, add it to the vector.
		// std::cout << str.substr(lastPos, pos - lastPos) << std::endl;
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		lastPos = pos + delimiter.size();
		pos = str.find(delimiter, lastPos);
	}

	tokens.push_back(str.substr(lastPos, str.size() - lastPos));
	return tokens;
}

Mat adjustImageBeforeDetection(Mat inuptImage)
{
	Mat gray, threshold;
	cvtColor(inuptImage, gray, COLOR_BGR2GRAY);
	//GaussianBlur(gray, blurred, Size(7, 7), 0);
	//cv::threshold(gray, threshold, 210, 255, CV_THRESH_BINARY);
	//Canny(blurred, edged, 50, 150);
	return gray;
}

class StreamManager : public PXCSenseManager::Handler
{
public:
	StreamManager()
	{
		psm = PXCSenseManager::CreateInstance();
	};
	virtual ~StreamManager()
	{
	};
	PXCSenseManager* GetSenseManager(PXCCapture::DeviceModel modelType);
protected:
	PXCSenseManager *psm;
	pxcStatus PXCAPI OnConnect(PXCCapture::Device *device, pxcBool connected) override;
};

pxcStatus StreamManager::OnConnect(PXCCapture::Device *device, pxcBool connected)
{
	if (connected)
	{
		PXCVideoModule::DataDesc desc = {};
		desc.streams.color.frameRate.min = desc.streams.color.frameRate.max = 30;
		desc.streams.color.sizeMax.height = desc.streams.color.sizeMin.height = 480;
		desc.streams.color.sizeMax.width = desc.streams.color.sizeMin.width = 640;
		desc.streams.color.options = PXCCapture::Device::STREAM_OPTION_UNRECTIFIED;

		desc.streams.depth.frameRate.min = desc.streams.depth.frameRate.max = 30;
		desc.streams.depth.sizeMax.height = desc.streams.depth.sizeMin.height = 468;
		desc.streams.depth.sizeMax.width = desc.streams.depth.sizeMin.width = 628;
		desc.streams.depth.options = PXCCapture::Device::STREAM_OPTION_ANY;

//		desc.streams.ir.frameRate.min = desc.streams.ir.frameRate.max = 30; // Uncomment if IR is desired.
//		desc.streams.depth.sizeMax.height = desc.streams.depth.sizeMin.height = 480;
//		desc.streams.depth.sizeMax.width = desc.streams.depth.sizeMin.width = 640;
//		desc.streams.depth.options = PXCCapture::Device::STREAM_OPTION_ANY;
		psm->EnableStreams(&desc);
	}
	return PXC_STATUS_NO_ERROR;
}


PXCSenseManager* StreamManager::GetSenseManager(PXCCapture::DeviceModel modelType)
{
	PXCCapture::DeviceInfo dinfo;
	dinfo.model = modelType;
	//psm->QueryCaptureManager()->FilterByDeviceInfo(&dinfo);
	auto init = psm->Init(this);
//	PXCCapture::Device *d = psm->QueryCaptureManager()->QueryDevice();
//	d->ResetProperties(PXCCapture::STREAM_TYPE_ANY);
	return psm;
}

void ConvertPXCImageToOpenCVMat(PXCImage *inImg, Mat *outImg, PXCImage::PixelFormat type) {
	int cvDataType = 0;
	int cvDataWidth = 0;


	PXCImage::ImageData data;
	inImg->AcquireAccess(PXCImage::ACCESS_READ, type, &data);
	PXCImage::ImageInfo imgInfo = inImg->QueryInfo();

	switch (data.format) {
		/* STREAM_TYPE_COLOR */
	case PXCImage::PIXEL_FORMAT_YUY2: /* YUY2 image  */
	case PXCImage::PIXEL_FORMAT_NV12: /* NV12 image */
	case PXCImage::PIXEL_FORMAT_ANY:
		throw(0); // Not implemented
	case PXCImage::PIXEL_FORMAT_RGB32: /* BGRA layout on a little-endian machine */
		cvDataType = CV_8UC4;
		cvDataWidth = 4;
		break;
	case PXCImage::PIXEL_FORMAT_RGB24: /* BGR layout on a little-endian machine */
		cvDataType = CV_8UC3;
		cvDataWidth = 3;
		break;
	case PXCImage::PIXEL_FORMAT_Y8:  /* 8-Bit Gray Image, or IR 8-bit */
		cvDataType = CV_8U;
		cvDataWidth = 1;
		break;

		/* STREAM_TYPE_DEPTH */
	case PXCImage::PIXEL_FORMAT_DEPTH: /* 16-bit unsigned integer with precision mm. */
	case PXCImage::PIXEL_FORMAT_DEPTH_RAW: /* 16-bit unsigned integer with device specific precision (call device->QueryDepthUnit()) */
		cvDataType = CV_16U;
		cvDataWidth = 2;
		break;
	case PXCImage::PIXEL_FORMAT_DEPTH_F32: /* 32-bit float-point with precision mm. */
		cvDataType = CV_32F;
		cvDataWidth = 4;
		break;

		/* STREAM_TYPE_IR */
	case PXCImage::PIXEL_FORMAT_Y16:          /* 16-Bit Gray Image */
		cvDataType = CV_16U;
		cvDataWidth = 2;
		break;
	case PXCImage::PIXEL_FORMAT_Y8_IR_RELATIVE:    /* Relative IR Image */
		cvDataType = CV_8U;
		cvDataWidth = 1;
		break;
	default: break;
	}


	// suppose that no other planes
	if (data.planes[1] != nullptr) throw(0); // not implemented
											 // suppose that no sub pixel padding needed
	if (data.pitches[0] % cvDataWidth != 0) throw(0); // not implemented

	outImg->create(imgInfo.height, data.pitches[0] / cvDataWidth, cvDataType);

	memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE));

	inImg->ReleaseAccess(&data);
}

Point2f centreOfRectangle(Point2f corner1, Point2f corner2)
{
	return Point2f((corner1.x + corner2.x)/2, (corner1.y + corner2.y) / 2);
}

void RunTestOnVidoes()
{
		for (int i = 0; i < sizeof(videofiles) / sizeof(*videofiles); i++) {
			VideoCapture inputVideo;
			//inputVideo.set(CV_CAP_PROP_FOURCC, CV_FOURCC('H', '2', '6', '4'));
			inputVideo.open(0);
			if (!inputVideo.isOpened()) {
				std::cerr << "Failed to open video file!\n" << std::endl;
				system("Pause");
				exit(1);
			}
			Ptr<aruco::Dictionary> dictionary = getPredefinedDictionary(aruco::DICT_6X6_250);
			Mat image, imageCopy;
			float frames = 0, detected_frames = 0;
			clock_t start;
			double duration;
			start = clock();
			int j = 0;
			while (waitKey(1) < 0 && inputVideo.read(image)) {
				if (j % 3 != 0) {
					j++;
					continue;
				}
				j++;
				
				imageCopy = adjustImageBeforeDetection(image);
	
				std::vector<int> ids;
				std::vector<std::vector<Point2f>> rejected;
				std::vector<std::vector<Point2f>> corners;
				detectMarkers(imageCopy, dictionary, corners, ids, aruco::DetectorParameters::create(), rejected);
	
				// if at least one marker detected
				if (ids.size() > 0) {
					detected_frames++;
					aruco::drawDetectedMarkers(imageCopy, corners, ids);
					Point2f centre = centreOfRectangle(corners[0][0], corners[0][2]);
					Point2f imageCentre = Point2f(image.cols / 2, image.rows / 2);
					Point2f resutlt = imageCentre - centre;
					std::cout << "Centre of marker is at: (" << centre.x << "," << centre.y << ")";
				}
	
	//			if (rejected.size() > 0)
	//				aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(255, 255, 0));
				frames++;
				cv::imshow("out", imageCopy);
			}
			std::cout << "\nTime Taken:\t" << (clock() - start) / static_cast<double>(CLOCKS_PER_SEC);
			std::cout << "\nPercentage of frames detected:\t" << (detected_frames / frames) << "\n";
		}
		system("pause");
}

class NaturalFeatureTracker
{
// Mostly taken from http://docs.opencv.org/3.0-beta/doc/tutorials/features2d/akaze_tracking/akaze_tracking.html
public:
	NaturalFeatureTracker(Mat image)
	{
		image.copyTo(reference_image);
		ORBDetection(reference_image, reference_key_points, reference_descriptors);
		matcher = FlannBasedMatcher(new flann::LshIndexParams(6, 12, 1), new flann::SearchParams(100));
	};
	NFTResult process(const Mat frame);
protected:
	Mat reference_image, reference_descriptors;
	std::vector<KeyPoint> reference_key_points;
	static void ORBDetection(Mat img, std::vector<KeyPoint>& key_points, Mat& descriptors);
	FlannBasedMatcher matcher;
};

void NaturalFeatureTracker::ORBDetection(Mat img, std::vector<KeyPoint>& key_points, Mat& descriptors)
{
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	Ptr<FeatureDetector> detector = ORB::create();
	detector->detect(gray, key_points);
	detector->compute(gray, key_points, descriptors);
}

NFTResult NaturalFeatureTracker::process(const Mat frame)
{
	NFTResult result;
	std::vector<KeyPoint> kp;
	Mat desc;
	ORBDetection(frame, kp, desc);
	result.Stats.keypoints = static_cast<int>(kp.size());

	if (kp.size() == 0)
	{
		return result;
	}
	std::vector<std::vector<DMatch> > matches;
	std::vector<KeyPoint> matched1, matched2;
	std::vector<Point2f> points_matched1, points_matched2;
	matcher.knnMatch(reference_descriptors, desc, matches, 2);
	for (unsigned i = 0; i < matches.size(); i++) {
		if (matches[i].size() < 2) continue;
		if (matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
			matched1.push_back(reference_key_points[matches[i][0].queryIdx]);
			points_matched1.push_back(reference_key_points[matches[i][0].queryIdx].pt);
			matched2.push_back(kp[matches[i][0].trainIdx]);
			points_matched2.push_back(kp[matches[i][0].trainIdx].pt);
		}
	}
	result.Stats.matches = (int)matched1.size();


	Mat inlier_mask, homography_in_function;
	std::vector<KeyPoint> inliers1, inliers2;
	std::vector<DMatch> inlier_matches;
	if (matched1.size() >= 4) {
		homography_in_function = findHomography(points_matched1, points_matched2,
			RANSAC, ransac_thresh, inlier_mask);
	}
	homography_in_function.copyTo(result.Homography);
	if (matched1.size() < 4 || homography_in_function.empty()) {
		Mat res;
		result.Stats.inliers = 0;
		result.Stats.ratio = 0;
		result.ResultantImage = res;
		return result;
	}

	for (unsigned i = 0; i < matched1.size(); i++) {
		if (inlier_mask.at<uchar>(i)) {
			int new_i = static_cast<int>(inliers1.size());
			inliers1.push_back(matched1[i]);
			inliers2.push_back(matched2[i]);
			inlier_matches.push_back(DMatch(new_i, new_i, 0));
		}
	}
	result.Stats.inliers = (int)inliers1.size();
	result.Stats.ratio = result.Stats.inliers * 1.0 / result.Stats.matches;

	Mat res;
	drawMatches(reference_image, inliers1, frame, inliers2,
		inlier_matches, res,
		Scalar(255, 0, 0), Scalar(255, 0, 0));
	result.ResultantImage = res;
	return result;
}

void cameraPoseFromHomography(const Mat& H, Mat& pose)
{
	pose = Mat::eye(3, 4, CV_64FC1); //3x4 matrix
	float norm1 = (float)norm(H.col(0));
	float norm2 = (float)norm(H.col(1));
	float tnorm = (norm1 + norm2) / 2.0f;

	Mat v1 = H.col(0);
	Mat v2 = pose.col(0);

	cv::normalize(v1, v2); // Normalize the rotation

	v1 = H.col(1);
	v2 = pose.col(1);

	cv::normalize(v1, v2);

	v1 = pose.col(0);
	v2 = pose.col(1);

	Mat v3 = v1.cross(v2);  //Computes the cross-product of v1 and v2
	Mat c2 = pose.col(2);
	v3.copyTo(c2);

	pose.col(3) = H.col(2) / tnorm; //vector t [R|t]
}

NFTResult CallProcess(NaturalFeatureTracker nft, Mat image)
{
	return nft.process(image);
}

int countMarkers(std::queue<DetctionType> queue, DetctionType marker)
{
	int count = 0;
	for (int i = 0; i < queue.size(); i++)
	{
		if (queue._Get_container().at(i) == marker)
		{
			count++;
		}
	}
	return count;
}

int main() {
	std::string refernce_image_path = "C:\\Users\\james\\OneDrive\\Documents\\Uni\\Year 4\\COSC428 Computer Vision\\Project\\Solution\\MarkerDetection\\MarkerDetection\\reference_image.jpg"; // Change for reference image Todo make this a command line argument
	//VideoCapture video;
	Stats stats;
	PXCImage *depth, *colour, *ir;
	PXCImage::ImageData data_depth;
	IplImage *depth_image;
	Mat image, imageCopy, adjusted, depth_mat, resulting_image, homography, reference_image, ir_image;
	pxcStatus sts;
	PXCCapture::Sample *sample;
	Point2f markerCentre, imageCentre, offset;
	int offset_limit = 50;
	depth_image = cvCreateImage(Size(640, 480), 8, 1);

	//VideoWriter video("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(640, 480), true);

	// Setup cmaera
	 StreamManager manager;
	 PXCSenseManager *psm = manager.GetSenseManager(PXCCapture::DeviceModel::DEVICE_MODEL_R200); // Change this for different camera types
//	PXCSenseManager *psm = PXCSenseManager::CreateInstance(); // This should be used instead of the line above if using the F200. For some reason it's very difficult to make it work for both of them
//	psm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480);
//	psm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 640, 480);
//	psm->EnableStream(PXCCapture::STREAM_TYPE_IR, 640, 480);
//	auto foo = psm->Init();

	// Set up natural feature tracker and other relevant stuff
	// Get the 3rd image as a reference image (camera takes a while to warm up)
	for (int i = 0; i < 10; i++)
	{
		psm->AcquireFrame();
		sample = psm->QuerySample();
		if (i == 9)
		{
			colour = sample->color;
			ConvertPXCImageToOpenCVMat(colour, &reference_image, PXCImage::PIXEL_FORMAT_RGB32);
			imwrite("referencer_image.jpg", reference_image);
		}
		psm->ReleaseFrame();
	}

	NaturalFeatureTracker tracker(reference_image);
	double data[] = { 1.0,1.0,1.0 };
	Mat directional_vector = Mat(3, 1, DataType<double>::type, data);
	NFTResult nftresult;
	bool nft_used;

	Ptr<aruco::Dictionary> dictionary = getPredefinedDictionary(aruco::DICT_6X6_250); // Change this if you're using a different marker 

	std::queue<DetctionType> detection_type_history;
	std::queue<float> depth_history;
	int frames = 0;
	int detected = 0;
	int total_matches = 0;
	while (waitKey(1)) {
		// Get frame from camera
		sts = psm->AcquireFrame();
		frames++;
		// Extract depth and colour
		sample = psm->QuerySample();
		depth = sample->depth;
		colour = sample->color;
		// This retrieves the ir image from the camera
		// ir = sample->ir;
		// ConvertPXCImageToOpenCVMat(ir, &ir_image, PXCImage::PIXEL_FORMAT_Y8);
		ConvertPXCImageToOpenCVMat(colour, &image, PXCImage::PIXEL_FORMAT_RGB32);
		depth->AcquireAccess(PXCImage::ACCESS_READ_WRITE, &data_depth);

		short* depth_data = reinterpret_cast<short*>(data_depth.planes[0]);
		for (int y = 0; y<920; y++)
		{
			for (int x = 0; x<320; x++)
			{
				depth_image->imageData[y * 320 + x] = depth_data[y * 320 + x];
			}
		}

		depth_mat = cvarrToMat(depth_image);
		imageCentre = Point2f(image.cols / 2, image.rows / 2);

//		// This is for testing IR for detecting the marker
//		std::vector<int> ids;
//		std::vector<std::vector<Point2f>> rejected;
//		std::vector<std::vector<Point2f>> corners;
//		auto thing = ir_image.channels();
//		detectMarkers(ir_image, dictionary, corners, ids, aruco::DetectorParameters::create(), rejected);
//		if (ids.size() > 0)
//		{
//			aruco::drawDetectedMarkers(ir_image, corners, ids);
//		}
//		imshow("IR", ir_image);
//		psm->ReleaseFrame();
//		continue;

		// Run NFT async, to be called upon later if necessary.
		nft_used = false;
		std::future<NFTResult> nft = std::async(CallProcess, tracker, image);

		// Adjust image before detecting marker
		adjusted = adjustImageBeforeDetection(image);

		// Process for marker
		std::vector<int> ids;
		std::vector<std::vector<Point2f>> rejected;
		std::vector<std::vector<Point2f>> corners;
		detectMarkers(adjusted, dictionary, corners, ids, aruco::DetectorParameters::create(), rejected);

		//image.copyTo(imageCopy);

		// If we can see the marker
		if (ids.size() > 0) {
			detection_type_history.push(Marker);
			std::cout << "Found markers\n";
			//aruco::drawDetectedMarkers(adjusted, corners, ids);

			// Find the centre of the marker
			markerCentre = centreOfRectangle(corners[0][0], corners[0][2]);
			
			offset = imageCentre - markerCentre;
			std::cout << "Centre of marker is at: (" << offset.x << "," << offset.y << ")\n";

			// Todo judge where to move to centralise the marker
			if (offset.x > offset_limit)
			{
				arrowedLine(image, imageCentre, imageCentre - Point2f(30, 0), Scalar(0, 255), 5);
			} else if (offset.x < -offset_limit)
			{
				arrowedLine(image, imageCentre, imageCentre + Point2f(30, 0), Scalar(0, 255), 5);
			}

			if (offset.y > offset_limit)
			{
				arrowedLine(image, imageCentre, imageCentre - Point2f(0, 30), Scalar(0, 255), 5);
			}
			else if (offset.y < -offset_limit)
			{
				arrowedLine(image, imageCentre, imageCentre + Point2f(0, 30), Scalar(0, 255), 5);
			}

			// If we're roughly centre then we can start to go down
			if (-offset_limit < offset.x && offset.x < offset_limit && -offset_limit < offset.y && offset.y < offset_limit) {
				// Todo Get depth to marker
				Point2f diagonals[9];
				for (int i = 0; i < 8; i += 2)
				{
					diagonals[i] = centreOfRectangle(corners[0][i / 2], markerCentre);
					diagonals[i + 1] = corners[0][i / 2];
				}
				diagonals[8] = markerCentre;
				float depth_max = 0;
				float depth_min = std::numeric_limits<float>::infinity();
				float depth_total = 0;
				int num_depths = 0;
				for (int i = 0; i < 9; i++)
				{
					float depth_value = depth_data[int(diagonals[i].y * 640 + diagonals[i].x)];
					if (depth_value != 0)
					{
						depth_total += depth_value;
						num_depths++;
						if (depth_value < depth_min) depth_min = depth_value;
						if (depth_value > depth_max) depth_max = depth_value;
					}
					cv::circle(image, diagonals[i], 2, Scalar(255, 0));
				}
				float depth_average = 0;
				if (num_depths != 0) {
					depth_average = depth_total / num_depths;
				}
				depth_history.push(depth_average);

				// Determine whether or not to move down
				if (depth_average > 500 && depth_min > 400)
				{
					// Go down
					putText(image, "Go Down", imageCentre, 0, 1, Scalar(0, 0));
				} else
				{
					// Check the history to see if the last 5 frames have had averages of 0.
					// If so move down
					bool go_down = true;
					for (int i = 0; i < depth_history.size(); i++)
					{
						if (depth_history._Get_container().at(i) != 0)
						{
							go_down = false;
							break;
						}
					}
					if (go_down)
					{
						putText(image, "Go Down", imageCentre, 0, 1, Scalar(0, 0));
						// Clear the history
						for (int i = 0; i < depth_history.size(); i++)
						{
							depth_history.pop();
						}
					}
				}
				if (depth_history.size() > 5)
				{
					depth_history.pop();
				}
				std::cout << "Depth:" << depth_average << " \n";
			}
		}
		else // We can't see the marker so try using nft
		{
			putText(image, "Using NFT", Point(5, 50), 0, 1, Scalar(255, 255, 255));
			nftresult = nft.get();
			nft_used = true;
			if (countMarkers(detection_type_history, Marker) < 2 && nftresult.Stats.ratio > 0.25 && !nftresult.Stats.keypoints == 0 && !nftresult.ResultantImage.empty()) {
				detected++;
				total_matches += nftresult.Stats.matches;
				resulting_image = nftresult.ResultantImage;
				homography = nftresult.Homography;
				Mat shifted_vector = homography * directional_vector;
				//			std::ostringstream h;
				//			h << homography;
				//			std::string homography_string = h.str();
				//			std::vector<std::string> split_string = split(homography_string, "\n");
				//			putText(resulting_image, "Homography: " + split_string[0], Point(5, 970), 0, 1, Scalar(255, 255, 255));
				//			putText(resulting_image, split_string[1], Point(5, 1005), 0, 1, Scalar(255, 255, 255));
				//			putText(resulting_image, split_string[2], Point(5, 1040), 0, 1, Scalar(255, 255, 255));
				Mat camera_movement_vector = directional_vector - shifted_vector;
				double* x = camera_movement_vector.ptr<double>(0);
				double* y = camera_movement_vector.ptr<double>(1);
				double* z = camera_movement_vector.ptr<double>(2);
				Mat norm_shifted, norm_directional;
				normalize(shifted_vector, norm_shifted);
				normalize(directional_vector, norm_directional);
				double dot = norm_shifted.dot(norm_directional);
				double rotation = acos(dot);
				if (*x > movement_cutoff_nft)
				{
					arrowedLine(image, imageCentre, imageCentre + Point2f(30, 0), Scalar(0, 255), 5);
				}
				else if (*x < -movement_cutoff_nft)
				{
					arrowedLine(image, imageCentre, imageCentre - Point2f(30, 0), Scalar(0, 255), 5);
				}
				if (*y > movement_cutoff_nft)
				{
					arrowedLine(image, imageCentre, imageCentre + Point2f(0, 30), Scalar(0, 255), 5);
				}
				else if (*y < -movement_cutoff_nft)
				{
					arrowedLine(image, imageCentre, imageCentre - Point2f(0, 30), Scalar(0, 255), 5);
				}
			}
			detection_type_history.push(NFT);
		}
		if (!nft_used)
		{
			nftresult = nft.get();
		}
		if (detection_type_history.size() > 10)
		{
			detection_type_history.pop();
		}
		//if (frames >= 200) break;
		//video.write(image);
		//imwrite("depth_image.jpg", depth_mat);
		imshow("out", image);
		psm->ReleaseFrame();
	}
}
