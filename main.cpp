#include <opencv2/opencv.hpp>
#include <iostream>


cv::KalmanFilter kalmanfilter(2,2);

cv::Mat last_measurement(2, 1, CV_32FC1);
cv::Mat current_measurement(2, 1, CV_32FC1);

cv::Mat last_prediction(2, 1, CV_32FC1);
cv::Mat current_prediction(2, 1, CV_32FC1);


cv::Mat frame(720, 1280, CV_8UC3);

cv::RNG rng;
static int times=0;

void onMouseMove(int event, int x, int y, int flag, void *data)
{
    //应该给定初始状态
    if (times++ == 0)
    {
        current_measurement.at<float>(0) = x + rng.gaussian(5);
        current_measurement.at<float>(1) = y + rng.gaussian(5);

        current_prediction.at<float>(0) = x;
        current_prediction.at<float>(1) = y;

        kalmanfilter.statePost = current_prediction;
        return;
    }

    last_prediction=current_prediction;
    last_measurement=current_measurement;

    current_measurement.at<float>(0) = x+rng.gaussian(5);
    current_measurement.at<float>(1) = y+rng.gaussian(5);

    kalmanfilter.predict();
    current_prediction = kalmanfilter.correct(current_measurement);

    cv::line(frame, cv::Point(last_measurement.at<float>(0), last_measurement.at<float>(1)), cv::Point(current_measurement.at<float>(0), current_measurement.at<float>(1)), cv::Scalar(0, 0, 255), 2);
    cv::line(frame, cv::Point(last_prediction.at<float>(0), last_prediction.at<float>(1)), cv::Point(current_prediction.at<float>(0), current_prediction.at<float>(1)), cv::Scalar(0, 255, 0), 2);

    return;
    
}

int main(int argc, char **argv)
{
    cv::namedWindow("test");
    cv::setMouseCallback("test", onMouseMove, NULL);//鼠标相应回调函数

    cv::Mat F(2, 2, CV_32F, cv::Scalar(0));
    cv::setIdentity(F, cv::Scalar(1));

    cv::Mat Q(2, 2, CV_32F);
    cv::setIdentity(Q, cv::Scalar(0.1));

    cv::Mat H(2, 2, CV_32F);
    cv::setIdentity(H, cv::Scalar(1));

    cv::Mat R(2, 2, CV_32F);
    cv::setIdentity(R, cv::Scalar(1));

    kalmanfilter.transitionMatrix = F;
    kalmanfilter.processNoiseCov = Q;
    kalmanfilter.measurementMatrix = H;
    kalmanfilter.measurementNoiseCov = R;

    while (true)
    {
        cv::imshow("test", frame);

        if (cv::waitKey(30) == 113)
            break;
    }

    cv::destroyAllWindows();

    return 0;
}