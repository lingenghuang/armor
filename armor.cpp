
#include <vector>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <math.h>
#include <omp.h>
#include <stdio.h> /*标准输入输出的定义*/
#include <errno.h> /*错误号定义*/
#include <sys/stat.h>
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <stdlib.h>  /*标准函数库定义*/
#include <sys/types.h>
#include <unistd.h> /*UNIX 标准函数定义*/
using namespace cv;
using namespace std;

#define BLUE 0 //选择检测蓝色还是红色装甲

const double PI = 3.14159;

int set_serial(int fd, int nSpeed, int nBits, char nEvent, int nStop); //串口初始化设置   (fd,波特率,数据位,奇偶校验,停止位)

double cal_distance(RotatedRect ellipse_1, RotatedRect ellipse_2); //用于计算两椭圆中心点距离

class EllipseMatch //完成匹配时储存两个椭圆的一个类型
{
  public:
	RotatedRect ellipse_1;
	RotatedRect ellipse_2;
	EllipseMatch(RotatedRect src_1, RotatedRect src_2)
	{
		ellipse_1 = src_1;
		ellipse_2 = src_2;
	}
};

double get_angle(double x1, double y1, double x2, double y2); //计算直线的角度

class hsv
{
	friend Mat gray(Mat &src);

  public:
	hsv(int src_b, int src_g, int src_r); //rgb转化为hsv值并储存

  private:
	int s[3];
	int max;
	int min;
	double S;
	int V;
	double H;
};

class arrange_ellipse //储存并匹配椭圆
{
  public:
	arrange_ellipse(std::vector<RotatedRect> src_ellipse); //储存并匹配椭圆

	vector<EllipseMatch> getmatch() //返回匹配的椭圆
	{
		return matched_ellipse;
	}

  private:
	std::vector<EllipseMatch> matched_ellipse;
};

Mat gray(Mat &src); //根据hsv值进行阈值化

int main(int argc, char **argv)
{
    std::stringstream input_video;
    if (argc == 2)
    {
        input_video << argv[1];
    }
    else
    {
        input_video << "1.mp4";
    }
	long int count_frame=0;
	VideoWriter writer("output1.avi",CV_FOURCC('M','J','P','G'),60,Size(640,480));
	VideoCapture capture(input_video.str());
	int jump = 0; //某个视频前几帧imshow会报错，这里的jump用来跳过前几帧
	int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	set_serial(fd, 115200, 8, 'N', 1); //初始化串口
	while (1)
	{
		Mat src;
		capture >> src;
		jump++; //某个视频前几帧imshow会报错，这里的jump用来跳过前几帧
		if (jump > 10)
		{
			if(src.empty())
			    break;
			Mat image = gray(src);												   //阈值化
			vector<std::vector <Point> > contours;								   //储存找到的轮廓
			findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //查找外轮廓

			vector<RotatedRect> src_ellipse;		  //储存fitellipse函数找到的的椭圆
			for (int n = 0; n < contours.size(); n++) //拟合出椭圆，同时初步筛选掉面积过小的椭圆
			{
				if (contourArea(contours[n]) > 20)
				{
					cv::RotatedRect ellipsepoint = cv::fitEllipse(contours[n]);
					src_ellipse.push_back(ellipsepoint);
				}
			}
			arrange_ellipse ep(src_ellipse);						//用类封装椭圆，同时进行筛选和匹配
			std::vector<EllipseMatch> matchEllipse = ep.getmatch(); //得到匹配到的装甲椭圆
			vector<Point> center_point;								//储存装甲片中心点坐标
			center_point.clear();
			for (int n = 0; n < matchEllipse.size(); n++) //画出匹配到的装甲片和中心点
			{
				cv::ellipse(src, matchEllipse[n].ellipse_1, Scalar(0, 255, 255), 3);
				cv::ellipse(src, matchEllipse[n].ellipse_2, Scalar(0, 255, 255), 3);
				line(src, Point(matchEllipse[n].ellipse_1.center), Point(matchEllipse[n].ellipse_2.center), Scalar(255, 255, 255), 3);

				circle(src, (matchEllipse[n].ellipse_1.center + matchEllipse[n].ellipse_2.center) / 2, 5, Scalar(0, 255, 0), -1);
				center_point.push_back((matchEllipse[n].ellipse_1.center + matchEllipse[n].ellipse_2.center) / 2);
			}

			if (center_point.size() > 0) //发送其中一个中心点的数据
			{
				char head[] = "head";
				int bytes_written = 0;
				bytes_written = write(fd, head, sizeof(head));

				char write_buffer[8];
				write_buffer[0] = center_point[0].x / 100 + 48;
				write_buffer[1] = center_point[0].x / 10 % 10 + 48;
				write_buffer[2] = center_point[0].x % 10 + 48;
				write_buffer[3] = 32;
				write_buffer[4] = center_point[0].y / 100 + 48;
				write_buffer[5] = center_point[0].y / 10 % 10 + 48;
				write_buffer[6] = center_point[0].y % 10 + 48;
				write_buffer[7] = 32;
				bytes_written = 0;
				bytes_written = write(fd, write_buffer, sizeof(write_buffer));

				char tail[] = "tail";
				// cahr tail = 0x11;
				bytes_written = 0;
				bytes_written = write(fd, tail, sizeof(tail));
			}
            count_frame++;
			if(count_frame%300==0)
			{
				cout<<count_frame<<endl;
			}
			writer << src;
			waitKey(1);
		}
	}

	return 0;
}

int set_serial(int fd, int nSpeed, int nBits, char nEvent, int nStop) //fd,波特率,数据位,奇偶校验,停止位
{
	struct termios newttys1;
	newttys1.c_cflag |= (CLOCAL | CREAD); /*CREAD 开启串行数据接收，CLOCAL并打开本地连接模式*/
	newttys1.c_cflag &= ~CSIZE;

	/*设置数据位*/ /*数据位选择*/
	switch (nBits)
	{
	case 7:
		newttys1.c_cflag |= CS7;
		break;
	case 8:
		newttys1.c_cflag |= CS8;
		break;
	default:
		printf("error bits,only 7 and 8 is avaliable\n");
		getchar();
		exit(0);
	}

	/*设置奇偶校验位*/
	switch (nEvent)
	{
	case '0':								  /*奇校验*/
		newttys1.c_cflag |= PARENB;			  /*开启奇偶校验*/
		newttys1.c_iflag |= (INPCK | ISTRIP); /*INPCK打开输入奇偶校验；ISTRIP去除字符的第八个比特  */
		newttys1.c_cflag |= PARODD;			  /*启用奇校验(默认为偶校验)*/
		break;
	case 'E':								  /*偶校验*/
		newttys1.c_cflag |= PARENB;			  /*开启奇偶校验  */
		newttys1.c_iflag |= (INPCK | ISTRIP); /*打开输入奇偶校验并去除字符第八个比特*/
		newttys1.c_cflag &= ~PARODD;		  /*启用偶校验*/
		break;
	case 'N': /*无奇偶校验*/
		newttys1.c_cflag &= ~PARENB;
		break;
	default:
		printf("error nEvent,only 0 and 'E' and 'N' is avaliable\n");
		getchar();
		exit(0);
	}

	/*设置波特率*/
	switch (nSpeed)
	{
	case 2400:
		cfsetispeed(&newttys1, B2400);
		cfsetospeed(&newttys1, B2400);
		break;
	case 4800:
		cfsetispeed(&newttys1, B4800);
		cfsetospeed(&newttys1, B4800);
		break;
	case 9600:
		cfsetispeed(&newttys1, B9600);
		cfsetospeed(&newttys1, B9600);
		break;
	case 115200:
		cfsetispeed(&newttys1, B115200);
		cfsetospeed(&newttys1, B115200);
		break;
	default:
		cfsetispeed(&newttys1, B115200);
		cfsetospeed(&newttys1, B115200);
		break;
	}

	/*设置停止位*/
	if (nStop == 1) /*设置停止位；若停止位为1，则清除CSTOPB，若停止位为2，则激活CSTOPB*/
	{
		newttys1.c_cflag &= ~CSTOPB; /*默认为一位停止位； */
	}
	else if (nStop == 2)
	{
		newttys1.c_cflag |= CSTOPB; /*CSTOPB表示送两位停止位*/
	}

	/*设置最少字符和等待时间，对于接收字符和等待时间没有特别的要求时*/
	newttys1.c_cc[VTIME] = 0; /*非规范模式读取时的超时时间；*/
	newttys1.c_cc[VMIN] = 0;  /*非规范模式读取时的最小字符数*/

	tcflush(fd, TCIFLUSH); /*tcflush清空终端未完成的输入/输出请求及数据；TCIFLUSH表示清空正收到的数据，且不读取出来 */ /*激活配置使其生效*/
	if ((tcsetattr(fd, TCSANOW, &newttys1)) != 0)
	{
		perror("com set error");
		return -1;
	}
	return 0;
}

double cal_distance(RotatedRect ellipse_1, RotatedRect ellipse_2)
{
	return sqrt((ellipse_1.center.x - ellipse_2.center.x) * (ellipse_1.center.x - ellipse_2.center.x) + (ellipse_1.center.y - ellipse_2.center.y) * (ellipse_1.center.y - ellipse_2.center.y));
}

double get_angle(double x1, double y1, double x2, double y2)
{
	double angle;
	if (x2 == x1)
		angle = 180;
	else
	{
		double K = (y2 - y1) / (x2 - x1);
		angle = 90 + atan(K) / PI * 180.;
	}
	return angle;
}

hsv::hsv(int src_b, int src_g, int src_r)
{
	s[0] = src_b;
	max = src_b;
	min = src_b;
	s[1] = src_g;
	if (s[1] > max)
		max = s[1];
	if (s[1] < min)
		min = s[1];
	s[2] = src_r;
	if (s[2] < min)
		min = s[2];
	if (s[2] > max)
		max = s[2];
	S = (double)(max - min) / (double)max;
	V = max;

	if (max == s[0])
	{
		H = 240 + (double)(s[2] - s[1]) / (double)(max - min) * 60.;
		if (H < 0)
		{
			H = H + 360.;
		}
	}
	else if (max == s[1])
	{
		H = 120 + (double)(s[0] - s[2]) / (double)(max - min) * 60.;
		if (H < 0)
		{
			H = H + 360.;
		}
	}
	else
	{
		H = (double)(s[1] - s[0]) / (double)(max - min) * 60.;
		if (H < 0)
		{
			H = H + 360.;
		}
	}
}

arrange_ellipse::arrange_ellipse(std::vector<cv::RotatedRect> src_ellipse)
{
	bool *num = new bool[src_ellipse.size()];
	for (int n = 0; n < src_ellipse.size(); n++)
	{
		num[n] = 0;
	}
	for (int n = 0; n < src_ellipse.size(); n++)
	{
		if (num[n] == 0)
		{
			if ((src_ellipse[n].angle < 45 || src_ellipse[n].angle > 135) && src_ellipse[n].size.height > 1.7 * src_ellipse[n].size.width)
			{
				for (int i = n + 1; i < src_ellipse.size(); i++)
				{
					if ((src_ellipse[i].angle < 45 || src_ellipse[i].angle > 135) && src_ellipse[i].size.height > 1.7 * src_ellipse[i].size.width)
					{
						if (abs(src_ellipse[n].angle - src_ellipse[i].angle) <= 7 || abs(src_ellipse[n].angle - src_ellipse[i].angle) >= 173)
						{
							if (abs(src_ellipse[n].size.height - src_ellipse[i].size.height) / src_ellipse[n].size.height < 0.18)

							{
								double angle = get_angle(src_ellipse[n].center.x, src_ellipse[n].center.y, src_ellipse[i].center.x, src_ellipse[i].center.y);
								if (angle > 45 && angle < 135 && abs(abs(src_ellipse[n].angle - angle) - 90) + abs(abs(src_ellipse[i].angle - angle) - 90) < 20)
								{
									double distance = cal_distance(src_ellipse[n], src_ellipse[i]);
									if (distance / src_ellipse[n].size.height < 4.9 && distance / src_ellipse[i].size.height < 4.9 && distance / src_ellipse[n].size.height > 1 && distance / src_ellipse[i].size.height > 1)
									{
										matched_ellipse.push_back(EllipseMatch(src_ellipse[n], src_ellipse[i]));
										num[i] = 1;
										break;
									}
								}
							}
						}
					}
				}
			}
		}
	}
	delete[] num;
}

#if BLUE
Mat gray(Mat &src)
{

	Mat image(src.size(), CV_8U, Scalar(0));

	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			hsv imagehsv(src.at<Vec3b>(i, j)[0], src.at<Vec3b>(i, j)[1], src.at<Vec3b>(i, j)[2]);
			if (imagehsv.V > 130 && (imagehsv.H > 190 && imagehsv.H < 260))
			{
				if (imagehsv.S > 0.35)
				{
					image.at<uchar>(i, j) = 255;
				}
			}
		}
	}
	return image;
}
#else
Mat gray(Mat &src)
{
	Mat image(src.size(), CV_8U, Scalar(0));

	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			hsv imagehsv(src.at<Vec3b>(i, j)[0], src.at<Vec3b>(i, j)[1], src.at<Vec3b>(i, j)[2]);
			if (imagehsv.V > 130 && (imagehsv.H > 330 || imagehsv.H < 30))
			{
				if (imagehsv.S > 0.35)
				{
					image.at<uchar>(i, j) = 255;
				}
			}
		}
	}
	return image;
}
#endif