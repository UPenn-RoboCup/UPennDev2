/*
 * ballfinder.h
 *
 *  Created on: 2013. 3. 20.
 *      Author: hjsong
 */

#ifndef BALLFINDER_H_
#define BALLFINDER_H_

#include <string>
#include <vector>
#include "math/point.h"
#include "image.h"
#include "framework/minIni.h"
#define COLOR_SECTION   "Find Color"
#define INVALID_VALUE   -1024.0

namespace Thor
{
	class Circle
	{
	public:
		double X;
		double Y;
		double R;

		Circle() { X = 0.0; Y = 0.0;  R = 0.0; }
		Circle(double x, double y, double r) { this->X = x; this->Y = y; this->R = r;  }
		Circle(const Circle &circle) { X = circle.X; Y = circle.Y; R = circle.R;     }
		~Circle() {   }
	};


    class BallFinder
    {
    private:
		unsigned char* temp_img;
        Circle m_circle;
        std::vector<Circle> m_circles;

        void Filtering(Image* img);
        void DetectEdge(Image* hsv_img,);
        void FindCircles(Image* hsv_img, int find_condition, double radius_min, double radius_max);

    public:
        enum
        {
        	FIND_ALL = 0,
        	FIND_BIGGEST_ONE = 1,
        	FINE_SMALLEST_ONE = 2
        };

        int m_hue;             /* 0 ~ 360 */
        int m_hue_tolerance;   /* 0 ~ 180 */
        int m_min_saturation;  /* 0 ~ 100 */
        int m_min_value;       /* 0 ~ 100 */
        double m_min_percent;  /* 0.0 ~ 100.0 */
        double m_max_percent;  /* 0.0 ~ 100.0 */

        std::string color_section;

        Image*  m_result;

        BallFinder();
        BallFinder(int hue, int hue_tol, int min_sat, int min_val, double min_per, double max_per);
        virtual ~BallFinder();

        void LoadINISettings(minIni* ini);
        void LoadINISettings(minIni* ini, const std::string &section);
        void SaveINISettings(minIni* ini);
        void SaveINISettings(minIni* ini, const std::string &section);

        std::vector<Circle>& GetCicles(Image* hsv_img, int find_condition, int edge_threshold, int accumulator_threshhold, double radius_min, double radius_max);
    };
}

#endif /* BALLFINDER_H_ */
