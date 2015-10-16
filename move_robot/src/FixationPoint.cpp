#include "FixationPoint.h"

FixationPoint::FixationPoint(const double & base_line_) :
    base_line(base_line_),
    half_base_line(0.5*base_line_)
{
}

Eigen::Vector3d FixationPoint::getFixationPoint(const double & left_eye_angle_,
                                                const double & right_eye_angle_,
                                                const double & eyes_tilt_angle_)
{
    Eigen::Vector3d fixation_point;

    double tan_l=tan(left_eye_angle_);
    double tan_r=tan(right_eye_angle_);
    double aux=base_line/( tan_l-tan_r);
    //std::cout << "left_eye_angle:" << left_eye_angle_ << " right_eye_angle:"<<right_eye_angle_<< " baseline:"<< base_line<< std::endl;
    // depth
    fixation_point(2)=aux; // depth
    fixation_point(0)=half_base_line+tan_r*aux; // x
    return fixation_point;
}
