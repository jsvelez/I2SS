#ifndef POINT_SEG_H_
#define POINT_SEG_H_

namespace pcl 
{
  struct PointSegment 
  {
	PCL_ADD_POINT_4D;
	union
	{
	  struct
	  {
		uint32_t segment_id;
	  };
	  float data_s[4];
	};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
inline std::ostream& operator<< (std::ostream& os, const PointSegment& p)
{
  os << "(" << p.x << "," << p.y << "," p.z << ") [segmentID: " << p.segment_id << "]";
  return os;
}

#endif 
