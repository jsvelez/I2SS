//NOTE: This has yet to be implemented in the actual code

#ifndef POINT_SEG_H_
#define POINT_SEG_H_

// This struct is a custom PCL point type to store a 3D coordinate and a segment_id.
// This way the result of a segmentation can be stored directly on the points as labels.
// When you see "4D", that is simply to do a fancy bit alignment, we are really only interested
// in storing 3D coordinates.

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
}


#endif 
