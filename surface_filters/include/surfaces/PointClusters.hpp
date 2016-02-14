#ifndef POINTCLUSTERS_H
#define POINTCLUSTERS_H

#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

namespace surfaces {

// Defined exactly the same as pcl::PointIndices
struct PointClusters
{
  PointClusters() : header(), has_models(), clusters(), models()
  {}

  ::pcl::PCLHeader header;

  bool has_models;

  std::vector<pcl::PointIndices> clusters;

  std::vector<pcl::ModelCoefficients> models;

  public:
    typedef boost::shared_ptr< ::surfaces::PointClusters> Ptr;
    typedef boost::shared_ptr< ::surfaces::PointClusters const> ConstPtr;
}; // struct PointClusters

typedef boost::shared_ptr< ::surfaces::PointClusters> PointClustersPtr;
typedef boost::shared_ptr< ::surfaces::PointClusters const> PointClustersConstPtr;

inline std::ostream& operator << (std::ostream& s, const ::surfaces::PointClusters &v)
{
  s << "header: " << std::endl;
  s << "  " << v.header;
  s << "clusters[]" << std::endl;
  for (size_t i = 0; i < v.clusters.size (); ++i)
  {
    s << "  clusters[" << i << "]: ";
    s << "  " << v.clusters[i] << std::endl;
  }
  s << "models[]" << std::endl;
  for (size_t i = 0; i < v.models.size (); ++i)
  {
    s << "  models[" << i << "]: ";
    s << "  " << v.models[i] << std::endl;
  }
  return (s);
}
}

#endif // POINTCLUSTERS_H
