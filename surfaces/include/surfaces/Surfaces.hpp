#ifndef SURFACES_HPP
#define SURFACES_HPP

#include <pcl/PCLHeader.h>
#include "Surface.hpp"

namespace surfaces {

// Defined similarly to pcl::PointIndices
    template <typename PointType>
    struct Surfaces
    {
        Surfaces() : header(), surfaces()
        {}

        pcl::PCLHeader header;
        std::vector<Surface<PointType> > surfaces;


    public:
        typedef boost::shared_ptr< ::surfaces::Surfaces<PointType> > Ptr;
        typedef boost::shared_ptr< ::surfaces::Surfaces<PointType> const> ConstPtr;
    }; // struct Surfaces

    template <typename PointType>
    inline std::ostream& operator << (std::ostream& s, const ::surfaces::Surfaces<PointType> &v)
    {
        s << "header: " << std::endl;
        s << "  " << v.header;
        s << "surfaces[]" << std::endl;
        for (size_t i = 0; i < v.surfaces.size(); ++i)
        {
            s << "  surfaces[" << i << "]: ";
            s << "  " << v.surfaces[i] << std::endl;
        }
        return (s);
    }
}
#endif // SURFACES_HPP
