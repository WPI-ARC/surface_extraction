#ifndef POINTCLUSTERS_H
#define POINTCLUSTERS_H

#include <pcl/Vertices.h>

namespace surfaces {

// Defined exactly the same as pcl::PointIndices
    struct Polygons {
        Polygons() : header(), polygons() { }

        ::pcl::PCLHeader header;

        std::vector<pcl::Vertices> polygons;

    public:
        typedef boost::shared_ptr<::surfaces::Polygons> Ptr;
        typedef boost::shared_ptr<::surfaces::Polygons const> ConstPtr;

        unsigned long size() const {
            unsigned int s = 0;
            BOOST_FOREACH(auto polygon, this->polygons) {
                            s += polygon.vertices.size();
                        }
            return s;
        }
    }; // struct Surface

    typedef boost::shared_ptr<::surfaces::Polygons> PolygonsPtr;
    typedef boost::shared_ptr<::surfaces::Polygons const> PolygonsConstPtr;

    inline std::ostream &operator<<(std::ostream &s, const ::surfaces::Polygons &v) {
        s << "header: " << std::endl;
        s << "  " << v.header;
        s << "polygons[]" << std::endl;
        for (size_t i = 0; i < v.polygons.size(); ++i) {
            s << "  polygons[" << i << "]: ";
            s << "  " << v.polygons[i] << std::endl;
        }
        return (s);
    }
}

#endif // POINTCLUSTERS_H
