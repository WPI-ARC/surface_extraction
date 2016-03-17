#ifndef POINTCLUSTERS_H
#define POINTCLUSTERS_H

#include <pcl/Vertices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <surfaces/Polygons.hpp>
#include <surface_msgs/Polygons.h>
#include <surfaces/Vertices_Serialization.hpp>

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

namespace ros
{
    namespace message_traits
    {
        template<> struct IsFixedSize<surfaces::Polygons> : public FalseType {};
// TODO: Determine if this actually is a simple message
        template<> struct IsSimple<surfaces::Polygons> : public FalseType {};
        template<> struct HasHeader<surfaces::Polygons> : public TrueType {};

        template<>
        struct MD5Sum<surfaces::Polygons>
        {
            static const char* value()
            {
                return MD5Sum<surface_msgs::Polygons>::value();
            }

            static const char* value(const surfaces::Polygons& m)
            {
                return MD5Sum<surface_msgs::Polygons>::value();
            }
        };

        template<>
        struct DataType<surfaces::Polygons>
        {
            static const char* value()
            {
                return DataType<surface_msgs::Polygons>::value();
            }

            static const char* value(const surfaces::Polygons& m)
            {
                return DataType<surface_msgs::Polygons>::value();
            }
        };

        template<>
        struct Definition<surfaces::Polygons>
        {
            static const char* value()
            {
                return Definition<surface_msgs::Polygons>::value();
            }

            static const char* value(const surfaces::Polygons& m)
            {
                return Definition<surface_msgs::Polygons>::value();
            }
        };


        template<>
        struct TimeStamp<surfaces::Polygons>
        {
            static ros::Time* pointer(surfaces::Polygons &m) {
                header_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_));
                return &(header_->stamp);
            }
            static ros::Time const* pointer(const surfaces::Polygons& m) {
                header_const_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_const_));
                return &(header_const_->stamp);
            }
            static ros::Time value(const surfaces::Polygons& m) {
                return pcl_conversions::fromPCL(m.header).stamp;
            }
        private:
            static boost::shared_ptr<std_msgs::Header> header_;
            static boost::shared_ptr<std_msgs::Header> header_const_;
        };
    } // namespace message_traits

    namespace serialization
    {
        template<>
        struct Serializer<surfaces::Polygons>
        {
            template<typename Stream>
            inline static void write(Stream& stream, const surfaces::Polygons& item)
            {
                stream.next(item.header);
                stream.next(item.polygons);
            }

            template<typename Stream>
            inline static void read(Stream& stream, surfaces::Polygons& item)
            {
                // Convert ROS header to PCL header
                std_msgs::Header ros_header;
                stream.next(ros_header);
                pcl_conversions::toPCL(ros_header, item.header);
                stream.next(item.polygons);
            }

            inline static int32_t serializedLength(const surfaces::Polygons& item)
            {
                uint32_t size = 0;
                size += serializationLength(item.header);
                size += serializationLength(item.polygons);
                return size;
            }

        };
    } // namespace serialization
} // namespace ros


#endif // POINTCLUSTERS_H
