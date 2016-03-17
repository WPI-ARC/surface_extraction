#ifndef SURFACES_HPP
#define SURFACES_HPP

#include <pcl/PCLHeader.h>
#include "Surface.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <surface_msgs/Surfaces.h>

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

namespace ros
{
    namespace message_traits
    {
        template<typename T> struct IsFixedSize<surfaces::Surfaces<T> > : public FalseType {};
// TODO: Determine if this actually is a simple message
        template<typename T> struct IsSimple<surfaces::Surfaces<T> > : public FalseType {};
        template<typename T> struct HasHeader<surfaces::Surfaces<T> > : public FalseType {};

        template<typename T>
        struct MD5Sum<surfaces::Surfaces<T> >
        {
            static const char* value()
            {
                return MD5Sum<surface_msgs::Surfaces>::value();
            }

            static const char* value(const surfaces::Surfaces<T>& m)
            {
                return MD5Sum<surface_msgs::Surfaces>::value();
            }
        };

        template<typename T>
        struct DataType<surfaces::Surfaces<T> >
        {
            static const char* value()
            {
                return DataType<surface_msgs::Surfaces>::value();
            }

            static const char* value(const surfaces::Surfaces<T>& m)
            {
                return DataType<surface_msgs::Surfaces>::value();
            }
        };

        template<typename T>
        struct Definition<surfaces::Surfaces<T> >
        {
            static const char* value()
            {
                return Definition<surface_msgs::Surfaces>::value();
            }

            static const char* value(const surfaces::Surfaces<T>& m)
            {
                return Definition<surface_msgs::Surfaces>::value();
            }
        };


        template<typename T>
        struct TimeStamp<surfaces::Surfaces<T> >
        {
            static ros::Time* pointer(surfaces::Surfaces<T> &m) {
                header_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_));
                return &(header_->stamp);
            }
            static ros::Time const* pointer(const surfaces::Surfaces<T>& m) {
                header_const_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_const_));
                return &(header_const_->stamp);
            }
            static ros::Time value(const surfaces::Surfaces<T>& m) {
                return pcl_conversions::fromPCL(m.header).stamp;
            }
        private:
            static boost::shared_ptr<std_msgs::Header> header_;
            static boost::shared_ptr<std_msgs::Header> header_const_;
        };
    } // namespace message_traits

    namespace serialization
    {
        template<typename T>
        struct Serializer<surfaces::Surfaces<T> >
        {
            template<typename Stream>
            inline static void write(Stream& stream, const surfaces::Surfaces<T>& item)
            {
                stream.next(item.header);
                stream.next(item.surfaces);
            }

            template<typename Stream>
            inline static void read(Stream& stream, surfaces::Surfaces<T>& item)
            {
                // Convert ROS header to PCL header
                std_msgs::Header ros_header;
                stream.next(ros_header);
                pcl_conversions::toPCL(ros_header, item.header);
                stream.next(item.surfaces);
            }

            inline static int32_t serializedLength(const surfaces::Surfaces<T>& item)
            {
                uint32_t size = 0;
                size += serializationLength(item.header);
                size += serializationLength(item.surfaces);
                return size;
            }

        };
    } // namespace serialization
} // namespace ros

#endif // SURFACES_HPP
