#ifndef surface_types_HPP
#define surface_types_HPP

#include <pcl/PCLHeader.h>
#include "Surface.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <surface_msgs2/Surfaces.h>

namespace surface_types {

// Defined similarly to pcl::PointIndices
    template <typename PointType>
    struct Surfaces
    {
        Surfaces() : header(), latest_update(), surfaces()
        {}

        pcl::PCLHeader header;
        pcl::uint64_t latest_update;

        std::vector<Surface<PointType> > surfaces;


    public:

        operator surface_msgs2::Surfaces() const {
            surface_msgs2::Surfaces ros_surfaces;

            pcl_conversions::fromPCL(header, ros_surfaces.header);
            pcl_conversions::fromPCL(latest_update, ros_surfaces.latest_update);
            ros_surfaces.surfaces.reserve(surfaces.size());
            std::copy(surfaces.begin(), surfaces.end(), std::back_inserter(ros_surfaces.surfaces));

            return ros_surfaces;
        }
        typedef boost::shared_ptr< ::surface_types::Surfaces<PointType> > Ptr;
        typedef boost::shared_ptr< ::surface_types::Surfaces<PointType> const> ConstPtr;
    }; // struct surface_types

    template <typename PointType>
    inline std::ostream& operator << (std::ostream& s, const ::surface_types::Surfaces<PointType> &v)
    {
        s << "header: " << std::endl;
        s << "  " << v.header;
        s << "latest_update: " << std::endl;
        s << "  " << v.latest_update;
        s << "surface_types[]" << std::endl;
        for (size_t i = 0; i < v.surfaces.size(); ++i)
        {
            s << "  surface_types[" << i << "]: ";
            s << "  " << v.surfaces[i] << std::endl;
        }
        return (s);
    }
}

namespace ros
{
    namespace message_traits
    {
        template<typename T> struct IsFixedSize<surface_types::Surfaces<T> > : public FalseType {};
// TODO: Determine if this actually is a simple message
        template<typename T> struct IsSimple<surface_types::Surfaces<T> > : public FalseType {};
        template<typename T> struct HasHeader<surface_types::Surfaces<T> > : public FalseType {};

        template<typename T>
        struct MD5Sum<surface_types::Surfaces<T> >
        {
            static const char* value()
            {
                return MD5Sum<surface_msgs2::Surfaces>::value();
            }

            static const char* value(const surface_types::Surfaces<T>& m)
            {
                return MD5Sum<surface_msgs2::Surfaces>::value();
            }
        };

        template<typename T>
        struct DataType<surface_types::Surfaces<T> >
        {
            static const char* value()
            {
                return DataType<surface_msgs2::Surfaces>::value();
            }

            static const char* value(const surface_types::Surfaces<T>& m)
            {
                return DataType<surface_msgs2::Surfaces>::value();
            }
        };

        template<typename T>
        struct Definition<surface_types::Surfaces<T> >
        {
            static const char* value()
            {
                return Definition<surface_msgs2::Surfaces>::value();
            }

            static const char* value(const surface_types::Surfaces<T>& m)
            {
                return Definition<surface_msgs2::Surfaces>::value();
            }
        };


        template<typename T>
        struct TimeStamp<surface_types::Surfaces<T> >
        {
            static ros::Time* pointer(surface_types::Surfaces<T> &m) {
                header_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_));
                return &(header_->stamp);
            }
            static ros::Time const* pointer(const surface_types::Surfaces<T>& m) {
                header_const_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_const_));
                return &(header_const_->stamp);
            }
            static ros::Time value(const surface_types::Surfaces<T>& m) {
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
        struct Serializer<surface_types::Surfaces<T> >
        {
            template<typename Stream>
            inline static void write(Stream& stream, const surface_types::Surfaces<T>& item)
            {
                stream.next(item.header);
                stream.next(item.latest_update);
                stream.next(item.surfaces);
            }

            template<typename Stream>
            inline static void read(Stream& stream, surface_types::Surfaces<T>& item)
            {
                // Convert ROS header to PCL header
                std_msgs::Header ros_header;
                stream.next(ros_header);
                pcl_conversions::toPCL(ros_header, item.header);
                stream.next(item.latest_update);
                stream.next(item.surfaces);
            }

            inline static int32_t serializedLength(const surface_types::Surfaces<T>& item)
            {
                uint32_t size = 0;
                size += serializationLength(item.header);
                size += serializationLength(item.surfaces);
                size += serializationLength(item.latest_update);
                return size;
            }

        };
    } // namespace serialization
} // namespace ros

#endif // surface_types_HPP
