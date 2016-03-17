//
// Created by will on 3/14/16.
//

#ifndef SURFACES_SEGMENT_HPP
#define SURFACES_SEGMENT_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <surface_msgs/Segment.h>
#include <surfaces/pcl_shim/ModelCoefficients_Serialization.hpp>

namespace surfaces {

    template <typename PointType>
    struct Segment
    {
        Segment() : header(), surface_id(Segment::NEW_SURFACE), model(), inliers()
        {}

        ::pcl::PCLHeader header;

        int surface_id;

        pcl::ModelCoefficients model;

        pcl::PointCloud<PointType> inliers;

    public:
        const static int NEW_SURFACE = -1;
        typedef boost::shared_ptr< ::surfaces::Segment<PointType> > Ptr;
        typedef boost::shared_ptr< ::surfaces::Segment<PointType> const> ConstPtr;
    }; // struct Segment

    template <typename PointType>
    inline std::ostream& operator << (std::ostream& s, const ::surfaces::Segment<PointType> &v)
    {
        s << "header: " << std::endl;
        s << "  " << v.header;
        s << "surface_id: " << v.surface_id << std::endl;
        s << "model[]" << std::endl;
        for (size_t i = 0; i < v.model.values.size(); ++i)
        {
            s << "  model[" << i << "]: ";
            s << "  " << v.model.values.at(i) << std::endl;
        }
        s << "inliers[]" << std::endl;
        s << "  " << v.inliers;
        return (s);
    }
}

namespace ros
{
    namespace message_traits
    {
        template<typename T> struct IsFixedSize<surfaces::Segment<T> > : public FalseType {};
        template<typename T> struct IsSimple<surfaces::Segment<T> > : public FalseType {};
        template<typename T> struct HasHeader<surfaces::Segment<T> > : public FalseType {};

        template<typename T>
        struct MD5Sum<surfaces::Segment<T> >
        {
            static const char* value()
            {
                return MD5Sum<surface_msgs::Segment>::value();
            }

            static const char* value(const surfaces::Segment<T>& m)
            {
                return MD5Sum<surface_msgs::Segment>::value();
            }
        };

        template<typename T>
        struct DataType<surfaces::Segment<T> >
        {
            static const char* value()
            {
                return DataType<surface_msgs::Segment>::value();
            }

            static const char* value(const surfaces::Segment<T>& m)
            {
                return DataType<surface_msgs::Segment>::value();
            }
        };

        template<typename T>
        struct Definition<surfaces::Segment<T> >
        {
            static const char* value()
            {
                return Definition<surface_msgs::Segment>::value();
            }

            static const char* value(const surfaces::Segment<T>& m)
            {
                return Definition<surface_msgs::Segment>::value();
            }
        };
        template<typename T>
        struct TimeStamp<surfaces::Segment<T> >
        {
            static ros::Time* pointer(surfaces::Segment<T> &m) {
                header_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_));
                return &(header_->stamp);
            }
            static ros::Time const* pointer(const surfaces::Segment<T>& m) {
                header_const_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_const_));
                return &(header_const_->stamp);
            }
            static ros::Time value(const surfaces::Segment<T>& m) {
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
        struct Serializer<surfaces::Segment<T> >
        {
            template<typename Stream>
            inline static void write(Stream& stream, const surfaces::Segment<T>& item)
            {
                stream.next(item.header);
                stream.next(item.surface_id);
                stream.next(item.model);
                stream.next(item.inliers);
            }

            template<typename Stream>
            inline static void read(Stream& stream, surfaces::Segment<T>& item)
            {
                stream.next(item.header);
                stream.next(item.surface_id);
                stream.next(item.model);
                stream.next(item.inliers);
            }

            inline static int32_t serializedLength(const surfaces::Segment<T>& item)
            {
                uint32_t size = 0;
                size += serializationLength(item.header);
                size += serializationLength(item.surface_id);
                size += serializationLength(item.model);
                size += serializationLength(item.inliers);
                return size;
            }

        };
    } // namespace serialization

} // namespace ros
#endif //SURFACES_SEGMENT_HPP
