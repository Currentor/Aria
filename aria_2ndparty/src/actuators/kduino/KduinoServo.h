/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /Users/ssb/workspace/src/kduino/msg/KduinoServo.msg
 *
 */


#ifndef KDUINO_MESSAGE_KDUINOSERVO_H
#define KDUINO_MESSAGE_KDUINOSERVO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kduino
{
template <class ContainerAllocator>
struct KduinoServo_
{
  typedef KduinoServo_<ContainerAllocator> Type;

  KduinoServo_()
    : servo_id(0)
    , servo_value(0)  {
    }
  KduinoServo_(const ContainerAllocator& _alloc)
    : servo_id(0)
    , servo_value(0)  {
    }



   typedef int16_t _servo_id_type;
  _servo_id_type servo_id;

   typedef int16_t _servo_value_type;
  _servo_value_type servo_value;




  typedef boost::shared_ptr< ::kduino::KduinoServo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kduino::KduinoServo_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct KduinoServo_

typedef ::kduino::KduinoServo_<std::allocator<void> > KduinoServo;

typedef boost::shared_ptr< ::kduino::KduinoServo > KduinoServoPtr;
typedef boost::shared_ptr< ::kduino::KduinoServo const> KduinoServoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kduino::KduinoServo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kduino::KduinoServo_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace kduino

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'kduino': ['/Users/ssb/workspace/src/kduino/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::kduino::KduinoServo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kduino::KduinoServo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kduino::KduinoServo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kduino::KduinoServo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kduino::KduinoServo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kduino::KduinoServo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kduino::KduinoServo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "13f5474b0ceb1a33e62725e75dd4a3ac";
  }

  static const char* value(const ::kduino::KduinoServo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x13f5474b0ceb1a33ULL;
  static const uint64_t static_value2 = 0xe62725e75dd4a3acULL;
};

template<class ContainerAllocator>
struct DataType< ::kduino::KduinoServo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kduino/KduinoServo";
  }

  static const char* value(const ::kduino::KduinoServo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kduino::KduinoServo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 servo_id\n\
int16 servo_value\n\
";
  }

  static const char* value(const ::kduino::KduinoServo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kduino::KduinoServo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.servo_id);
      stream.next(m.servo_value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct KduinoServo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kduino::KduinoServo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kduino::KduinoServo_<ContainerAllocator>& v)
  {
    s << indent << "servo_id: ";
    Printer<int16_t>::stream(s, indent + "  ", v.servo_id);
    s << indent << "servo_value: ";
    Printer<int16_t>::stream(s, indent + "  ", v.servo_value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KDUINO_MESSAGE_KDUINOSERVO_H
