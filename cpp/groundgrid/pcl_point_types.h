
#include <boost/mpl/vector.hpp>
#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/transform.hpp>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/comparison.hpp>
#include <cstdint>

#include <cstddef> //offsetof
#include <type_traits>

#define PCL_ADD_POINT4D \
  union EIGEN_ALIGN16 { \
    float data[4]; \
    struct { \
      float x; \
      float y; \
      float z; \
    }; \
  };

#define POINT_CLOUD_REGISTER_POINT_STRUCT(name, fseq)                 \
    POINT_CLOUD_REGISTER_POINT_STRUCT_I(name,                         \
      BOOST_PP_CAT(POINT_CLOUD_REGISTER_POINT_STRUCT_X fseq, 0))      \
    /***/

namespace pcl
{
  namespace traits
  {
    template<typename T> inline
    std::enable_if_t<!std::is_array<T>::value>
    plus (T &l, const T &r)
    {
      l += r;
    }

    template<typename T> inline
    std::enable_if_t<std::is_array<T>::value>
    plus (std::remove_const_t<T> &l, const T &r)
    {
      using type = std::remove_all_extents_t<T>;
      static const std::uint32_t count = sizeof (T) / sizeof (type);
      for (int i = 0; i < count; ++i)
        l[i] += r[i];
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<!std::is_array<T1>::value>
    plusscalar (T1 &p, const T2 &scalar)
    {
      p += scalar;
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<std::is_array<T1>::value>
    plusscalar (T1 &p, const T2 &scalar)
    {
      using type = std::remove_all_extents_t<T1>;
      static const std::uint32_t count = sizeof (T1) / sizeof (type);
      for (int i = 0; i < count; ++i)
        p[i] += scalar;
    }

    template<typename T> inline
    std::enable_if_t<!std::is_array<T>::value>
    minus (T &l, const T &r)
    {
      l -= r;
    }

    template<typename T> inline
    std::enable_if_t<std::is_array<T>::value>
    minus (std::remove_const_t<T> &l, const T &r)
    {
      using type = std::remove_all_extents_t<T>;
      static const std::uint32_t count = sizeof (T) / sizeof (type);
      for (int i = 0; i < count; ++i)
        l[i] -= r[i];
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<!std::is_array<T1>::value>
    minusscalar (T1 &p, const T2 &scalar)
    {
      p -= scalar;
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<std::is_array<T1>::value>
    minusscalar (T1 &p, const T2 &scalar)
    {
      using type = std::remove_all_extents_t<T1>;
      static const std::uint32_t count = sizeof (T1) / sizeof (type);
      for (int i = 0; i < count; ++i)
        p[i] -= scalar;
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<!std::is_array<T1>::value>
    mulscalar (T1 &p, const T2 &scalar)
    {
      p *= scalar;
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<std::is_array<T1>::value>
    mulscalar (T1 &p, const T2 &scalar)
    {
      using type = std::remove_all_extents_t<T1>;
      static const std::uint32_t count = sizeof (T1) / sizeof (type);
      for (int i = 0; i < count; ++i)
        p[i] *= scalar;
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<!std::is_array<T1>::value>
    divscalar (T1 &p, const T2 &scalar)
    {
      p /= scalar;
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<std::is_array<T1>::value>
    divscalar (T1 &p, const T2 &scalar)
    {
      using type = std::remove_all_extents_t<T1>;
      static const std::uint32_t count = sizeof (T1) / sizeof (type);
      for (int i = 0; i < count; ++i)
        p[i] /= scalar;
    }
  }
}

// Point operators
#define PCL_PLUSEQ_POINT_TAG(r, data, elem)                \
  pcl::traits::plus (lhs.BOOST_PP_TUPLE_ELEM(3, 1, elem),  \
                     rhs.BOOST_PP_TUPLE_ELEM(3, 1, elem)); \
  /***/

#define PCL_PLUSEQSC_POINT_TAG(r, data, elem)                 \
  pcl::traits::plusscalar (p.BOOST_PP_TUPLE_ELEM(3, 1, elem), \
                           scalar);                           \
  /***/
   //p.BOOST_PP_TUPLE_ELEM(3, 1, elem) += scalar;  \

#define PCL_MINUSEQ_POINT_TAG(r, data, elem)                \
  pcl::traits::minus (lhs.BOOST_PP_TUPLE_ELEM(3, 1, elem),  \
                      rhs.BOOST_PP_TUPLE_ELEM(3, 1, elem)); \
  /***/

#define PCL_MINUSEQSC_POINT_TAG(r, data, elem)                 \
  pcl::traits::minusscalar (p.BOOST_PP_TUPLE_ELEM(3, 1, elem), \
                            scalar);                           \
  /***/
   //p.BOOST_PP_TUPLE_ELEM(3, 1, elem) -= scalar;   \

#define PCL_MULEQSC_POINT_TAG(r, data, elem)                 \
  pcl::traits::mulscalar (p.BOOST_PP_TUPLE_ELEM(3, 1, elem), \
                            scalar);                         \
  /***/

#define PCL_DIVEQSC_POINT_TAG(r, data, elem)   \
  pcl::traits::divscalar (p.BOOST_PP_TUPLE_ELEM(3, 1, elem), \
                            scalar);                         \
  /***/

#define POINT_CLOUD_REGISTER_POINT_STRUCT_I(name, seq)                           \
  namespace pcl                                                                  \
  {                                                                              \
    namespace fields                                                             \
    {                                                                            \
      BOOST_PP_SEQ_FOR_EACH(POINT_CLOUD_REGISTER_FIELD_TAG, name, seq)           \
    }                                                                            \
    namespace traits                                                             \
    {                                                                            \
      BOOST_PP_SEQ_FOR_EACH(POINT_CLOUD_REGISTER_FIELD_NAME, name, seq)          \
      BOOST_PP_SEQ_FOR_EACH(POINT_CLOUD_REGISTER_FIELD_OFFSET, name, seq)        \
      BOOST_PP_SEQ_FOR_EACH(POINT_CLOUD_REGISTER_FIELD_DATATYPE, name, seq)      \
      POINT_CLOUD_REGISTER_POINT_FIELD_LIST(name, POINT_CLOUD_EXTRACT_TAGS(seq)) \
    }                                                                            \
    namespace common                                           \
    {                                                          \
      inline const name&                                       \
      operator+= (name& lhs, const name& rhs)                  \
      {                                                        \
        BOOST_PP_SEQ_FOR_EACH(PCL_PLUSEQ_POINT_TAG, _, seq)    \
        return (lhs);                                          \
      }                                                        \
      inline const name&                                       \
      operator+= (name& p, const float& scalar)                \
      {                                                        \
        BOOST_PP_SEQ_FOR_EACH(PCL_PLUSEQSC_POINT_TAG, _, seq)  \
        return (p);                                            \
      }                                                        \
      inline const name operator+ (const name& lhs, const name& rhs)   \
      { name result = lhs; result += rhs; return (result); }           \
      inline const name operator+ (const float& scalar, const name& p) \
      { name result = p; result += scalar; return (result); }          \
      inline const name operator+ (const name& p, const float& scalar) \
      { name result = p; result += scalar; return (result); }          \
      inline const name&                                       \
      operator-= (name& lhs, const name& rhs)                  \
      {                                                        \
        BOOST_PP_SEQ_FOR_EACH(PCL_MINUSEQ_POINT_TAG, _, seq)   \
        return (lhs);                                          \
      }                                                        \
      inline const name&                                       \
      operator-= (name& p, const float& scalar)                \
      {                                                        \
        BOOST_PP_SEQ_FOR_EACH(PCL_MINUSEQSC_POINT_TAG, _, seq) \
        return (p);                                            \
      }                                                        \
      inline const name operator- (const name& lhs, const name& rhs)   \
      { name result = lhs; result -= rhs; return (result); }           \
      inline const name operator- (const float& scalar, const name& p) \
      { name result = p; result -= scalar; return (result); }          \
      inline const name operator- (const name& p, const float& scalar) \
      { name result = p; result -= scalar; return (result); }          \
      inline const name&                                       \
      operator*= (name& p, const float& scalar)                \
      {                                                        \
        BOOST_PP_SEQ_FOR_EACH(PCL_MULEQSC_POINT_TAG, _, seq)   \
        return (p);                                            \
      }                                                        \
      inline const name operator* (const float& scalar, const name& p) \
      { name result = p; result *= scalar; return (result); }          \
      inline const name operator* (const name& p, const float& scalar) \
      { name result = p; result *= scalar; return (result); }          \
      inline const name&                                       \
      operator/= (name& p, const float& scalar)                \
      {                                                        \
        BOOST_PP_SEQ_FOR_EACH(PCL_DIVEQSC_POINT_TAG, _, seq)   \
        return (p);                                            \
      }                                                        \
      inline const name operator/ (const float& scalar, const name& p) \
      { name result = p; result /= scalar; return (result); }          \
      inline const name operator/ (const name& p, const float& scalar) \
      { name result = p; result /= scalar; return (result); }          \
    }                                                          \
  }                                                            \
  /***/

#define POINT_CLOUD_REGISTER_FIELD_TAG(r, name, elem)   \
  struct BOOST_PP_TUPLE_ELEM(3, 2, elem);               \
  /***/

#define POINT_CLOUD_REGISTER_FIELD_NAME(r, point, elem)                 \
  template<int dummy>                                                   \
  struct name<point, pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem), dummy> \
  {                                                                     \
    static const char value[];                                          \
  };                                                                    \
                                                                        \
  template<int dummy>                                                   \
  const char name<point,                                                \
                  pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem),         \
                  dummy>::value[] =                                     \
    BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(3, 2, elem));                \
  /***/

#define POINT_CLOUD_REGISTER_FIELD_OFFSET(r, name, elem)                \
  template<> struct offset<name, pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem)> \
  {                                                                     \
    static const std::size_t value = offsetof(name, BOOST_PP_TUPLE_ELEM(3, 1, elem)); \
  };                                                                    \
  /***/

#define POINT_CLOUD_REGISTER_FIELD_DATATYPE(r, name, elem)              \
  template<> struct datatype<name, pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem)> \
  {                                                                     \
    using type = boost::mpl::identity<BOOST_PP_TUPLE_ELEM(3, 0, elem)>::type; \
    using decomposed = decomposeArray<type>;                            \
    static const std::uint8_t value = asEnum<decomposed::type>::value;       \
    static const std::uint32_t size = decomposed::value;                     \
  };                                                                    \
  /***/

#define POINT_CLOUD_TAG_OP(s, data, elem) pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem)

#define POINT_CLOUD_EXTRACT_TAGS(seq) BOOST_PP_SEQ_TRANSFORM(POINT_CLOUD_TAG_OP, _, seq)

#define POINT_CLOUD_REGISTER_POINT_FIELD_LIST(name, seq)        \
  template<> struct fieldList<name>                             \
  {                                                             \
    using type = boost::mpl::vector<BOOST_PP_SEQ_ENUM(seq)>;    \
  };                                                            \
  /***/
