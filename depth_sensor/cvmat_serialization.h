/*
 * cvmat_serialization.h
 *
 *  Created on: Feb 15, 2017
 *      Author: kevin
 */

#ifndef CVMAT_SERIALIZATION_H_
#define CVMAT_SERIALIZATION_H_

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <opencv2/core/core.hpp>

BOOST_SERIALIZATION_SPLIT_FREE( ::cv::Mat )

namespace boost {
namespace serialization {

    template< typename Archive >
    void save( Archive &_ar, const ::cv::Mat & _mat, const unsigned int _version )
    {
        size_t elem_size = _mat.elemSize();
        size_t elem_type = _mat.type();

        _ar & _mat.cols;
        _ar & _mat.rows;
        _ar & elem_size;
        _ar & elem_type;

        const size_t data_size = _mat.cols * _mat.rows * elem_size;
        _ar & boost::serialization::make_array( _mat.ptr(), data_size );
    }

    template< typename Archive >
    void load( Archive &_ar, ::cv::Mat &_mat, const unsigned int _version )
    {
        int cols, rows;
        size_t elem_size, elem_type;

        _ar & cols;
        _ar & rows;
        _ar & elem_size;
        _ar & elem_type;

        _mat.create( rows, cols, elem_type );

        const size_t data_size = _mat.cols * _mat.rows * elem_size;
        _ar & boost::serialization::make_array( _mat.ptr(), data_size );
    }

} //end-of-namespace serialization
} //end-of-namespace boost




#endif /* CVMAT_SERIALIZATION_H_ */
