/*
 * perlin_noise.h
 *
 *  Created on: Apr 5, 2017
 *      Author: kevin
 */

#ifndef PERLIN_NOISE_H_
#define PERLIN_NOISE_H_

#include <math.h>
#include <vector>

using namespace std;
using namespace gazebo;

float s_shaped_curve( float _t )
{
	//return 3 * pow( _t, 2 ) - 2 * pow( _t, 3 );	// old method from Ken Perlin
	return	6 * pow( _t,5 ) - 15 * pow( _t,4 ) + 10 * pow( _t, 3 );// improved method from Ken Perlin	// http://mrl.nyu.edu/~perlin/paper445.pdf
}

vector< float > perlinNoise( int _width, int _height, int _grid_size )
{

	////cv::Mat noise( _height, _width, CV_32F );
	vector< float > noise( _width * _height );

	// ********************* //
	// generate perlin noise //
	// ********************* //

	int grid_width = _width / _grid_size + 1 + 1;	// first +1 is for completeness, second +1 is for ( _width % _grid_size != 0 )
	int grid_height = _height / _grid_size + 1 + 1;

	// pre-calculate every grid point's pseudorandom gradient
	////cv::Mat grid_pt_grad( grid_height, grid_width, CV_32FC2 );
	vector< math::Vector2d > grid_pt_grad( grid_width * grid_height );

	// initialize random seed
	srand( time( NULL ) );

	for( int j = 0; j < grid_height; j++ )
	{
		for( int i = 0; i < grid_width; i++ )
		{
			float degree = rand() % 36000 / 100.f;
			float x = cos( degree * M_PI / 180.f );
			float y = sin( degree * M_PI / 180.f );

			math::Vector2d grad( x, y );
			grid_pt_grad[ i + j * grid_width ] = grad;
		}
	}

	// TODO : pre_calculate grid_point-point vector in a grid


	// calculate perlin noise
	for( int j = 0; j < _height; j++ )
	{
		for( int i = 0; i < _width; i++ )
		{
			// calculate current grid index
			int gx = i / _grid_size;
			int gy = j / _grid_size;


			int local_x = i % _grid_size;
			int local_y = j % _grid_size;

			math::Vector2i v_p0( local_x, local_y );							// upper-left
			math::Vector2i v_p1( local_x - ( _grid_size - 1 ), local_y );		// upper-right
			math::Vector2i v_p2( local_x, local_y - ( _grid_size - 1 ) );		// lower-left
			math::Vector2i v_p3( local_x - ( _grid_size - 1 ), local_y - ( _grid_size - 1 ) );	// lower_right

			float dot_p0 = 	grid_pt_grad[ gx + gy * grid_width ].x * v_p0.x +
							grid_pt_grad[ gx + gy * grid_width ].y * v_p0.y;
			float dot_p1 = 	grid_pt_grad[ gx + 1 + gy * grid_width ].x	* v_p1.x +
							grid_pt_grad[ gx + 1 + gy * grid_width ].y * v_p1.y;
			float dot_p2 = 	grid_pt_grad[ gx + ( gy + 1 ) * grid_width ].x	* v_p2.x +
							grid_pt_grad[ gx + ( gy + 1 ) * grid_width ].y * v_p2.y;
			float dot_p3 = 	grid_pt_grad[ gx + 1 + ( gy + 1 ) * grid_width ].x * v_p3.x +
							grid_pt_grad[ gx + 1 + ( gy + 1 ) * grid_width ].y * v_p3.y;

			// interpolate
			float weight_x = s_shaped_curve( (float)local_x  / _grid_size );
			float weight_y = s_shaped_curve( (float)local_y / _grid_size );
			float interpolate_x0 = dot_p0 * ( 1 - weight_x ) + dot_p1 * weight_x;
			float interpolate_x1 = dot_p2 * ( 1 - weight_x ) + dot_p3 * weight_x;

			////noise.at< float >( j, i ) = interpolate_x0 * ( 1 - weight_y ) + interpolate_x1 * weight_y;
			noise[ i + j * _width ] = interpolate_x0 * ( 1 - weight_y ) + interpolate_x1 * weight_y;
		}
	}

	return noise;
}



#endif /* PERLIN_NOISE_H_ */
