/*
 * evaluation_criteria.h
 *
 *  Created on: Feb 15, 2017
 *      Author: kevin
 */

#include <gazebo/math/gzmath.hh>

#ifndef EVALUATION_CRITERIA_H_
#define EVALUATION_CRITERIA_H_

struct EvaluationCriteria
{

	float translation_threshold;
	float quaternion_degree_threshold;
	// rotational symmetry
	bool has_rotational_symmetry;
	std::vector< gazebo::math::Vector3 > rot_sym_axes;
	std::vector< int > rot_sym_order;
	std::vector< float > rot_sym_tolerance_degree;
	std::vector< float > rot_sym_axis_deviation_degree;
	// circular symmetry
	bool has_circular_symmetry;
	gazebo::math::Vector3 cir_sym_axis;
	float cir_sym_axis_deviation_degree;
	// cylinder like model
	bool is_cylinder_like;
	gazebo::math::Vector3 cylinder_axis;
	float cylinder_axis_deviation_threshold;

};

std::ostream &operator<<( std::ostream &_out, const EvaluationCriteria &_criteria )
{
	_out << "Criteria:" << std::endl;

	_out << "translation threshold : " << _criteria.translation_threshold << std::endl;
	_out << "quaternion degree_threshold : " << _criteria.quaternion_degree_threshold << std::endl;

	_out << "has rotational symmetry : " << _criteria.has_rotational_symmetry << std::endl;
	if( _criteria.has_rotational_symmetry )
	{
		for( uint i = 0; i < _criteria.rot_sym_axes.size(); i++ )
		{
			_out << "\t" << i << std::endl;
			_out << "\t" << "axis: " << _criteria.rot_sym_axes[ i ] << std::endl;
			_out << "\t" << "order: " << _criteria.rot_sym_order[ i ] << std::endl;
			_out << "\t" << "tolerance degree: " << _criteria.rot_sym_tolerance_degree[ i ] << std::endl;
			_out << "\t" << "axis deviation degree: " << _criteria.rot_sym_axis_deviation_degree[ i ] << std::endl;
		}
	}

	_out << "has circular symmetry : " << _criteria.has_circular_symmetry << std::endl;
	if( _criteria.has_circular_symmetry )
	{
		_out << "\t" << "axis : " << _criteria.cir_sym_axis << std::endl;
		_out << "\t" << "axis deviation degree : " << _criteria.cir_sym_axis_deviation_degree << std::endl;
	}

	_out << "is cylinder like evaluation criteria" << _criteria.is_cylinder_like << std::endl;
	if( _criteria.is_cylinder_like )
	{
		_out << "\t" << "axis : " << _criteria.cylinder_axis << std::endl;
		_out << "\t" << "axis deviation degree : " << _criteria.cylinder_axis_deviation_threshold << std::endl;
	}

	return _out;
}


#endif /* EVALUATION_CRITERIA_H_ */
