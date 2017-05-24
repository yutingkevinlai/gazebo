#ifndef _STRUCT_SHADOWSETTINGS_
#define _STRUCT_SHADOWSETTINGS_

#include <vector>
#include <string>

#include <OGRE/Ogre.h>

struct ShadowSettings
{
	// save gazebo's default shadow settings
	Ogre::ShadowTechnique shadow_tech;
	unsigned int texture_count_dir;
	unsigned int texture_count_point;
	unsigned int texture_count_spot;
	unsigned int texture_count;
	std::vector< Ogre::ShadowTextureConfig > texture_configs;
	bool self_shadow;
	bool render_back_faces;
	Ogre::Real dir_light_extrusion_dist;
	Ogre::Real dir_light_texture_offset;
	Ogre::Real far_dist;
	// TODO can't get shadow texture caster material from scenemanager   ( see if can find a way
	std::string caster_material;

};

#endif //_STRUCT_SHADOWSETTINGS_
