/*
 * DepthRTListener.h
 *
 *  Created on: Feb 15, 2017
 *      Author: kevin
 */

#include <Ogre.h>

#include <gazebo/rendering/rendering.hh>

#include "ShadowSettings.h"

using namespace gazebo;
using namespace std;

class DepthRTListener: public Ogre::RenderTargetListener
{
public:
	DepthRTListener( 	rendering::ScenePtr 	_scene,
						rendering::CameraPtr	_camera,
						Ogre::SceneManager		*_scene_mgr,
						Ogre::RenderTexture 	*_render_texture,
						float		 			*_depth_buffer,
						ShadowSettings			*_shadow_settings,
						vector< Ogre::Light * >	*_turned_off_lights,
						vector< Ogre::MovableObject * >	*_turned_off_mobj,
						vector< Ogre::Entity * >	*_cloned_entity,
						Ogre::Real				*_base_dist,
						const string			sensor_ir_projector_name_prefix )
		: m_scene( _scene ),
		  m_camera( _camera ),
		  m_scene_mgr( _scene_mgr ),
		  m_render_texture( _render_texture ),
		  m_depth_buffer( _depth_buffer ),
		  m_shadow_settings( _shadow_settings ),
		  m_turned_off_lights( _turned_off_lights ),
		  m_turned_off_mobj( _turned_off_mobj ),
		  m_cloned_entity( _cloned_entity ),
		  m_base_dist( _base_dist ),
		  SENSOR_IR_PROJECTOR_NAME_PREFIX( sensor_ir_projector_name_prefix )
	{

	}
	~DepthRTListener()
	{
	}

	void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt )
	{
		// set shadow settings
		_setShadowSettings();
		// turn on IR Projector, and turn off other lights
		_setLightSettings();
		// set materials for all objects
		_setMaterials();
	}

	void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt )
	{
		_textureToPixmap();
	}

private:
	void _textureToPixmap()
	{
		// ******************************* //
		// convert RenderTexture to QImage //
		// ******************************* //
		unsigned int width = m_render_texture->getWidth();
		unsigned int height = m_render_texture->getHeight();

		Ogre::PixelBox pixelBox(	width,
									height,
									1,
									Ogre::PF_FLOAT32_RGBA,
									m_depth_buffer );

		m_render_texture->copyContentsToMemory( pixelBox, Ogre::RenderTarget::FB_AUTO );


	}

	void _setShadowSettings()
	{
		// ************************************ //
		// get gazebo's default shadow settings //
		// ************************************ //
		m_shadow_settings->shadow_tech = 			m_scene_mgr->getShadowTechnique();
		m_shadow_settings->texture_count_dir = 		m_scene_mgr->getShadowTextureCountPerLightType( Ogre::Light::LT_DIRECTIONAL );
		m_shadow_settings->texture_count_point = 	m_scene_mgr->getShadowTextureCountPerLightType( Ogre::Light::LT_POINT );
		m_shadow_settings->texture_count_spot = 	m_scene_mgr->getShadowTextureCountPerLightType( Ogre::Light::LT_SPOTLIGHT );
		m_shadow_settings->texture_count = 			m_scene_mgr->getShadowTextureCount();
		Ogre::ConstShadowTextureConfigIterator config_iter = m_scene_mgr->getShadowTextureConfigIterator();
		m_shadow_settings->texture_configs.clear();

		while( config_iter.current() != config_iter.end() )
		{
			Ogre::ShadowTextureConfig config;
			config.width = config_iter.current()->width;
			config.height = config_iter.current()->height;
			config.format = config_iter.current()->format;
			config.fsaa = config_iter.current()->fsaa;
			config.depthBufferPoolId = config_iter.current()->depthBufferPoolId;

			m_shadow_settings->texture_configs.push_back( config );

			if( !config_iter.hasMoreElements() )
			{
				break;
			}
			config_iter.moveNext();
		}

		m_shadow_settings->self_shadow = 				m_scene_mgr->getShadowTextureSelfShadow();
		m_shadow_settings->render_back_faces = 			m_scene_mgr->getShadowCasterRenderBackFaces();
		m_shadow_settings->dir_light_extrusion_dist =	m_scene_mgr->getShadowDirectionalLightExtrusionDistance();
		m_shadow_settings->dir_light_texture_offset =	m_scene_mgr->getShadowDirLightTextureOffset();
		m_shadow_settings->far_dist =					m_scene_mgr->getShadowFarDistance();
		// TODO can't get shadow texture caster material from scenemanager   ( see if can find a way
		m_shadow_settings->caster_material = 			"Gazebo/shadow_caster";

		// **************************************** //
		// set shadow settings for depth shadow map //
		// **************************************** //

		// Finally enable the shadows using texture additive integrated
		m_scene_mgr->setShadowTechnique( Ogre::SHADOWTYPE_TEXTURE_ADDITIVE_INTEGRATED) ;
		// Set the pixel format to floating point
		m_scene_mgr->setShadowTexturePixelFormat( Ogre::PF_FLOAT32_R );
		// Allow self shadowing (note: this only works in conjunction with the shaders defined above)
		m_scene_mgr->setShadowTextureSelfShadow( true );
		// Set the caster material which uses the shaders defined above
		m_scene_mgr->setShadowTextureCasterMaterial( "Ogre/DepthShadowmap/Caster/Float" );
		// You can switch this on or off, I suggest you try both and see which works best for you
		m_scene_mgr->setShadowCasterRenderBackFaces( false );
		// bigger texture size, more smooth, see http://www.ogre3d.org/docs/manual/manual_72.html
		m_scene_mgr->setShadowTextureSize( 2048 );


		// reset the shadow setting that GAZEBO modified in RTShaderSystem
		m_scene_mgr->setShadowTextureCountPerLightType( Ogre::Light::LT_DIRECTIONAL, 0 );	// this one is essential for shadowmap
		m_scene_mgr->setShadowTextureCountPerLightType( Ogre::Light::LT_POINT, 0 );	// this one is essential for shadowmap
		m_scene_mgr->setShadowTextureCountPerLightType( Ogre::Light::LT_SPOTLIGHT, 1 );	// this one is essential for shadowmap
		m_scene_mgr->setShadowTextureCount( 1 );
		m_scene_mgr->setShadowTextureConfig(0, 2048, 2048, Ogre::PF_FLOAT32_R);
		m_scene_mgr->setShadowDirectionalLightExtrusionDistance( 10000 );	// this is ogre's default
		m_scene_mgr->setShadowDirLightTextureOffset( 0.6 );					// this is ogre's default
		m_scene_mgr->setShadowFarDistance( 0 );								// this is ogre's default
	}

	void _setLightSettings()
	{
		// turn off the gazebo's light ( turn off the light & visual representation )
		unsigned int num_lights = m_scene->GetLightCount();
		for( unsigned int i = 0; i < num_lights; i++ )
		{
			m_scene->GetLight( i )->ShowVisual( false );
		}


		// turn off those light which is not created by gazebo,  except ours ( IR projector )
		// radius = m_base_dist, the shader only consider the nearest lights, so just disable the light which is closer than IR Projector
		m_turned_off_lights->clear();

		Ogre::HashedVector< Ogre::Light * > light_list;
		gazebo::math::Vector3 cam_pos = m_camera->GetWorldPosition();
		Ogre::Vector3 cam_pos_ogre( cam_pos.x, cam_pos.y, cam_pos.z );
		m_scene_mgr->_populateLightList( cam_pos_ogre, *m_base_dist, light_list );
		for( Ogre::HashedVector< Ogre::Light * >::iterator iter = light_list.begin(); iter != light_list.end(); iter++ )
		{
			if( (*iter)->isVisible() )
			{
				(*iter)->setVisible( false );
				m_turned_off_lights->push_back( (*iter) );
			}
		}

		// turn on IR Projector
		m_scene_mgr->getLight( SENSOR_IR_PROJECTOR_NAME_PREFIX + m_camera->GetName() )->setVisible( true );
	}

	void _setMaterials()
	{
		m_turned_off_mobj->clear();
		m_cloned_entity->clear();

		// this vector contains scene node that cloned Entity is going to attached to
		vector< Ogre::SceneNode* > cloned_scene_node;

		unsigned int num_visual = m_scene->GetVisualCount();
		// go through every visual in rendering::Scene
		for( unsigned int i = 1; i < num_visual+10; i++ )	// skip the first one, or crash		// +10 is due to bug in GetVisualCount(), it return less value than actual value.
		{
			rendering::VisualPtr cur_visual = m_scene->GetVisual( i );
			if( cur_visual )
			{
				// only visual with NO "_MATERIAL_" in it's Visual material name will pass through
				string material_name = cur_visual->GetMaterialName();
				size_t found_pos = material_name.rfind( "_MATERIAL_" );
				if( found_pos != string::npos )
				{
					continue;
				}

				// clone the visual if it's visible
				if( cur_visual->GetVisible() )
				{
					Ogre::SceneNode *scene_node = cur_visual->GetSceneNode();

					// Apply material to all child scene nodes
					for( unsigned int i = 0; i < scene_node->numChildren(); ++i )
					{
						Ogre::SceneNode *sn = (Ogre::SceneNode*)(scene_node->getChild(i));
						for (int j = 0; j < sn->numAttachedObjects(); j++)
						{
							Ogre::MovableObject *mobj = sn->getAttachedObject( j );
							Ogre::Entity *entity = dynamic_cast<Ogre::Entity*>( mobj );
							Ogre::SimpleRenderable *sr = dynamic_cast<Ogre::SimpleRenderable*>( mobj );
							if( mobj->getVisible() )
							{
								if( entity )
								{
									// clone only model visualize object ( those dosen't have "__COLLISION_VISUAL__" in it's entity name
									string entity_name = entity->getName();
									size_t found_pos = entity_name.rfind( "__COLLISION_VISUAL__" );
									if( found_pos == string::npos )
									{
										Ogre::Entity *cloned_entity = entity->clone( "CLONED_ENTITY_" + entity->getName() );
										m_cloned_entity->push_back( cloned_entity );
										cloned_scene_node.push_back( entity->getParentSceneNode() );
										cloned_entity->setMaterialName( "Ogre/DepthShadowmap_Depth/BasicTemplateMaterial" );
										cloned_entity->setCastShadows( true );
									}
									entity->setVisible( false );
									m_turned_off_mobj->push_back( mobj );

								}
								else if( sr )	// Ogre::SimpleRenderable
								{
									sr->setVisible( false );
									m_turned_off_mobj->push_back( mobj );
								}
								else	// not sure
								{
									cerr << "Something didn't handled in DepthRTListener!" << endl;
								}
							}
						}
					}
				}
			}
		}
		// attach cloned Entity to it's SceneNode	( cloned entity are not attach to SceneNode, thus not visible
		unsigned int i = 0;
		for( vector< Ogre::Entity* >::iterator iter = m_cloned_entity->begin(); iter != m_cloned_entity->end(); iter++, i++ )
		{
			cloned_scene_node[i]->attachObject( (*iter) );
		}
	}
public:

private:
	// the pointer to gazebo::rendering::Scene that contain the listened camera
	rendering::ScenePtr m_scene;
	// gazebo::rendering::camera pointer that this class is listening to
	rendering::CameraPtr m_camera;
	// Ogre::SceneManager
	Ogre::SceneManager *m_scene_mgr;
	// the render texture that this listener is listening to
	Ogre::RenderTexture *m_render_texture;
	// this buffer that contain the corresponding RenderTexture's content
	float *m_depth_buffer;
	// the instance contain gazebo's shadow settings
	ShadowSettings *m_shadow_settings;
	// this vector contain lights being turned off by this class
	vector<Ogre::Light *>	*m_turned_off_lights;
	// this vector contain MovableObject being hidden by depth RT listener
	vector< Ogre::MovableObject * > *m_turned_off_mobj;
	// this vcetor contain Entity being cloned for depth simulator
	vector< Ogre::Entity * > *m_cloned_entity;
	// distance between IR camera & IR projector
	Ogre::Real *m_base_dist;
	//
	const std::string SENSOR_IR_PROJECTOR_NAME_PREFIX;
};
