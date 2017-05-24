#include <Ogre.h>

#include "ShadowSettings.h"

class RayConfRTListener: public Ogre::RenderTargetListener
{
public:
	RayConfRTListener( 	rendering::ScenePtr 	_scene,
						rendering::CameraPtr	_camera,
						Ogre::SceneManager		*_scene_mgr,
						Ogre::RenderTexture 	*_render_texture,
						float		 			*_rayconf_buffer,
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
	  	  m_rayconf_buffer( _rayconf_buffer ),
	  	  m_shadow_settings( _shadow_settings ),
	  	  m_turned_off_lights( _turned_off_lights ),
		  m_turned_off_mobj( _turned_off_mobj ),
		  m_cloned_entity( _cloned_entity ),
		  m_base_dist( _base_dist ),
		  SENSOR_IR_PROJECTOR_NAME_PREFIX( sensor_ir_projector_name_prefix )
	{

	}
	~RayConfRTListener()
	{
	}

	void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt )
	{
		// change material to RayConf for every model
		for( vector< Ogre::Entity * >::iterator iter = m_cloned_entity->begin(); iter != m_cloned_entity->end(); iter++ )
		{
			( *iter )->setMaterialName( "Ogre/DepthShadowmap_RayConf/BasicTemplateMaterial" );
		}
	}

	void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt )
	{
		_textureToPixmap();

		// restore gazebo's shadow settings
		_resetShadowSettings();
		// turn back on the light which should be on, turn off our IR Projector
		_resetLightSettings();
		// set Visual to their original materials
		_resetMaterials();
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
									m_rayconf_buffer );

		m_render_texture->copyContentsToMemory( pixelBox, Ogre::RenderTarget::FB_AUTO );
	}

	void _resetShadowSettings()
	{
		// restore gazebo's default shadow settings
		m_scene_mgr->setShadowTechnique( m_shadow_settings->shadow_tech );
		m_scene_mgr->setShadowTextureCount( m_shadow_settings->texture_count );
		m_scene_mgr->setShadowTextureCountPerLightType( Ogre::Light::LT_DIRECTIONAL, m_shadow_settings->texture_count_dir );
		m_scene_mgr->setShadowTextureCountPerLightType( Ogre::Light::LT_POINT, m_shadow_settings->texture_count_point );
		m_scene_mgr->setShadowTextureCountPerLightType( Ogre::Light::LT_SPOTLIGHT, m_shadow_settings->texture_count_spot );

		for( unsigned int i = 0; i < m_shadow_settings->texture_configs.size(); i++ )
		{
			m_scene_mgr->setShadowTextureConfig(	i,
													m_shadow_settings->texture_configs[i].width,
													m_shadow_settings->texture_configs[i].height,
													m_shadow_settings->texture_configs[i].format,
													m_shadow_settings->texture_configs[i].fsaa,
													m_shadow_settings->texture_configs[i].depthBufferPoolId );
		}

		m_scene_mgr->setShadowTextureSelfShadow( m_shadow_settings->self_shadow );
		m_scene_mgr->setShadowCasterRenderBackFaces( m_shadow_settings->render_back_faces );

		m_scene_mgr->setShadowDirectionalLightExtrusionDistance( m_shadow_settings->dir_light_extrusion_dist );
		m_scene_mgr->setShadowDirLightTextureOffset( m_shadow_settings->dir_light_texture_offset );
		m_scene_mgr->setShadowFarDistance( m_shadow_settings->far_dist );

		m_scene_mgr->setShadowTextureCasterMaterial( m_shadow_settings->caster_material );
	}

	void _resetLightSettings()
	{
		// turn on the gazebo's light ( turn on the light & visual representation )
		unsigned int num_lights = m_scene->GetLightCount();
		for( unsigned int i = 0; i < num_lights; i++ )
		{
			m_scene->GetLight( i )->ShowVisual( true );
		}


		// turn on the light being turned off manually
		for( vector<Ogre::Light *>::iterator iter = m_turned_off_lights->begin(); iter != m_turned_off_lights->end(); iter++ )
		{
			(*iter)->setVisible( true );
		}

		// turn off IR Projector
		m_scene_mgr->getLight( SENSOR_IR_PROJECTOR_NAME_PREFIX + m_camera->GetName() )->setVisible( false );
	}

	void _resetMaterials()
	{
		// turn on original visual
		for( vector< Ogre::MovableObject* >::iterator iter = m_turned_off_mobj->begin(); iter != m_turned_off_mobj->end(); iter++ )
		{
			( *iter )->setVisible( true );
		}
		// delete cloned visual
		for( vector< Ogre::Entity* >::iterator iter = m_cloned_entity->begin(); iter != m_cloned_entity->end(); iter++ )
		{
			(*iter)->getParentSceneNode()->detachObject( (*iter) );
			m_scene_mgr->destroyEntity( (*iter) );
		}
		m_cloned_entity->clear();
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
	// the buffer that contain the corresponding RenderTexture's content
	float *m_rayconf_buffer;
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
