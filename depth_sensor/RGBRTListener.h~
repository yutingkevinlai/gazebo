#include <Ogre.h>

#include <gazebo/rendering/rendering.hh>

using namespace gazebo;
using namespace std;

class RGBRTListener: public Ogre::RenderTargetListener
{
public:

	RGBRTListener( 	rendering::ScenePtr 	_scene,
						rendering::CameraPtr	_camera,
						Ogre::SceneManager		*_scene_mgr,
						Ogre::RenderTexture 	*_render_texture,
						unsigned char 			*_rgb_buffer,
						Ogre::Real				*_base_dist,
						Ogre::Light 			*_ir_projector )
		: m_scene( _scene ),
		  m_camera( _camera ),
		  m_scene_mgr( _scene_mgr ),
		  m_render_texture( _render_texture ),
		  m_rgb_buffer( _rgb_buffer ),
		  m_base_dist( _base_dist ),
		  m_ir_projector( _ir_projector )
	{
	}

	~RGBRTListener()
	{
	}

	void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
	{
		// turn on IR Projector, and turn off other lights
		_setLightSettings();
	}

	void postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
	{
		// save render texture to buffer
		_textureToPixmap();

		// turn back on the light which should be on, turn off our IR Projector
		_resetLightSettings();
	}

private:

	void _textureToPixmap()
	{
		// ******************************* //
		// convert RenderTexture to QImage //
		// ******************************* //

		Ogre::PixelBox pixelBox(	m_render_texture->getWidth(),
									m_render_texture->getHeight(),
									1,
									Ogre::PF_BYTE_RGB,
									m_rgb_buffer );

		m_render_texture->copyContentsToMemory( pixelBox, Ogre::RenderTarget::FB_AUTO );
	}

	void _setLightSettings()
	{
		// TODO: currently, specular map doesn't consider other light sources besides sensor ir projector

		// turn off the gazebo's light ( turn off the light & visual representation )
		unsigned int num_lights = m_scene->GetLightCount();
		for( unsigned int i = 0; i < num_lights; i++ )
		{
			m_scene->GetLight( i )->ShowVisual( false );
		}


		// turn off those light which is not created by gazebo,  except ours ( IR projector )
		// radius = m_base_dist, the shader only consider the nearest lights, so just disable the light which is closer than IR Projector
		m_turned_off_lights.clear();

		Ogre::HashedVector< Ogre::Light * > light_list;
		gazebo::math::Vector3 cam_pos = m_camera->GetWorldPosition();
		Ogre::Vector3 cam_pos_ogre( cam_pos.x, cam_pos.y, cam_pos.z );
		m_scene_mgr->_populateLightList( cam_pos_ogre, *m_base_dist, light_list );
		for( Ogre::HashedVector< Ogre::Light * >::iterator iter = light_list.begin(); iter != light_list.end(); iter++ )
		{
			if( (*iter)->isVisible() )
			{
				(*iter)->setVisible( false );
				m_turned_off_lights.push_back( (*iter) );
			}
		}

		// turn on IR Projector
		m_ir_projector->setVisible( true );

		// set settings for specular map
		m_ambient = m_scene_mgr->getAmbientLight();
		m_scene_mgr->setAmbientLight( Ogre::ColourValue::Black );
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
		for( vector<Ogre::Light *>::iterator iter = m_turned_off_lights.begin(); iter != m_turned_off_lights.end(); iter++ )
		{
			(*iter)->setVisible( true );
		}

		// turn off IR Projector
		m_ir_projector->setVisible( false );

		// restore settings
		m_scene_mgr->setAmbientLight( m_ambient );
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
	// the QImage that contain the corresponding RenderTexture's content
	unsigned char *m_rgb_buffer;
	// distance between IR camera & IR projector
	Ogre::Real *m_base_dist;
	// Sensor IR Projector as Ogre::Light
	Ogre::Light *m_ir_projector;
	// this vector contain lights being turned off by this class
	vector<Ogre::Light *>	m_turned_off_lights;

	// settings

	// ogre ambient light
	Ogre::ColourValue m_ambient;

};
