#include <Ogre.h>

#include <gazebo/rendering/rendering.hh>

#include "ShadowSettings.h"

using namespace gazebo;
using namespace std;

class SegmentRTListener: public Ogre::RenderTargetListener
{
public:
	SegmentRTListener( 	rendering::ScenePtr 	_scene,
						Ogre::SceneManager		*_scene_mgr,
						Ogre::RenderTexture 	*_render_texture,
						unsigned char 			*_segment_buffer )
		: m_scene( _scene ),
		  m_scene_mgr( _scene_mgr ),
		  m_render_texture( _render_texture ),
		  m_segment_buffer( _segment_buffer ),
		  m_shadow_settings(),
		  m_turned_off_lights(),
		  m_turned_off_mobj(),
		  m_cloned_entity()
	{

	}
	~SegmentRTListener()
	{
	}

	void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt )
	{
		// set materials for all objects
		_toggleMaterials( true );
	}

	void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt )
	{
		_textureToPixmap();

		// set materials for all objects
		_toggleMaterials( false );
	}

private:
	void _textureToPixmap()
	{
		// ******************************* //
		// save data in to buffer //
		// ******************************* //
		unsigned int width = m_render_texture->getWidth();
		unsigned int height = m_render_texture->getHeight();

		Ogre::PixelBox pixelBox(	width,
									height,
									1,
									Ogre::PF_BYTE_RGB,
									m_segment_buffer );

		m_render_texture->copyContentsToMemory( pixelBox, Ogre::RenderTarget::FB_AUTO );
	}

	void _toggleMaterials( bool _in_pre )
	{
		if( _in_pre )
		{
			// **************** //
			// count all models //
			// **************** //
			vector< string > model_names;

			unsigned int num_visual = m_scene->GetVisualCount();
			// go through every visual in rendering::Scene
			for( unsigned int i = 1; i < num_visual+10; i++ )	// skip the first one, or crash		// +10 is due to bug in GetVisualCount(), it return less value than actual value.
			{
				rendering::VisualPtr cur_visual = m_scene->GetVisual( i );
				if( cur_visual )
				{
					// get only model names
					string model_name = cur_visual->GetName();
					size_t found_pos = model_name.rfind( "::" );
					if( found_pos != string::npos )
					{
						continue;
					}

					if( find( model_names.begin(), model_names.end(), model_name ) == model_names.end() )
					{
						model_names.push_back( model_name );
					}
					else	// should not happen
					{
						cout << "ERROR : u have two same visual name!" << endl;
					}
				}
			}
			// ****************************** //
			// create material for each model //
			// ****************************** //

			// materials for every models, the order is the same with correspondence model
			vector< Ogre::MaterialPtr > model_materials;

			// get material manager
			Ogre::MaterialManager& material_manager = Ogre::MaterialManager::getSingleton();

			for( unsigned int i = 0; i < model_names.size(); i++ )
			{
				std::stringstream ss;
				ss << i;

				Ogre::MaterialPtr material = material_manager.create( 	"segment_material_" + ss.str(),
																		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );

				Ogre::Pass* pass = material->getTechnique( 0 )->getPass( 0 );

				float color = ( i + 1.0f ) / model_names.size();

				// Emissive / self illumination is the color 'produced' by the object.
				pass->setSelfIllumination( Ogre::ColourValue( color, color, color ) );
				// diffuse color is the traditionnal color of the lit object.
				pass->setDiffuse( Ogre::ColourValue( 0.0f, 0.0f, 0.0f, 0.0f ) );
				// ambient colour is linked to ambient lighting.
				pass->setAmbient( Ogre::ColourValue( 0.0f, 0.0f, 0.0f, 0.0f ) );

				model_materials.push_back( material );
			}

			// ***************************** //
			// clone entity and set material //
			// ***************************** //
			m_turned_off_mobj.clear();
			m_cloned_entity.clear();

			// this vector contains scene node that cloned Entity is going to attached to
			vector< Ogre::SceneNode* > cloned_scene_node;

			num_visual = m_scene->GetVisualCount();
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
											m_cloned_entity.push_back( cloned_entity );
											cloned_scene_node.push_back( entity->getParentSceneNode() );

											// set it's material
											int pos = entity_name.find( "::" );

											vector< string >::iterator found_pos = find( 	model_names.begin(),
																							model_names.end(),
																							entity_name.substr( 7, pos - 7 ) );

											if( found_pos != model_names.end() )
											{
												int idx = found_pos - model_names.begin();
												cloned_entity->setMaterial( model_materials[ idx ] );
												cloned_entity->setCastShadows( false );
											}
											else
											{
												cout << "ERROR: weird result." << endl;
												cout << "not found model name : " << entity_name.substr( 7, pos - 7 ) << endl;
												exit( -1 );
											}
										}
										entity->setVisible( false );
										m_turned_off_mobj.push_back( mobj );

									}
									else if( sr )	// Ogre::SimpleRenderable
									{
										sr->setVisible( false );
										m_turned_off_mobj.push_back( mobj );
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
			for( vector< Ogre::Entity* >::iterator iter = m_cloned_entity.begin(); iter != m_cloned_entity.end(); iter++, i++ )
			{
				cloned_scene_node[i]->attachObject( (*iter) );
			}
		}
		else
		{
			// turn on original visual
			for( vector< Ogre::MovableObject* >::iterator iter = m_turned_off_mobj.begin(); iter != m_turned_off_mobj.end(); iter++ )
			{
				( *iter )->setVisible( true );
			}
			// delete cloned visual
			for( vector< Ogre::Entity* >::iterator iter = m_cloned_entity.begin(); iter != m_cloned_entity.end(); iter++ )
			{
				(*iter)->getParentSceneNode()->detachObject( (*iter) );
				m_scene_mgr->destroyEntity( (*iter) );
			}
			m_cloned_entity.clear();
		}
	}
public:

private:
	// the pointer to gazebo::rendering::Scene that contain the listened camera
	rendering::ScenePtr m_scene;
	// Ogre::SceneManager
	Ogre::SceneManager *m_scene_mgr;
	// the render texture that this listener is listening to
	Ogre::RenderTexture *m_render_texture;
	// this buffer contain the corresponding RenderTexture's content
	unsigned char *m_segment_buffer;
	// the instance contain gazebo's shadow settings
	ShadowSettings m_shadow_settings;
	// this vector contain lights being turned off by this class
	vector<Ogre::Light *> m_turned_off_lights;
	// this vector contain MovableObject being hidden by depth RT listener
	vector< Ogre::MovableObject * > m_turned_off_mobj;
	// this vcetor contain Entity being cloned for depth simulator
	vector< Ogre::Entity * > m_cloned_entity;
};
