#include <gz/rendering/Actor.hh>
#include <gz/rendering/Scene.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

using namespace gz;

class PauseActorRenderPlugin : public rendering::System
{
private:
  bool initialized = false;
  bool foundActor = false;

public:
  void Update(rendering::ScenePtr _scene, double /*_dt*/) override
  {
    if (!_scene) return;

    // Try to find actor visual on first update
    static rendering::ActorPtr actor;
    if (!foundActor) {
      // Try several candidate names; model naming may differ
      std::vector<std::string> candidates = {
        "actor_walking", "actor1", "actor", 
        "default::actor1", "actor1::actor"
      };
      
      for (auto &name : candidates) {
        auto vis = _scene->VisualByName(name);
        if (vis) {
          actor = std::dynamic_pointer_cast<rendering::Actor>(vis);
          if (actor) {
            gzmsg << "PauseActorRenderPlugin: Found actor with name: " << name << std::endl;
            foundActor = true;
            break;
          }
        }
      }
      
      if (!foundActor) {
        // Try iterating all nodes in scene
        for (unsigned int i = 0; i < _scene->NodeCount(); ++i) {
          auto node = _scene->NodeByIndex(i);
          auto actorNode = std::dynamic_pointer_cast<rendering::Actor>(node);
          if (actorNode) {
            actor = actorNode;
            gzmsg << "PauseActorRenderPlugin: Found actor by iteration" << std::endl;
            foundActor = true;
            break;
          }
        }
      }
      
      if (!initialized && !foundActor) {
        gzwarn << "PauseActorRenderPlugin: Actor not found yet, will keep trying..." << std::endl;
        initialized = true;
      }
    }

    if (!actor) return;

    // Default to paused (animation frozen at startup)
    static bool paused = true;
    
    // Freeze skeleton by setting animation time to zero
    // This is more reliable than trying to find a SetAnimationSpeed method
    if (paused) {
      // Keep animation at time=0 to freeze it
      actor->SetAnimationTime(std::chrono::steady_clock::duration::zero());
    }
    // When not paused, don't touch animation time - let it advance naturally
  }
};

GZ_ADD_PLUGIN(PauseActorRenderPlugin, rendering::System)
