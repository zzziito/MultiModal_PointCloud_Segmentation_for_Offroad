#include <gazebo/gazebo.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Spline.hh>

#include <ignition/math/Spline.hh>

#include <iostream>
#include <optional>
#include <algorithm>

namespace gazebo
{
  class SinWavePlugin : public WorldPlugin
  {
  public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
      std::cout << "Sin wave plugin loaded." << std::endl;

      // Get the model name from the SDF element, or use the default name "ground_plane"
      std::string modelName = _sdf->Get<std::string>("model_name", "ground_plane");
      // Get the pointer to the model with the specified name
      physics::ModelPtr model = _world->ModelByName(modelName);

      // Check if the model exists
      if (!model)
      {
        std::cerr << "Model " << modelName << " not found!" << std::endl;
        return;
      }

      // Get the pointer to the model's link
      physics::LinkPtr link = model->GetLink("link");

      // Check if the link exists
      if (!link)
      {
        std::cerr << "Link not found!" << std::endl;
        return;
      }

      // Create the sine wave function with amplitude and wavelength parameters
      double amplitude = _sdf->Get<double>("amplitude", 0.1);
      double wavelength = _sdf->Get<double>("wavelength", 1.0);
      ignition::math::Spline spline;
      spline.AddPoint(0.0, 0.0);
      spline.AddPoint(wavelength / 4.0, amplitude);
      spline.AddPoint(wavelength / 2.0, 0.0);
      spline.AddPoint(3.0 * wavelength / 4.0, -amplitude);
      spline.AddPoint(wavelength, 0.0);
      spline.AutoCalculate();

      // Get the pointer to the physics engine
      physics::PhysicsEnginePtr physicsEngine = _world->Physics();

      // Apply the sine wave function to the link's vertical position
      physics::JointPtr joint = link->GetJoint("joint");
      if (joint)
      {
        physicsEngine->SetGravity(math::Vector3(0, 0, 0));
        double period = wavelength / physicsEngine->GetMaxStepSize();
        double timeOffset = _sdf->Get<double>("time_offset", 0.0);
        auto updateFunc = [joint, spline, period, timeOffset](const common::UpdateInfo& _info) {
          double time = _info.simTime.Double() + timeOffset;
          double position = spline.Eval(std::fmod(time, period));
          joint->SetPosition(0, position);
        };
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(updateFunc);
      }
      else
      {
        std::cerr << "Joint not found!" << std::endl;
      }
    }

  private:
    event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_WORLD_PLUGIN(SinWavePlugin)
}