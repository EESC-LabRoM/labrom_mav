#ifndef BLIND_LANDING_CLIENT_H_
#define BLIND_LANDING_CLIENT_H_

// landing libraries
#include <blind_action/blind.h>
#include <blind_action/LandingAction.h>

// Actionlib libraries
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

namespace blind{
namespace landing{
//! Class LandingClient scope
class LandingClient{
  public:
    //! Constructor
    LandingClient(void);
    //! Empty Destructor 
    ~LandingClient(void);
    //! Set Goal
    void SetGoal(double landing_accel = 10.0);
    //! Send Goal
    bool SendGoal(double timeout = 2.0);
    
  private:
    blind_action::LandingGoal goal_;           //!< Goal
};

} // landing namespace
} // blind namespace

#endif //BLIND_LANDING_CLIENT_H_
