namespace sm_drawer_gate
{
// STATE DECLARATION
struct StLoopDrawerAccess : smacc2::SmaccState<StLoopDrawerAccess, MsRun>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct ReceivedMsg : SUCCESS{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvTopicMessage<CbOpenDrawerSubscriber, OrDrawerControl>, SsDrawerAccess, ReceivedMsg>,

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrDrawerControl, CbOpenDrawerSubscriber>();
  }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_drawer_gate
