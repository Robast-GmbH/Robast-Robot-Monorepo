import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/models/provider/hygiene_provider.dart';
import 'package:robot_frontend/models/provider/inactivity_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/models/sidebar_menu_point.dart';
import 'package:robot_frontend/pages/disinfection_page.dart';
import 'package:robot_frontend/widgets/caire_ai_card_view.dart';
import 'package:robot_frontend/widgets/clock_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/home_views/patient_home_view.dart';
import 'package:robot_frontend/widgets/home_views/staff_home_view.dart';
import 'package:robot_frontend/widgets/info_views/patient_info_view.dart';
import 'package:robot_frontend/widgets/info_views/staff_info_view.dart';
import 'package:robot_frontend/widgets/notification_views/notification_view.dart';

import 'package:robot_frontend/widgets/robot_map_view.dart';
import 'package:robot_frontend/widgets/settings_views/admin_settings_view.dart';
import 'package:robot_frontend/widgets/settings_views/patient_settings_view.dart';
import 'package:robot_frontend/widgets/settings_views/staff_settings_view.dart';
import 'package:robot_frontend/widgets/sidebar.dart';
import 'package:shared_data_models/shared_data_models.dart';

class MenuPage extends StatefulWidget {
  const MenuPage({super.key});

  @override
  State<MenuPage> createState() => _MenuPageState();
}

class _MenuPageState extends State<MenuPage> {
  late final Future<User?> loadCurrentUserFuture;
  int selectedMainMenuIndex = 0;

  final sidebarMenuPoints = [
    SidebarMenuPoint(
      title: 'Home',
      icon: Icons.home,
      userGroupWidgets: {
        'ADMIN': StaffHomeView.new,
        'STAFF': StaffHomeView.new,
        'PATIENT': PatientHomeView.new,
      },
    ),
    if (kIsWeb)
      SidebarMenuPoint(
        title: 'caire.ai',
        icon: Icons.favorite,
        userGroupWidgets: {
          'ADMIN': CaireAiCardView.new,
          'STAFF': CaireAiCardView.new,
          'PATIENT': CaireAiCardView.new,
        },
      ),
    SidebarMenuPoint(
      title: 'Karte',
      icon: Icons.map,
      userGroupWidgets: {
        'ADMIN': RobotMapView.new,
        'STAFF': RobotMapView.new,
        'PATIENT': RobotMapView.new,
      },
    ),
    SidebarMenuPoint(
      title: 'Infos',
      icon: Icons.info_outline,
      userGroupWidgets: {
        'ADMIN': StaffInfoView.new,
        'STAFF': StaffInfoView.new,
        'PATIENT': PatientInfoView.new,
      },
    ),
    SidebarMenuPoint(
      title: 'Meldungen',
      icon: Icons.notifications,
      userGroupWidgets: {
        'ADMIN': NotificationView.new,
        'STAFF': NotificationView.new,
        'PATIENT': NotificationView.new,
      },
    ),
    SidebarMenuPoint(
      title: 'Einstellungen',
      icon: Icons.settings,
      userGroupWidgets: {
        'ADMIN': AdminSettingsView.new,
        'STAFF': StaffSettingsView.new,
        'PATIENT': PatientSettingsView.new,
      },
    ),
  ];

  Future<User?> loadCurrentUser() async {
    // await Provider.of<UserProvider>(context, listen: false).setUserSession(robotName: 'rb_theron', userID: 'a1f26ade-d83a-414a-aaae-62366e0c083c');
    return Provider.of<UserProvider>(context, listen: false).getUserSession(robotName: 'rb_theron');
  }

  @override
  void initState() {
    super.initState();
    loadCurrentUserFuture = loadCurrentUser();
  }

  @override
  deactivate() {
    Provider.of<InactivityProvider>(context, listen: false).cancelTimer();
    Provider.of<RobotProvider>(context, listen: false).unblockNavigation();
    Provider.of<UserProvider>(context, listen: false).endUserSession(robotName: 'rb_theron');
    super.deactivate();
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      collapsedTitle: true,
      child: FutureBuilder<User?>(
        future: loadCurrentUserFuture,
        builder: (context, snapshot) {
          if (snapshot.connectionState != ConnectionState.done) {
            return const SizedBox.expand(child: Center(child: CircularProgressIndicator()));
          }
          final user = snapshot.data;
          return Row(
            children: [
              Sidebar(
                sidebarMenuPoints: sidebarMenuPoints,
                onMenuPointSelected: (index) => setState(() {
                  selectedMainMenuIndex = index;
                }),
                user: user,
              ),
              Expanded(
                child: Padding(
                  padding: const EdgeInsets.only(top: 16, right: 16, bottom: 16),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Stack(
                        children: [
                          const Padding(
                            padding: EdgeInsets.only(top: 12),
                            child: Align(
                              child: ClockView(),
                            ),
                          ),
                          Row(
                            mainAxisAlignment: MainAxisAlignment.spaceBetween,
                            children: [
                              Padding(
                                padding: const EdgeInsets.only(left: 16),
                                child: Text(
                                  sidebarMenuPoints[selectedMainMenuIndex].title,
                                  style: const TextStyle(color: RobotColors.primaryText, fontSize: 40, fontWeight: FontWeight.w400),
                                ),
                              ),
                              InkWell(
                                onTap: () {
                                  Navigator.pop(context);
                                  final hygieneProvider = Provider.of<HygieneProvider>(context, listen: false);
                                  if (hygieneProvider.requiresDisinfectionAfterUsage) {
                                    Navigator.push(context, MaterialPageRoute<void>(builder: (context) => const DisinfectionPage()));
                                  }
                                },
                                child: Padding(
                                  padding: const EdgeInsets.only(bottom: 16),
                                  child: Container(
                                    decoration: BoxDecoration(color: Colors.black.withOpacity(0.5), borderRadius: BorderRadius.circular(16)),
                                    child: const Padding(
                                      padding: EdgeInsets.symmetric(horizontal: 32, vertical: 8),
                                      child: Row(
                                        children: [
                                          Text(
                                            'Abmelden',
                                            style: TextStyle(color: RobotColors.primaryText, fontSize: 32),
                                          ),
                                          SizedBox(
                                            width: 16,
                                          ),
                                          Icon(
                                            Icons.logout,
                                            size: 40,
                                          ),
                                        ],
                                      ),
                                    ),
                                  ),
                                ),
                              ),
                            ],
                          ),
                        ],
                      ),
                      Container(
                        margin: const EdgeInsets.symmetric(horizontal: 8),
                        width: double.infinity,
                        height: 2,
                        decoration: BoxDecoration(
                          borderRadius: BorderRadius.circular(8),
                          color: Colors.black.withOpacity(0.1),
                        ),
                      ),
                      Expanded(
                        child: Padding(
                          padding: const EdgeInsets.only(top: 16),
                          child: sidebarMenuPoints[selectedMainMenuIndex].userGroupWidgets[user!.userGroups.last]!(),
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ],
          );
        },
      ),
    );
  }
}
