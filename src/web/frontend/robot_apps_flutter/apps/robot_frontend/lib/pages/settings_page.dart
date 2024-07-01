import 'package:flutter/material.dart';
import 'package:robot_frontend/pages/config_page.dart';
import 'package:robot_frontend/pages/drawer_setup_page.dart';
import 'package:robot_frontend/pages/user_management_page.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class SettingsPage extends StatelessWidget {
  const SettingsPage({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Einstellungen',
      child: Padding(
        padding: const EdgeInsets.all(128),
        child: Row(
          children: [
            Expanded(
              child: Column(
                children: [
                  Expanded(
                    child: CustomButtonView(
                      text: 'Schubladen Setup',
                      onPressed: () {
                        Navigator.push(
                          context,
                          MaterialPageRoute<DrawerSetupPage>(
                            builder: (context) => const DrawerSetupPage(),
                          ),
                        );
                      },
                    ),
                  ),
                  Expanded(
                    child: CustomButtonView(
                      text: 'Netzwerk Setup',
                      onPressed: () {
                        Navigator.push(
                          context,
                          MaterialPageRoute<ConfigPage>(
                            builder: (context) => const ConfigPage(),
                          ),
                        );
                      },
                    ),
                  ),
                ],
              ),
            ),
            Expanded(
              child: Column(
                children: [
                  Expanded(
                    child: CustomButtonView(
                      text: 'User Management',
                      onPressed: () => Navigator.push(
                        context,
                        MaterialPageRoute<UserManagementPage>(
                          builder: (context) => const UserManagementPage(),
                        ),
                      ),
                    ),
                  ),
                  Expanded(
                    child: CustomButtonView(
                      text: 'Weitere Einstellungen',
                      onPressed: () {},
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
