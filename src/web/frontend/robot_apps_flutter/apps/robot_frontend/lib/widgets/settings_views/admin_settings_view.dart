import 'package:flutter/material.dart';
import 'package:robot_frontend/pages/config_page.dart';
import 'package:robot_frontend/pages/more_settings_page.dart';
import 'package:robot_frontend/pages/submodule_setup_page.dart';
import 'package:robot_frontend/pages/user_management_page.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';

class AdminSettingsView extends StatelessWidget {
  const AdminSettingsView({super.key});

  @override
  Widget build(BuildContext context) {
    return Row(
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
                      MaterialPageRoute<SubmodulesSetupPage>(
                        builder: (context) => const SubmodulesSetupPage(),
                      ),
                    );
                  },
                ),
              ),
              SizedBox(height: 16),
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
        SizedBox(width: 16),
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
              SizedBox(height: 16),
              Expanded(
                child: CustomButtonView(
                  text: 'Weitere Einstellungen',
                  onPressed: () => Navigator.push(
                    context,
                    MaterialPageRoute<MoreSettingsPage>(
                      builder: (context) => const MoreSettingsPage(),
                    ),
                  ),
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }
}
