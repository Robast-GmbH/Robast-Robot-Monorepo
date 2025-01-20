import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/inactivity_provider.dart';
import 'package:robot_frontend/pages/setting_pages/config_page.dart';
import 'package:robot_frontend/pages/module_pages/modules_setup_page.dart';
import 'package:robot_frontend/pages/setting_pages/more_settings_page.dart';
import 'package:robot_frontend/pages/setting_pages/user_management_page.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';

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
                  text: 'Modul Setup',
                  onPressed: () async {
                    final inactivityProvider = Provider.of<InactivityProvider>(context, listen: false);
                    inactivityProvider.cancelTimer();
                    await Navigator.push(
                      context,
                      MaterialPageRoute<ModulesSetupPage>(
                        builder: (context) => const ModulesSetupPage(),
                      ),
                    );
                    inactivityProvider.resetInactivityTimer();
                  },
                ),
              ),
              const SizedBox(height: 16),
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
        const SizedBox(width: 16),
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
              const SizedBox(height: 16),
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
