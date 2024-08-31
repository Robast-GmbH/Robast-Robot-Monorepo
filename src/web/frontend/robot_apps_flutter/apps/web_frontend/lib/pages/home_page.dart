import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/user_provider.dart';
import 'package:web_frontend/pages/config_page.dart';
import 'package:web_frontend/pages/fleet_management_page.dart';
import 'package:web_frontend/pages/user_management_page.dart';
import 'package:web_frontend/pages/user_settings_page.dart';
import 'package:web_frontend/widgets/rounded_button.dart';

class HomePage extends StatelessWidget {
  const HomePage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Row(
          children: [
            Icon(Icons.home),
            SizedBox(
              width: 8,
            ),
            const Text('Hauptmenü'),
          ],
        ),
        actions: [
          IconButton(
            onPressed: () {
              Navigator.push(
                context,
                MaterialPageRoute<UserSettingsPage>(
                  builder: (context) => UserSettingsPage(),
                ),
              );
            },
            icon: const Icon(Icons.person),
          ),
          IconButton(
            onPressed: () {
              Navigator.push(
                context,
                MaterialPageRoute<ConfigPage>(
                  builder: (context) => const ConfigPage(),
                ),
              );
            },
            icon: const Icon(Icons.settings),
          ),
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: ListView(
          children: [
            RoundedButton(
              onPressed: () {
                Navigator.push(
                  context,
                  MaterialPageRoute<FleetManagementPage>(
                    builder: (context) => const FleetManagementPage(),
                  ),
                );
              },
              child: Padding(
                padding: const EdgeInsets.all(16),
                child: Text(
                  'Flottenmanagement',
                  style: TextStyle(color: Colors.white, fontSize: 24),
                ),
              ),
            ),
            if (Provider.of<UserProvider>(context).user!.userGroups.contains('ADMIN')) ...[
              SizedBox(height: 16),
              RoundedButton(
                onPressed: () {
                  Navigator.push(
                    context,
                    MaterialPageRoute<UserManagementPage>(
                      builder: (context) => const UserManagementPage(),
                    ),
                  );
                },
                child: Padding(
                  padding: const EdgeInsets.all(16),
                  child: Text(
                    'Nutzermanagement',
                    style: TextStyle(color: Colors.white, fontSize: 24),
                  ),
                ),
              ),
            ]
          ],
        ),
      ),
    );
  }
}
