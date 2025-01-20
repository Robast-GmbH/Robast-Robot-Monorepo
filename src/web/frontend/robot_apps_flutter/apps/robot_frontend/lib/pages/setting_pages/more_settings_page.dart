import 'package:flutter/material.dart';
import 'package:robot_frontend/pages/robot_status_page.dart';
import 'package:robot_frontend/pages/setting_pages/cleaning_page.dart';
import 'package:robot_frontend/pages/setting_pages/screen_settings_page.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class MoreSettingsPage extends StatelessWidget {
  const MoreSettingsPage({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Weitere Einstellungen',
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Row(
          children: [
            Expanded(
              child: Column(
                children: [
                  Expanded(
                    child: CustomButtonView(
                      text: 'Bildschirm',
                      onPressed: () {
                        Navigator.push(
                          context,
                          MaterialPageRoute<ScreenSettingsPage>(
                            builder: (context) => const ScreenSettingsPage(),
                          ),
                        );
                      },
                    ),
                  ),
                  const SizedBox(
                    height: 16,
                  ),
                  Expanded(
                    child: CustomButtonView(
                      text: 'Reinigungsmodus',
                      onPressed: () {
                        Navigator.push(
                          context,
                          MaterialPageRoute<CleaningPage>(
                            builder: (context) => const CleaningPage(),
                          ),
                        );
                      },
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              child: Column(
                children: [
                  Expanded(
                    child: CustomButtonView(
                      text: 'Roboterzustand',
                      onPressed: () {
                        Navigator.push(
                          context,
                          MaterialPageRoute(
                            builder: (context) => const RobotStatusPage(),
                          ),
                        );
                      },
                    ),
                  ),
                  const SizedBox(
                    height: 16,
                  ),
                  Expanded(
                    child: CustomButtonView(
                      text: 'Spracheinstellungen',
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
