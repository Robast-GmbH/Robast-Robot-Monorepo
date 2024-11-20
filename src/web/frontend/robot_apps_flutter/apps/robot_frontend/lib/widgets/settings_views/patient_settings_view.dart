import 'package:flutter/material.dart';
import 'package:robot_frontend/pages/setting_pages/config_page.dart';
import 'package:robot_frontend/pages/setting_pages/screen_settings_page.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';

class PatientSettingsView extends StatelessWidget {
  const PatientSettingsView({super.key});

  @override
  Widget build(BuildContext context) {
    return Row(
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
                      MaterialPageRoute<ConfigPage>(
                        builder: (context) => const ScreenSettingsPage(),
                      ),
                    );
                  },
                ),
              ),
              const SizedBox(height: 16),
              Expanded(
                child: CustomButtonView(
                  text: 'Platzhalter',
                  onPressed: () {},
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
                  text: 'Platzhalter',
                  onPressed: () {},
                ),
              ),
              const SizedBox(height: 16),
              Expanded(
                child: CustomButtonView(
                  text: 'Platzhalter',
                  onPressed: () {},
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }
}
