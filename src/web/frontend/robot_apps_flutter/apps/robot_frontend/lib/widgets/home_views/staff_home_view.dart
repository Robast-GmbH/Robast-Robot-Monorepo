import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/pages/content_distribution_task_creation_page.dart';
import 'package:robot_frontend/pages/delivery_task_creation_page.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/menu_modules_overview.dart';
import 'package:robot_frontend/widgets/rounded_button.dart';
import 'package:robot_frontend/widgets/weather_view.dart';
import 'package:robot_frontend/widgets/welcome_view.dart';

class StaffHomeView extends StatelessWidget {
  const StaffHomeView({super.key});

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(
          flex: 2,
          child: Column(
            children: [
              const Expanded(
                child: WelcomeView(),
              ),
              const SizedBox(height: 16),
              Expanded(
                child: Row(
                  children: [
                    const Expanded(
                      child: WeatherView(),
                    ),
                    const SizedBox(
                      width: 16,
                    ),
                    Expanded(
                      child: CustomButtonView(
                        text: 'Auftrag erstellen',
                        onPressed: () {},
                        content: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            const SizedBox(height: 16),
                            Expanded(
                              child: RoundedButton(
                                onPressed: () {
                                  Navigator.push(context, MaterialPageRoute<DeliveryTaskCreationPage>(builder: (context) => const DeliveryTaskCreationPage()));
                                },
                                child: const Text(
                                  'Abholen und Liefern',
                                  style: TextStyle(fontSize: 32, color: RobotColors.secondaryText),
                                ),
                              ),
                            ),
                            const SizedBox(height: 16),
                            Expanded(
                              child: RoundedButton(
                                onPressed: () {
                                  Navigator.push(
                                    context,
                                    MaterialPageRoute<ContentDistributionTaskCreationPage>(builder: (context) => const ContentDistributionTaskCreationPage()),
                                  );
                                },
                                child: const Text(
                                  'Beladen und Liefern',
                                  style: TextStyle(fontSize: 32, color: RobotColors.secondaryText),
                                ),
                              ),
                            ),
                            const SizedBox(height: 16),
                            Expanded(
                              child: RoundedButton(
                                onPressed: () {},
                                child: const Text(
                                  'Benutzerdefiniert',
                                  style: TextStyle(fontSize: 32, color: RobotColors.secondaryText),
                                ),
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
        const SizedBox(width: 16),
        const Expanded(
          child: Column(
            children: [
              Expanded(
                child: MenuModulesOverview(),
              ),
            ],
          ),
        ),
      ],
    );
  }
}
