import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';
import 'package:robot_frontend/widgets/delivery_plan_view.dart';
import 'package:robot_frontend/widgets/buttons/rounded_button.dart';
import 'package:robot_frontend/widgets/weather_view.dart';
import 'package:robot_frontend/widgets/welcome_view.dart';

class PatientHomeView extends StatelessWidget {
  const PatientHomeView({super.key});

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
                    const Expanded(child: WeatherView()),
                    const SizedBox(width: 16),
                    Expanded(
                      child: CustomButtonView(
                        text: 'Aktivitäten',
                        onPressed: () {},
                        content: Column(
                          crossAxisAlignment: CrossAxisAlignment.stretch,
                          children: [
                            const SizedBox(height: 16),
                            Expanded(
                              child: RoundedButton(
                                onPressed: () {},
                                color: Colors.black.withOpacity(0.1),
                                child: const Text(
                                  'Essen bestellen',
                                  style: TextStyle(fontSize: 32, color: RobotColors.secondaryText),
                                ),
                              ),
                            ),
                            const SizedBox(height: 16),
                            Expanded(
                              child: RoundedButton(
                                onPressed: () {},
                                color: Colors.black.withOpacity(0.1),
                                child: const Text(
                                  'Getränke bestellen',
                                  style: TextStyle(fontSize: 32, color: RobotColors.secondaryText),
                                ),
                              ),
                            ),
                            const SizedBox(height: 16),
                            Expanded(
                              child: RoundedButton(
                                onPressed: () {},
                                color: Colors.red.withOpacity(0.4),
                                child: const Text(
                                  'Pflegepersonal rufen',
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
          child: DeliveryPlanView(),
        ),
      ],
    );
  }
}
