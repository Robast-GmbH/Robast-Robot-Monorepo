import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';

class WeatherView extends StatelessWidget {
  const WeatherView({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomButtonView(
      text: 'Wetter',
      onPressed: () {},
      content: Center(
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            buildWeatherInfo(title: 'Do.', icon: Icons.cloud, highestTemperature: 16, lowestTemperature: 6),
            const SizedBox(width: 32),
            buildWeatherInfo(title: 'Fr.', icon: Icons.cloud, highestTemperature: 17, lowestTemperature: 7),
            const SizedBox(width: 32),
            buildWeatherInfo(title: 'Sa.', icon: Icons.cloud, highestTemperature: 17, lowestTemperature: 8),
          ],
        ),
      ),
    );
  }

  Widget buildWeatherInfo({required String title, required IconData icon, required int highestTemperature, required int lowestTemperature}) {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: [
        Text(
          title,
          style: const TextStyle(color: RobotColors.secondaryText, fontSize: 32),
        ),
        const SizedBox(
          height: 8,
        ),
        Icon(icon, size: 100),
        const SizedBox(
          height: 8,
        ),
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(
              '$highestTemperature°',
              style: const TextStyle(color: RobotColors.secondaryText, fontSize: 32),
            ),
            const SizedBox(
              width: 16,
            ),
            Text(
              '$lowestTemperature°',
              style: const TextStyle(color: RobotColors.tertiaryText, fontSize: 32),
            ),
          ],
        ),
      ],
    );
  }
}
