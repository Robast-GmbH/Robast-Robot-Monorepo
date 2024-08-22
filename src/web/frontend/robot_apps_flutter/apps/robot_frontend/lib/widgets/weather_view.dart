import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';

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
            buildWeatherInfo(title: 'Mi.', icon: Icons.sunny, highestTemperature: 24, lowestTemperature: 16),
            const SizedBox(width: 32),
            buildWeatherInfo(title: 'Do.', icon: Icons.sunny, highestTemperature: 22, lowestTemperature: 13),
            const SizedBox(width: 32),
            buildWeatherInfo(title: 'Fr.', icon: Icons.cloud, highestTemperature: 19, lowestTemperature: 11),
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
          style: const TextStyle(color: Colors.white70, fontSize: 32),
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
              style: const TextStyle(color: Colors.white70, fontSize: 32),
            ),
            const SizedBox(
              width: 16,
            ),
            Text(
              '$lowestTemperature°',
              style: const TextStyle(color: Colors.white38, fontSize: 32),
            ),
          ],
        ),
      ],
    );
  }
}
