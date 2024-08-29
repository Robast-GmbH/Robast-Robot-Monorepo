import 'dart:io';

import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

class BrightnessSlider extends StatefulWidget {
  const BrightnessSlider({super.key});

  @override
  State<BrightnessSlider> createState() => _BrightnessSliderState();
}

class _BrightnessSliderState extends State<BrightnessSlider> {
  final minBrigthness = 0.4;
  double _brightness = 0.5; // Initial brightness value
  late Future<void> loadInitialBrightness;

  Future<void> _setBrightness(double value) async {
    _brightness = value < minBrigthness ? minBrigthness : value;
    await setDisplayBrigthness();
    setState(() {});
  }

  Future<bool> setDisplayBrigthness() async {
    try {
      final screenInfo = await Process.run('bash', ['-c', 'xrandr -q | grep " connected"']);
      final screenName = screenInfo.stdout.toString().split(' ')[0];
      final result = await Process.run('xrandr', ['--output', screenName, '--brightness', _brightness.toString()]);
      return result.exitCode == 0;
    } catch (e) {
      return false;
    }
  }

  Future<void> initDisplayBrightness() async {
    try {
      final screenInfo = await Process.run('bash', ['-c', r'xrandr --verbose | awk "/Brightness/ { print $2; exit }"']);
      final infos = screenInfo.stdout.toString().split(' ');
      _brightness = double.parse(infos.last);
    } catch (e) {
      _brightness = minBrigthness;
    }
    setState(() {});
  }

  @override
  void initState() {
    super.initState();
    loadInitialBrightness = initDisplayBrightness();
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: FutureBuilder<void>(
        future: loadInitialBrightness,
        builder: (context, snapshot) {
          if (snapshot.connectionState != ConnectionState.done) {
            return const CircularProgressIndicator();
          }
          return Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              const Text(
                'Helligkeit anpassen:',
                style: TextStyle(fontSize: 24, color: RobotColors.primaryText),
              ),
              Slider(
                thumbColor: RobotColors.accent,
                activeColor: RobotColors.accent,
                inactiveColor: RobotColors.secondaryBackground,
                value: _brightness,
                divisions: 5,
                label: _brightness.toStringAsFixed(1),
                onChanged: _setBrightness,
              ),
              const SizedBox(height: 16),
              Text(
                'Aktuelle Helligkeit: ${(_brightness * 100).toInt()}%',
                style: const TextStyle(fontSize: 20, color: RobotColors.primaryText),
              ),
            ],
          );
        },
      ),
    );
  }
}
