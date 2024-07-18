import 'dart:io';

import 'package:flutter/material.dart';

class BrightnessSlider extends StatefulWidget {
  const BrightnessSlider({super.key});

  @override
  State<BrightnessSlider> createState() => _BrightnessSliderState();
}

class _BrightnessSliderState extends State<BrightnessSlider> {
  double _brightness = 0.5; // Initial brightness value
  late Future<void> loadInitialBrightness;

  Future<void> _setBrightness(double value) async {
    _brightness = value < 0.4 ? 0.4 : value;
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
      final screenInfo = await Process.run('bash', ['-c', 'xrandr --verbose | awk "/Brightness/ { print \$2; exit }"']);
      final infos = screenInfo.stdout.toString().split(' ');
      _brightness = double.parse(infos.last);
    } catch (e) {
      _brightness = 0.0;
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
                'Adjust Brightness:',
                style: TextStyle(fontSize: 20),
              ),
              Slider(
                value: _brightness,
                divisions: 5,
                label: _brightness.toStringAsFixed(1),
                onChanged: _setBrightness,
              ),
              const SizedBox(height: 16),
              Text(
                'Current Brightness: ${_brightness.toStringAsFixed(1)}',
                style: const TextStyle(fontSize: 16),
              ),
            ],
          );
        },
      ),
    );
  }
}
