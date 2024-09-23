import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/brightness_slider.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class ScreenSettingsPage extends StatefulWidget {
  const ScreenSettingsPage({super.key});

  @override
  State<ScreenSettingsPage> createState() => _ScreenSettingsPageState();
}

class _ScreenSettingsPageState extends State<ScreenSettingsPage> {
  @override
  Widget build(BuildContext context) {
    return const CustomScaffold(
      title: 'Bildschirm',
      child: Padding(
        padding: EdgeInsets.all(128),
        child: BrightnessSlider(),
      ),
    );
  }
}
