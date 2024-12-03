import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/disinfection_view.dart';

class DisinfectionPage extends StatelessWidget {
  const DisinfectionPage({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      inactivityTimerEnabled: false,
      showBackButton: false,
      collapsedTitle: true,
      child: DisinfectionView(onDisinfection: () => Navigator.pop(context)),
    );
  }
}
