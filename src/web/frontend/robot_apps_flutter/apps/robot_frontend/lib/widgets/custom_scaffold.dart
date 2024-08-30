import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/background_view.dart';
import 'package:robot_frontend/widgets/titled_view.dart';

class CustomScaffold extends StatelessWidget {
  const CustomScaffold({
    super.key,
    this.child = const SizedBox(),
    this.showBackButton = true,
    this.inactivityTimerEnabled = true,
    this.onBackButtonPressed,
    this.title = '',
  });

  final Widget child;
  final bool showBackButton;
  final bool inactivityTimerEnabled;
  final VoidCallback? onBackButtonPressed;
  final String title;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: BackgroundView(
        inactivityTimerEnabled: inactivityTimerEnabled,
        child: TitledView(
          title: title,
          showBackButton: showBackButton,
          onBackButtonPressed: onBackButtonPressed,
          child: child,
        ),
      ),
    );
  }
}
